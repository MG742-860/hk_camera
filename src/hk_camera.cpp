//
// Created by zihan on 2022/6/2.
//
#include <pluginlib/class_list_macros.h>
#include <hk_camera.h>
#include <utility>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>

namespace hk_camera
{
    PLUGINLIB_EXPORT_CLASS(hk_camera::HKCameraNodelet, nodelet::Nodelet)

    namespace
    {
        // 相机参数的统一命名空间：<node_ns>/hk_camera，与 config/*.yaml 的顶层键 hk_camera 对齐。
        // 多相机时每台相机在自己节点命名空间下各加载一份 yaml，本键名保持不变。
        const char* const kParamNamespace = "hk_camera";

        std::string joinNames(const std::vector<std::string>& names)
        {
            std::string out;
            for (size_t i = 0; i < names.size(); ++i)
            {
                if (i != 0) out += ", ";
                out += names[i];
            }
            return out;
        }

        // processFrame() 热路径的 RAII 守卫（165fps，绝不持锁）：
        //   构造：登记"在途回调"(+1) → 读 closing_ → 读句柄快照（closing_ 已置位则快照为 nullptr）
        //   析构：注销在途(-1)，使 closeDevice() 里的 drainFrameCallbacks() 得以推进
        // 与 closeDevice() 的固定约定：closing_=true → 等在途归零 → StopGrabbing / 注销图像回调 /
        // CloseDevice → DestroyHandle → 清空快照。因此只要本守卫取到的快照非空，
        // 该句柄在整个守卫生命周期内一定有效（不会被销毁）。
        class FrameCallbackGuard
        {
        public:
            FrameCallbackGuard(std::atomic<bool>& closing, std::atomic<int>& inflight,
                               std::atomic<void*>& handle_snapshot)
                : inflight_(inflight)
            {
                inflight_.fetch_add(1, std::memory_order_acq_rel);
                // 先登记再看 closing_：若关闭流程已经开始（或恰在本行前后开始），一律不取句柄，
                // 计数由析构函数归还（写者的等待循环依赖它归零）。
                if (!closing.load(std::memory_order_acquire))
                    handle_ = handle_snapshot.load(std::memory_order_acquire);
            }

            ~FrameCallbackGuard() { inflight_.fetch_sub(1, std::memory_order_acq_rel); }

            FrameCallbackGuard(const FrameCallbackGuard&) = delete;
            FrameCallbackGuard& operator=(const FrameCallbackGuard&) = delete;

            void* handle() const { return handle_; }

        private:
            std::atomic<int>& inflight_;
            void* handle_{nullptr};
        };

        // 从枚举结果里直接取 SN / 型号（官方 ReconnectDemo 的配对写法）：
        // 不需要打开设备就能配对，避免"为了读 SN 去 OpenDevice 别人的相机"——
        // 双实例时那会对另一台正在取流的相机产生 0x80000203 干扰，并且无谓地开关对端相机。
        // 注意：本包编译用的是 include/libMVSapi 下的旧版 SDK 头文件，其传输层宏只有
        // MV_GIGE_DEVICE(0x1)/MV_USB_DEVICE(0x4)/MV_CAMERALINK_DEVICE(0x8)，没有 GENTL 系列；
        // 而 initializeCamera 只枚举 MV_GIGE_DEVICE|MV_USB_DEVICE，故这两种之外一律返回空串。
        std::string deviceSerialNumber(const MV_CC_DEVICE_INFO* pDeviceInfo)
        {
            if (pDeviceInfo == nullptr) return std::string();
            switch (pDeviceInfo->nTLayerType)
            {
            case MV_USB_DEVICE:
                return std::string(reinterpret_cast<const char*>(pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber));
            case MV_GIGE_DEVICE:
                return std::string(reinterpret_cast<const char*>(pDeviceInfo->SpecialInfo.stGigEInfo.chSerialNumber));
            default:
                return std::string();
            }
        }

        std::string deviceModelName(const MV_CC_DEVICE_INFO* pDeviceInfo)
        {
            if (pDeviceInfo == nullptr) return std::string();
            switch (pDeviceInfo->nTLayerType)
            {
            case MV_USB_DEVICE:
                return std::string(reinterpret_cast<const char*>(pDeviceInfo->SpecialInfo.stUsb3VInfo.chModelName));
            case MV_GIGE_DEVICE:
                return std::string(reinterpret_cast<const char*>(pDeviceInfo->SpecialInfo.stGigEInfo.chModelName));
            default:
                return std::string();
            }
        }
    } // namespace

    HKCameraNodelet::HKCameraNodelet() = default;

    void HKCameraNodelet::onInit()
    {
        nh_ = this->getPrivateNodeHandle();
        node_name_ = nh_.getNamespace();
        // 参数命名空间与 config/*.yaml 的顶层键 hk_camera 对齐：
        // 节点私有命名空间是 /hk_camera（即 nodelet 名称），yaml 顶层键 hk_camera 正好落在
        // /hk_camera/hk_camera，故此前 nh_ 读不到 yaml 里的 11 个参数（退化成代码默认值）。
        params_nh_ = ros::NodeHandle(nh_, kParamNamespace);
        image_transport::ImageTransport it(nh_);
        pub_ = it.advertiseCamera("image_raw", 1);
        d_pub_ = it.advertise("image_raw_down", 1);
        this->status_change_srv_ = nh_.advertiseService("exposure_status_switch", &HKCameraNodelet::changeStatusCB,
                                                        this);

        // get param：一律经 getParamAligned 读取（优先 <ns>/hk_camera/，回退 <ns>/）
        {
            getParamAligned("camera_frame_id", image_.header.frame_id, std::string("camera_optical_frame"));
            getParamAligned("camera_name", camera_name_, std::string("camera"));
            getParamAligned("camera_info_url", camera_info_url_, std::string(""));
            getParamAligned("image_width", image_width_, 1440);
            getParamAligned("image_height", image_height_, 1080);
            getParamAligned("image_offset_x", image_offset_x_, 0);
            getParamAligned("image_offset_y", image_offset_y_, 0);
            // 默认值改成 RG8：getPixelFormat() 只认 RG8/RG10/RG12/mono8/mono10，
            // 旧默认 "bgr8" 会让"参数漏配"直接走到 else 分支报 illegal pixel format。
            getParamAligned("pixel_format", pixel_format_, std::string("RG8"));
            // frame_id 默认留空：留空表示"沿用 camera_frame_id"，避免默认值把 camera_frame_id 覆盖掉
            // （double_device 里每台相机的 camera_frame_id 不同，被覆盖会导致右目图像挂错坐标系）。
            getParamAligned("frame_id", frame_id_, std::string(""));
            getParamAligned("camera_sn", camera_sn_, std::string(""));
            getParamAligned("frame_rate", frame_rate_, 200.0);
            getParamAligned("sleep_time", sleep_time_, 0);
            getParamAligned("gain_value", gain_value_, 15.0);
            getParamAligned("gain_auto", gain_auto_, false);
            getParamAligned("gain_min", gain_min_, 0.0);
            getParamAligned("gain_max", gain_max_, 16.0);
            getParamAligned("gamma_selector", gamma_selector_, 2);
            getParamAligned("gamma_value", gamma_value_, 0.5);
            getParamAligned("exposure_auto", exposure_auto_, false);
            getParamAligned("exposure_value", exposure_value_, 100.0);
            getParamAligned("exposure_value_windmill", exposure_value_windmill_, 2000.0);
            getParamAligned("exposure_max", exposure_max_, 3000);
            getParamAligned("exposure_min", exposure_min_, 50);
            getParamAligned("white_auto", white_auto_, true);
            getParamAligned("white_selector", white_selector_, 0);
            getParamAligned("enable_resolution", enable_resolution_, false);
            getParamAligned("resolution_ratio_width", resolution_ratio_width_, 1440);
            getParamAligned("resolution_ratio_height", resolution_ratio_height_, 1080);
            getParamAligned("stop_grab", stop_grab_, false);
            // 抽帧参数是原子成员（reconfigCB 写 / 图像回调读），而 getParamAligned 需要"可写引用"，
            // 因此先落到局部变量再 store：参数来源自检逻辑(reportParamSources)不受影响。
            bool is_fps_down_param = false;
            getParamAligned("is_fps_down", is_fps_down_param, false);
            is_fps_down_.store(is_fps_down_param);
            double target_fps_param = 40.0;
            getParamAligned("target_fps", target_fps_param, 40.0);
            target_fps_.store(target_fps_param);
        }
        // 启动自检：把每个参数的实际来源与"同一参数两处定义且不一致"的情况打出来
        reportParamSources();

        // frame_id 归一：frame_id 显式配置时才覆盖 camera_frame_id，否则沿用 camera_frame_id。
        // 上游写法是读一次 camera_frame_id 到 image_.header.frame_id，再用默认值为
        // camera_optical_frame 的 frame_id_ 覆盖回去，于是 double_device 里设置的
        // camera_right_optical_frame 会失效（右目图像带着默认/左目坐标系发布）。
        if (frame_id_.empty()) frame_id_ = image_.header.frame_id;
        image_.header.frame_id = frame_id_;
        ROS_INFO("frame_id in use: %s", frame_id_.c_str());

        info_manager_.reset(new camera_info_manager::CameraInfoManager(nh_, camera_name_, camera_info_url_));

        // check for default camera info
        if (!info_manager_->isCalibrated())
        {
            info_manager_->setCameraName(camera_name_);
            sensor_msgs::CameraInfo camera_info;
            camera_info.header.frame_id = image_.header.frame_id;
            camera_info.width = image_width_;
            camera_info.height = image_height_;
            info_manager_->setCameraInfo(camera_info);
        }
        ROS_INFO("Starting '%s' at %dx%d", camera_name_.c_str(), image_width_, image_height_);
        info_ = std::move(info_manager_->getCameraInfo());
        // 多相机时下列id要改
        // TODO：未来支持多相机时需要更改
        info_.header.frame_id = frame_id_;
        image_.header.frame_id = frame_id_;
        image_.height = image_height_;
        image_.width = image_width_;
        image_.step = image_width_ * 3;
        image_.data.resize(image_.height * image_.step);
        // 发布的数据始终是 processFrame() 里 SDK 转换出来的 BGR8：
        // pixel_format_ 是"相机侧"格式(RG8/Bayer)，不是消息编码，不能当 encoding 用。
        image_.encoding = "bgr8";
        image_.is_bigendian = 0;
        // image_.header.frame_id = frame_id_;
        image_buffer_size_ = image_.height * image_.step;
        img_ = new unsigned char[image_.height * image_.step];
        ROS_INFO("OnINit initializeCamera");

        ros::Time start = ros::Time::now();
        while (!initializeCamera() && ros::ok())
        {
            ros::Duration(1).sleep();
            if ((ros::Time::now() - start).toSec() > 10 || !ros::ok())
            {
                ROS_ERROR("Failed to initialize camera. reinitial at timerCB().");
                // 标记为失败，在timerCB函数中走 false 路线，取消强制绑定SN(单机时)
                first_init_ = false;
                break;
            }
        }

        // 相机切换/启停话题沿用上游的**全局名**（不是节点私有名）：
        //   - /camera_name 是全局约定：rm_referee 等包直接订阅 /camera_name 来显示当前相机，
        //     外部"切换到某台相机"的发布方也发在全局话题上；写成 nh_.subscribe("camera_name")
        //     会变成 /hk_camera/camera_name，外部指令与别的包都收不到（上游 HEAD 就是全局名）。
        //   - 多实例(double_device)时两个实例都会收到同一条指令：被点名的实例恢复取流，
        //     其余实例停止取流 —— 这正是相机切换的语义。若要单独控制某台，用 remap 改名。
        camera_change_sub = nh_.subscribe("/camera_name", 50, &hk_camera::HKCameraNodelet::cameraChange, this);
        camera_stop_sub_ = nh_.subscribe("/camera_stop", 50, &hk_camera::HKCameraNodelet::cameraStop, this);

        timer_ = nh_.createTimer(ros::Duration(1), &HKCameraNodelet::timerCallback, this);

        ros::NodeHandle p_nh(nh_, "hk_camera_reconfig");

        if (enable_resolution_ && resolution_ratio_width_ > 0 && resolution_ratio_height_ > 0)
            pub_rect_ = p_nh.advertise<sensor_msgs::Image>("/image_rect", 1);
        else
            enable_resolution_ = false;

        srv_ = new dynamic_reconfigure::Server<CameraConfig>(p_nh);
        dynamic_reconfigure::Server<CameraConfig>::CallbackType cb = boost::bind(
            &HKCameraNodelet::reconfigCB, this, _1, _2);
        srv_->setCallback(cb);

        ROS_INFO("Camera %s is ready", camera_name_.c_str());
    }

    // 相机侧取流格式名 → SDK 枚举。只映射本相机(MV-CS016-10UC)实际会用的几种：
    //   RG8(BayerRG8) 165.369fps ← 现用；mono8 165.369fps；YUV422 118.840fps；RGB8/BGR8 79.227fps
    // 故意**不**支持 "bgr8"/"rgb8"：那是节点输出消息的编码（image_.encoding 恒为 bgr8），
    // 不是相机取流格式；若照字面设成 BGR8，帧率会从 165 掉到 79，赛场不可接受。
    // 传不认识的名字一律按 RG8 处理并告警（打印实际收到的名字，便于揪出 yaml 笔误）。
    MvGvspPixelType HKCameraNodelet::getPixelFormat(const std::string& pixelformat)
    {
        if (pixelformat == "mono8")
            return PixelType_Gvsp_Mono8;
        else if (pixelformat == "mono10")
            return PixelType_Gvsp_Mono10;
        else if (pixelformat == "RG8")
            return PixelType_Gvsp_BayerRG8;
        else if (pixelformat == "RG10")
            return PixelType_Gvsp_BayerRG10;
        else if (pixelformat == "RG12")
            return PixelType_Gvsp_BayerRG12;
        else
        {
            ROS_ERROR("illegal pixel format:%s ! use default:RG8 (supported: mono8/mono10/RG8/RG10/RG12)",
                      pixelformat.c_str());
            return PixelType_Gvsp_BayerRG8;
        }
    }

    bool HKCameraNodelet::initializeCamera(const bool from_timerCB)
    {
        ROS_WARN("start initializeCamera");
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (nRet != MV_OK)
        {
            ROS_ERROR("initializeCamera() failed: MV_CC_EnumDevices failed:0x%08x", nRet);
            return false;
        }
        if (stDeviceList.nDeviceNum <= 0)
        {
            ROS_ERROR("initializaCamera() failed:stDeviceList.nDeviceNum <= 0,"
                "check physical connection or wait all camera ready.");
            return false;
        }

        // MV_CC_CreateHandle 的 handle 参数是 [IN][OUT]：传入非空句柄时 SDK 会直接覆盖旧句柄，
        // 旧句柄既不会 StopGrabbing、也不会注销回调/CloseDevice/DestroyHandle，结果是"自己占着自己"，
        // 之后 OpenDevice 会返回 0x80000203(设备无访问权限)。所以枚举配对前必须先彻底释放旧句柄。
        // releaseDevice() 自己持 dev_mutex_（内部判空），这里必须在**锁外**调用；下面的 sleep/枚举
        // （最长 sleep_time_ 秒 + USB 扫描）也刻意留在锁外，避免长时间占锁拖住配置/服务回调。
        releaseDevice(false);

        // 官方建议的释放顺序：停止取流 → 注销回调 → 关闭设备 → 销毁句柄
        auto release_handle = [](void* handle)
        {
            if (handle == nullptr) return;
            MV_CC_StopGrabbing(handle); // 未取流时会返回错误码，忽略即可
            MV_CC_RegisterImageCallBackEx(handle, nullptr, nullptr); // 先摘回调，之后不会再有图像回调进来
            MV_CC_CloseDevice(handle);
            MV_CC_DestroyHandle(handle);
        };

        MVCC_STRINGVALUE dev_sn;
        memset(&dev_sn, 0, sizeof(MVCC_STRINGVALUE));

        // sleep_time: 启动时"等相机 ready"的固定延时。这里做范围夹紧，避免配置错误把初始化
        // 长时间阻塞住（本函数也会在 timerCallback 里被调用，长阻塞会拖慢重连节奏）。
        const int sleep_sec = std::min(std::max(sleep_time_, 0), 60);
        if (sleep_sec != sleep_time_)
            ROS_WARN("initializeCamera(): sleep_time %d out of range [0,60], clamp to %d", sleep_time_, sleep_sec);
        if (sleep_sec > 0) ros::Duration(sleep_sec).sleep();

        // 先把在线设备列出来：SN 对不上时一眼能看出"现在插着的是哪几台"
        for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            ROS_INFO("initializeCamera(): device[%u] SN:%s model:%s", i,
                     deviceSerialNumber(stDeviceList.pDeviceInfo[i]).c_str(),
                     deviceModelName(stDeviceList.pDeviceInfo[i]).c_str());
        }

        // 相机配对规则（与官方 ReconnectDemo 一致：先比对枚举结果里的 SN，命中才打开设备）：
        //  - 单机(nDeviceNum==1)且非重连：直接采用设备 SN（换相机后不需要改 launch/yaml）
        //  - 多机(nDeviceNum>1)或重连：必须与已记录的 camera_sn_ 严格配对，
        //    不允许"继续用别人的相机"，否则只有一台在线时两个实例会抢同一台设备
        int mis_match = 0;
        int match_index = -1;
        for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            const std::string dev_sn = deviceSerialNumber(stDeviceList.pDeviceInfo[i]);
            if (dev_sn.empty())
            {
                ROS_WARN("initializeCamera(): device[%u] SN is empty (nTLayerType:0x%08x), skip!", i,
                         stDeviceList.pDeviceInfo[i]->nTLayerType);
                mis_match++;
                continue;
            }
            if (stDeviceList.nDeviceNum == 1 && !from_timerCB)
            {
                // 如果是单相机，直接使用相机的SN而不是yaml中的SN
                camera_sn_ = dev_sn;
                ROS_INFO("initializeCamera(): single camera, camera SN now use:%s", camera_sn_.c_str());
            }
            else if (dev_sn == camera_sn_)
            {
                ROS_INFO("initializeCamera():find target %s!", camera_sn_.c_str());
            }
            else
            {
                // 只比对、不打开：不再为了读 SN 去 OpenDevice 别人的相机（双实例时会互相干扰）
                ROS_INFO("initializeCamera(): device[%u] SN:%s is not target SN:%s, skip!", i, dev_sn.c_str(),
                         camera_sn_.c_str());
                mis_match++;
                continue;
            }
            match_index = static_cast<int>(i);
            break;
        }

        if (match_index < 0)
        {
            ROS_ERROR("initializeCamera(): no camera matched (nDeviceNum:%u, mis_match:%d, target SN:%s). "
                      "Check camera_sn against the device list printed above.",
                      stDeviceList.nDeviceNum, mis_match, camera_sn_.c_str());
            return false;
        }

        // 从这里开始的"创建 → 打开 → 配置 → 注册回调 → 赋句柄 → 取流"全程持 dev_mutex_：
        // 期间 dev_handle_ 可能为空或指向半初始化的句柄，其它线程（配置/服务/订阅/析构）
        // 必须等锁，绝不允许拿旧句柄或半初始化句柄去调 SDK。
        std::unique_lock<std::mutex> dev_lock(dev_mutex_);

        // 只对命中的那一台创建句柄并打开
        void* handle = nullptr; // 局部句柄：配对+配置+取流全部成功后，才交给 dev_handle_
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[match_index]);
        if (nRet != MV_OK || handle == nullptr)
        {
            ROS_WARN("initializeCamera(): MV_CC_CreateHandle failed at device[%d]! Error code: 0x%08x", match_index,
                     nRet);
            return false;
        }
        nRet = MV_CC_OpenDevice(handle);
        if (nRet != MV_OK)
        {
            // 0x80000203 = 设备无访问权限，通常是设备已被其它进程/实例占用
            ROS_ERROR("initializeCamera(): MV_CC_OpenDevice failed at device[%d]! Error code: 0x%08x "
                      "(0x80000203 means the device is occupied by another process)",
                      match_index, nRet);
            MV_CC_DestroyHandle(handle); // 打开失败，无需 StopGrabbing/CloseDevice
            return false;
        }

        // 打开后再读一次 SN 做交叉校验：确认"命中的枚举项"就是"真正打开的设备"
        memset(&dev_sn, 0, sizeof(MVCC_STRINGVALUE));
        nRet = MV_CC_GetStringValue(handle, "DeviceSerialNumber", &dev_sn);
        if (nRet != MV_OK)
        {
            ROS_ERROR("initializeCamera(): get DeviceSerialNumber failed! Error code: 0x%08x", nRet);
            release_handle(handle);
            return false;
        }
        if (camera_sn_ != std::string(dev_sn.chCurValue))
            ROS_WARN("initializeCamera(): opened device SN:%s differs from target SN:%s!", dev_sn.chCurValue,
                     camera_sn_.c_str());
        ROS_INFO("Camera SN now use:%s", dev_sn.chCurValue);

        // 检查配置合法性
        if (!validateDimensions(handle, image_width_, image_height_, image_offset_x_, image_offset_y_))
        {
            release_handle(handle);
            return false;
        }

        // Retrieve and print the camera's model name using DeviceModelName
        MVCC_STRINGVALUE model_name;
        memset(&model_name, 0, sizeof(MVCC_STRINGVALUE));
        nRet = MV_CC_GetStringValue(handle, "DeviceModelName", &model_name);
        if (nRet == MV_OK)
        {
            // 相机型号
            ROS_INFO("Camera Model: %s", model_name.chCurValue);
        }
        else
        {
            ROS_WARN("Failed to get camera model name. Error code: 0x%08x", nRet);
            release_handle(handle);
            return false;
        }

        const MvGvspPixelType format = getPixelFormat(pixel_format_);

        ROS_INFO("Pixel Format: %s", pixel_format_.c_str());

        // ROI 节点之间有先后约束（只有先把 Width 缩小，OffsetX 才能增大），
        // 所以先归零 offset → 设 Width/Height → 再设目标 offset。
        // 断线重连后相机会回到默认 ROI，按这个顺序设置才能保证重连前后 ROI 完全一致。
        CHECK_MVS(MV_CC_SetIntValue(handle, "OffsetX", 0), false);
        CHECK_MVS(MV_CC_SetIntValue(handle, "OffsetY", 0), false);
        CHECK_MVS(MV_CC_SetIntValueEx(handle, "Width", image_width_), false);
        CHECK_MVS(MV_CC_SetIntValue(handle, "Height", image_height_), false);
        CHECK_MVS(MV_CC_SetIntValue(handle, "OffsetX", image_offset_x_), false);
        CHECK_MVS(MV_CC_SetIntValue(handle, "OffsetY", image_offset_y_), false);
        CHECK_MVS(MV_CC_SetEnumValue(handle, "ExposureAuto", 0), false);
        CHECK_MVS(MV_CC_SetFloatValue(handle, "ExposureTime" , static_cast<float>(exposure_value_)), false);
        CHECK_MVS(MV_CC_SetEnumValue(handle,"PixelFormat",format), false); // 每个设备都不同。单机/多机需要测试

        //  AcquisitionLineRate ,LineRate can't be set
        //  CHECK_MVS(MV_CC_SetBoolValue(handle,"AcquisitionLineRateEnable", true));
        //  CHECK_MVS(MV_CC_SetIntValue(handle,"AcquisitionLineRate", 10));

        _MVCC_FLOATVALUE_T frame_rate{0};
        if (MV_CC_SetFrameRate(handle, static_cast<float>(frame_rate_)) != MV_OK)
        {
            ROS_WARN("Failed to set targrt frame rate:%f", frame_rate_);
        }

        CHECK_MVS(MV_CC_GetFrameRate(handle, &frame_rate), false);

        ROS_INFO("Frame rate is: %f", frame_rate.fCurValue);

        CHECK_MVS(MV_CC_SetEnumValue(handle, "TriggerMode", 0), false);

        // 注册掉线异常回调（官方要求：MV_CC_OpenDevice 之后、MV_CC_StartGrabbing 之前调用）。
        // 设备掉线时会在 onExceptionCB 里收到 MV_EXCEPTION_DEV_DISCONNECT，
        // 由 timerCallback 按官方流程重连（停止取流 → 关闭设备 → 销毁句柄 → 重新枚举配对）。
        nRet = MV_CC_RegisterExceptionCallBack(handle, onExceptionCB, this);
        if (nRet != MV_OK)
        {
            // 拿不到掉线事件只会让重连慢一拍（还有 timerCallback 里 1Hz 的 MV_CC_IsDeviceConnected 兜底）
            ROS_WARN("initializeCamera(): MV_CC_RegisterExceptionCallBack failed! Error code: 0x%08x", nRet);
        }

        nRet = MV_CC_RegisterImageCallBackEx(handle, onFrameCB, this);
        if (nRet != MV_OK)
        {
            ROS_ERROR("initializeCamera(): MV_CC_RegisterImageCallBackEx failed! Error code: 0x%08x", nRet);
            release_handle(handle);
            return false;
        }

        // 句柄交给成员变量：图像回调里要用它做 MV_CC_ConvertPixelType，必须在 StartGrabbing 之前赋值
        // （setDevHandleLocked 会同时刷新热路径快照 dev_handle_snapshot_，见头文件注释）
        setDevHandleLocked(handle);
        need_grab_ = false;

        nRet = MV_CC_StartGrabbing(handle);
        if (nRet != MV_OK)
        {
            ROS_ERROR("Stream On failed! nRet: %d", nRet);
            dev_lock.unlock(); // releaseDevice() 自己持同一把锁，必须先解锁再调用（否则自锁死）
            releaseDevice(false);
            return false;
        }
        ROS_INFO("Stream On.");

        got_frame_ = false; // 重新开始计帧看门狗
        stream_on_sec_ = ros::WallTime::now().toSec();
        need_reconnect_ = false;
        reconnect_backoff_ms_ = 1000; // 重连成功，退避时间重置
        first_init_ = true;
        return true;
    }

    void HKCameraNodelet::timerCallback(const ros::TimerEvent&)
    {
        // 帧看门狗超时：首次出图允许 5s；已经出过图后不允许超过 2s 无帧（165fps 正常不会触发）
        const double first_frame_timeout_sec = 5.0;
        const double no_frame_timeout_sec = 2.0;

        // 掉线判定：异常回调置位 / 句柄为空 / 连接状态查询失败。
        // 本段持 dev_mutex_ 取句柄快照，保证 MV_CC_IsDeviceConnected() 用到的句柄不会在调用
        // 中途被别的线程销毁；has_handle 供后面"是否需要释放旧句柄"判断复用。
        bool dev_lost = true;
        bool has_handle = false;
        {
            std::lock_guard<std::mutex> dev_lock(dev_mutex_);
            has_handle = (dev_handle_ != nullptr);
            dev_lost = need_reconnect_ || !has_handle || !MV_CC_IsDeviceConnected(dev_handle_);
        }

        // 兜底判定：有些异常（实测 USB 总线复位后取流管道已断）设备仍报"已连接"，
        // 表现是图像停了但怎么查都查不出掉线，只能靠"长时间没有帧回调"来判定
        if (!dev_lost && has_handle && !need_grab_)
        {
            const double now_sec = ros::WallTime::now().toSec();
            const double last_active = got_frame_ ? last_frame_sec_ : stream_on_sec_; // 出过图比"最后一帧"，否则比"Stream On"
            const double timeout = got_frame_ ? no_frame_timeout_sec : first_frame_timeout_sec;
            if (last_active > 0.0 && now_sec - last_active > timeout)
            {
                dev_lost = true;
                ROS_WARN("timerCallback(): no frame for %.1fs (timeout %.1fs), force reconnect.",
                         now_sec - last_active, timeout);
            }
        }

        if (!dev_lost) return;
        if (reconnect_in_progress_) return; // 上一次重连还在进行，避免定时器叠加

        // 退避：失败后 1s → 2s → 4s → 5s(上限)，避免每秒都枚举设备/刷日志
        const ros::WallTime now = ros::WallTime::now();
        if (!next_reconnect_time_.isZero() && now < next_reconnect_time_) return;

        if (has_handle)
        {
            ROS_WARN("timerCallback(): camera lost, target SN:%s, reconnect now.", camera_sn_.c_str());
            need_reconnect_ = false;
            // 官方重连流程：重连前先停止取流、注销回调、关闭设备、销毁句柄，
            // 保证后面 MV_CC_CreateHandle 拿到的是 NULL（否则旧句柄泄漏、OpenDevice 报 0x80000203）
            releaseDevice(false);
        }
        else
        {
            ROS_WARN("timerCallback(): no valid handle, try to find target SN:%s.", camera_sn_.c_str());
        }

        // 如果从来没有初始化成功过(first_init_==false)，走 from_timerCB=false：
        // 单机场景允许直接采用设备 SN；一旦成功初始化过再掉线，就必须按记录的 SN 严格配对
        const bool from_timerCB = first_init_;

        reconnect_in_progress_ = true;
        const bool initialized = initializeCamera(from_timerCB);
        reconnect_in_progress_ = false;

        if (initialized)
        {
            reconnect_backoff_ms_ = 1000;
            next_reconnect_time_ = ros::WallTime::now() + ros::WallDuration(1.0);
            return;
        }

        reconnect_backoff_ms_ = std::min(reconnect_backoff_ms_ * 2, 5000);
        next_reconnect_time_ = ros::WallTime::now() + ros::WallDuration(reconnect_backoff_ms_ / 1000.0);
        ROS_WARN("timerCallback(): reconnect failed, retry after %d ms.", reconnect_backoff_ms_);
    }

    // 启动自检：打印每个参数的实际来源，并告警"同一参数在两处定义且取值不同"的情况。
    // 参数约定：统一写在 <node_ns>/hk_camera/ 下（即 config/*.yaml 的顶层键 hk_camera），
    // 多相机时每台相机在自己节点命名空间下各加载一份 yaml，键名不变。
    void HKCameraNodelet::reportParamSources()
    {
        static const char* const kNames[] = {
            "camera_frame_id", "camera_name", "camera_info_url", "image_width", "image_height",
            "image_offset_x", "image_offset_y", "pixel_format", "frame_id", "camera_sn",
            "frame_rate", "sleep_time", "gain_value", "gain_auto", "gain_min", "gain_max",
            "gamma_selector", "gamma_value",
            "exposure_auto", "exposure_value", "exposure_value_windmill", "exposure_max", "exposure_min",
            "white_auto", "white_selector", "enable_resolution", "resolution_ratio_width",
            "resolution_ratio_height", "stop_grab", "is_fps_down", "target_fps"
        };
        std::vector<std::string> from_aligned, from_legacy, from_default, conflicts;
        for (const char* name : kNames)
        {
            XmlRpc::XmlRpcValue v_aligned, v_legacy;
            const bool has_aligned = params_nh_.getParam(name, v_aligned);
            const bool has_legacy = nh_.getParam(name, v_legacy);
            if (has_aligned)
            {
                from_aligned.push_back(name);
                // 注：XmlRpcValue 的 == 对不同类型（如 1 与 1.0）会判为不等，此处宁可多报不漏报
                if (has_legacy && !(v_aligned == v_legacy))
                    conflicts.push_back(name);
            }
            else if (has_legacy)
            {
                from_legacy.push_back(name);
            }
            else
            {
                from_default.push_back(name);
            }
        }

        ROS_INFO("param namespace: %s (top-level key in config/*.yaml)", params_nh_.getNamespace().c_str());
        ROS_INFO("params from %s (%zu): %s", params_nh_.getNamespace().c_str(), from_aligned.size(),
                 joinNames(from_aligned).c_str());
        ROS_INFO("params from legacy %s (%zu): %s", nh_.getNamespace().c_str(), from_legacy.size(),
                 joinNames(from_legacy).c_str());
        ROS_INFO("params using code default (%zu): %s", from_default.size(), joinNames(from_default).c_str());
        if (!conflicts.empty())
            ROS_WARN("%zu param(s) defined in BOTH %s and %s with different values: %s | "
                     "%s wins (yaml), please delete the duplicated legacy definitions.",
                     conflicts.size(), params_nh_.getNamespace().c_str(), nh_.getNamespace().c_str(),
                     joinNames(conflicts).c_str(), params_nh_.getNamespace().c_str());
    }

    bool HKCameraNodelet::changeStatusCB(rm_msgs::StatusChange::Request& change, rm_msgs::StatusChange::Response& res)
    {
        res.switch_is_success = false;
        // 本回调跑在 ROS 服务线程，与重连线程（timerCallback）并发：持 dev_mutex_ 保证
        // 下面用到的句柄在本次调用期间不会被销毁。
        std::lock_guard<std::mutex> dev_lock(dev_mutex_);
        if (dev_handle_ == nullptr)
        {
            ROS_WARN("changeStatusCB(): camera not ready (dev_handle_ is null), reject switch.");
            return false;
        }
        // 曝光取值用局部变量 + 专用成员，不再写回 exposure_value_：
        // 原实现把参数读进 exposure_value_ 成员，切换一次状态就把它改写成 2000(风车) 或 20，
        // 而 initializeCamera() 里的"设曝光"用的正是这个成员 ⇒ 切换过状态后再掉线重连，
        // 相机会被设成 20us（几乎全黑）。风车模式曝光单独由 exposure_value_windmill 控制（yaml 可配）。
        // 注：本服务会把 ExposureAuto 置 OFF（上游行为），即切换后按 exposure_auto=false 工作。
        const double target_exposure = change.target ? exposure_value_windmill_ : exposure_value_;
        try
        {
            CHECK_MVS(MV_CC_SetEnumValue(dev_handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF), true);
            CHECK_MVS(MV_CC_SetFloatValue(dev_handle_, "ExposureTime", static_cast<float>(target_exposure)), true);
        }
        catch (std::runtime_error& e)
        {
            ROS_ERROR("changeStatusCB() error :%s", e.what());
            return false;
        }
        ROS_INFO("changeStatusCB(): exposure -> %.1f us (%s mode)", target_exposure,
                 change.target ? "windmill" : "normal");
        res.switch_is_success = true;
        return true;
    }

    void HKCameraNodelet::cameraChange(const std_msgs::String& camera_change)
    {
        // 订阅回调在 ROS spinner 线程执行，与重连线程并发：持 dev_mutex_ 保证句柄生命周期
        std::lock_guard<std::mutex> dev_lock(dev_mutex_);
        if (dev_handle_ == nullptr) return;
        try
        {
            // <manager> <nodelet> <...>
            // 只有消息目标是自己时才恢复抓流：node_name_ 为命名空间("/hk_camera")，
            // camera_name_ 为相机名("hk_camera")，两种写法都要认。
            // 不能只与 node_name_ 比较，否则"目标是自己"的消息会落进 else 分支把流停掉。
            const bool for_this_node =
                camera_change.data == node_name_ || camera_change.data == camera_name_;
            if (for_this_node)
            {
                if (need_grab_)
                {
                    CHECK_MVS(MV_CC_StartGrabbing(dev_handle_), true);
                    need_grab_ = false;
                }
            }
            else if (!need_grab_)
            {
                // 消息目标是别的相机时，才停止本相机抓流
                CHECK_MVS(MV_CC_StopGrabbing(dev_handle_), true);
                need_grab_ = true;
            }
        }
        catch (std::runtime_error& e)
        {
            ROS_ERROR("cameraChange() error: %s", e.what());
        }
    }

    void HKCameraNodelet::cameraStop(const std_msgs::Bool camera_stop_msg_)
    {
        // 订阅回调在 ROS spinner 线程执行，与重连线程并发：持 dev_mutex_ 保证句柄生命周期
        std::lock_guard<std::mutex> dev_lock(dev_mutex_);
        if (dev_handle_ == nullptr) return;
        try
        {
            // 停止/恢复相机抓流：话题为**全局** /camera_stop（与上游一致，见 onInit 里的说明）。
            // 收到即表示该命令是发给本节点的，不能再与写死的 "camera_back" 比较
            // （node_name_ 现在是 "/hk_camera"，原写法会让该启停命令永远失效）。
            // 多实例时两个实例都会收到：若只停/启某一台，请对各节点 remap 该话题。
            if (camera_stop_msg_.data == true && !need_grab_)
            {
                ROS_WARN("cameraStop(): stop grabbing by command.");
                CHECK_MVS(MV_CC_StopGrabbing(dev_handle_), true);
                need_grab_ = true;
            }
            if (camera_stop_msg_.data != true && need_grab_)
            {
                ROS_WARN("cameraStop(): start grabbing by command.");
                CHECK_MVS(MV_CC_StartGrabbing(dev_handle_), true);
                need_grab_ = false;
            }
        }
        catch (std::runtime_error& e)
        {
            ROS_ERROR("cameraStop() error: %s", e.what());
        }
    }

    void HKCameraNodelet::onFrameCB(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser)
    {
        try
        {
            auto* self = static_cast<HKCameraNodelet*>(pUser);
            if (self != nullptr)
            {
                // 帧看门狗时间戳：本回调被调用即说明 SDK 取流管道还是活的
                self->got_frame_ = true;
                self->last_frame_sec_ = ros::WallTime::now().toSec();
                self->processFrame(pData, pFrameInfo);
            }
        }
        catch (const std::exception& e)
        {
            ROS_ERROR("onFrameCB(): exception in onFrameCB: %s", e.what());
        }
        catch (...)
        {
            ROS_ERROR("onFrameCB(): unknown exception in onFrameCB.");
        }
    }

    void __stdcall HKCameraNodelet::onExceptionCB(unsigned int nMsgType, void* pUser)
    {
        auto* self = static_cast<HKCameraNodelet*>(pUser);
        if (self == nullptr) return;
        if (nMsgType == MV_EXCEPTION_DEV_DISCONNECT)
        {
            // 本函数运行在 SDK 内部线程，只做标记 + 日志：
            // 真正的重连动作(停止取流→关闭设备→销毁句柄→重新枚举配对)统一交给 timerCallback 执行，
            // 避免在 SDK 回调线程里做阻塞式 SDK 调用带来的死锁/重入风险。
            self->need_reconnect_ = true;
            ROS_WARN("onExceptionCB(): device disconnected! wait timerCallback to reconnect.");
        }
    }

    void HKCameraNodelet::processFrame(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo)
    {
        if (pData == nullptr || pFrameInfo == nullptr)
        {
            ROS_WARN("processFrame(): Garb image failed from onFramCB!");
            return;
        }
        // 不再逐帧调用 MV_CC_IsDeviceConnected()：200fps 下是纯额外开销，而且会与重连线程的
        // DestroyHandle 抢用句柄。掉线改由 onExceptionCB 置位 + timerCallback 的 1Hz 轮询判定。
        // 热路径无锁：FrameCallbackGuard 负责"登记在途回调 + 取句柄快照"（见其类注释），
        // 本函数后续一律使用这个快照，不再直接读 dev_handle_（return 时守卫析构自动注销在途）。
        const FrameCallbackGuard frame_guard(closing_, frame_callbacks_inflight_, dev_handle_snapshot_);
        void* const dev_handle = frame_guard.handle();
        if (dev_handle == nullptr) return;
        const uint32_t width = pFrameInfo->nWidth;
        const uint32_t height = pFrameInfo->nHeight;
        if (width == 0 || height == 0)
        {
            ROS_WARN("processFrame(): invalid width or height!");
            return;
        }

        // 容量检查
        if (!ensureBufferLocked(width, height)) return;

        MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
        // Top to bottom are：image width, image height, input data buffer, input data size, source pixel format,
        // destination pixel format, output data buffer, provided output buffer size
        stConvertParam.nWidth = pFrameInfo->nWidth;
        stConvertParam.nHeight = pFrameInfo->nHeight;
        stConvertParam.pSrcData = pData;
        stConvertParam.nSrcDataLen = pFrameInfo->nFrameLen;
        stConvertParam.enSrcPixelType = pFrameInfo->enPixelType;
        stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
        stConvertParam.pDstBuffer = img_;
        stConvertParam.nDstBufferSize = static_cast<unsigned int>(
            std::min<uint64_t>(static_cast<uint64_t>(image_buffer_size_), 0xFFFFFFFFULL));

        int nRet = MV_CC_ConvertPixelType(dev_handle, &stConvertParam);
        if (nRet != MV_OK)
        {
            ROS_WARN("processFrame(): failed to convert pixel type: 0x%08x", nRet);
            return;
        }

        // buffer size check
        if (stConvertParam.nDstLen == 0 || stConvertParam.nDstLen > image_buffer_size_)
        {
            ROS_WARN("converted size %u excced buffer size %zu,drop this frame.", stConvertParam.nDstLen,
                     image_buffer_size_);
            return;
        }

        // 同步时间戳
        const ros::Time stamp = ros::Time::now();
        image_.header.stamp = stamp;
        image_.header.seq++;
        info_.header.stamp = stamp;
        image_.width = width;
        image_.height = height;
        image_.step = width * 3;
        image_.encoding = "bgr8";
        image_.is_bigendian = 0;
        info_.width = width;
        info_.height = height;
        info_.header.seq = image_.header.seq;

        if (image_.data.size() < stConvertParam.nDstLen) image_.data.resize(stConvertParam.nDstLen);
        memcpy(&image_.data[0], img_, stConvertParam.nDstLen);
        pub_.publish(image_, info_);

        // 缩放发布：有人订阅才发布
        if (enable_resolution_ && pub_rect_.getNumSubscribers() > 0)
        {
            if (resolution_ratio_width_ <= 0 || resolution_ratio_height_ <= 0)
            {
                ROS_ERROR("processFrame(): res failed by invaild ratio %d  %d", resolution_ratio_width_,
                          resolution_ratio_height_);
                return;
            }
            try
            {
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_, "bgr8");
                cv::Mat cv_img;
                cv_ptr->image.copyTo(cv_img);
                cv::resize(cv_img, cv_img, cvSize(resolution_ratio_width_, resolution_ratio_height_));
                // image_rect 继承时间戳与坐标系，而非空 Header。
                pub_rect_.publish(cv_bridge::CvImage(image_.header, "bgr8", cv_img).toImageMsg());
            }
            catch (std::exception& e)
            {
                ROS_ERROR("processFrame(): res exception: %s", e.what());
            }
        }

        // 抽帧参数是原子成员：热路径各取一次快照（RELAXED 足够，只是发布节流判断，
        // 与句柄关闭/重连的同步无关），避免反复读原子
        const bool fps_down = is_fps_down_.load(std::memory_order_relaxed);
        const double target_fps = target_fps_.load(std::memory_order_relaxed);
        if (fps_down)
        {
            bool pub_down_sample = false;
            const ros::WallTime now = ros::WallTime::now();
            const ros::WallDuration interval(1.0 / std::max(target_fps, 1.0));
            {
                std::lock_guard<std::mutex> fps_lock(fps_down_mutex_);
                if (next_pub_time_.isZero())
                    next_pub_time_ = now;
                if (now >= next_pub_time_)
                {
                    do
                        next_pub_time_ += interval;
                    while (next_pub_time_ <= now);
                    pub_down_sample = true;
                }
            }
            if (pub_down_sample && d_pub_.getNumSubscribers() > 0)
                d_pub_.publish(image_);
        }
    }


    // 目前是根据实时图像进行扩容，但是相机最大分辨率和平时的参数一样:1440*1080
    // 可以修改此函数以保证使用的大小最终与相机或参数的一致
    bool HKCameraNodelet::ensureBufferLocked(const uint32_t width, const uint32_t height)
    {
        const uint64_t needed =
            static_cast<uint64_t>(width) * static_cast<uint64_t>(height) * 3ULL;
        if (needed == 0)
            return false;
        if (img_ != nullptr && image_buffer_size_ >= needed)
            return true;

        const auto new_buffer = new(std::nothrow) unsigned char[needed];
        if (new_buffer == nullptr)
        {
            ROS_ERROR_THROTTLE(1.0, "realloc image buffer %llu bytes failed.",
                               static_cast<unsigned long long>(needed));
            return false;
        }
        delete[] img_;
        img_ = new_buffer;
        image_buffer_size_ = static_cast<size_t>(needed);
        ROS_WARN("image buffer enlarged to %llu bytes for frame %ux%u",
                 static_cast<unsigned long long>(needed), width, height);
        return true;
    }

    void HKCameraNodelet::reconfigCB(CameraConfig& config, uint32_t level)
    {
        // dynamic_reconfigure 回调外壳：唯一职责是把异常挡在 ROS 回调边界之内。
        // 本包所有 CHECK_MVS 都是 warn 模式（不抛），但 SDK 调用/字符串处理仍可能抛；
        // 一旦异常穿过 dynamic_reconfigure 的服务回调，ROS 会直接 terminate 整个 nodelet 进程。
        try
        {
            reconfigApply(config, level);
        }
        catch (const std::exception& e)
        {
            ROS_ERROR("reconfigCB(): exception ignored: %s", e.what());
        }
        catch (...)
        {
            ROS_ERROR("reconfigCB(): unknown exception ignored.");
        }
    }

    void HKCameraNodelet::reconfigApply(CameraConfig& config, uint32_t level)
    {
        // 配置线程（dynamic_reconfigure 服务）与重连线程并发：全程持 dev_mutex_，
        // 与 initializeCamera()/releaseDevice() 串行化"句柄生命周期 + SDK 参数写"。
        std::lock_guard<std::mutex> dev_lock(dev_mutex_);
        (void)level;
        // Launch setting
        if (initialize_flag_)
        {
            config.exposure_auto = exposure_auto_;
            config.exposure_value = exposure_value_;
            config.exposure_max = exposure_max_;
            config.exposure_min = exposure_min_;
            config.gain_auto = gain_auto_;
            config.gain_value = gain_value_;
            // 自动增益上下限也来自参数（config/*.yaml > <node>/<param>），yaml 才能真正控制自动增益上限；
            // 缺省 0/16dB 与 cfg/camera.cfg 一致，所以不写这两项时行为与上游完全相同。
            config.gain_min = gain_min_;
            config.gain_max = gain_max_;
            config.gamma_selector = gamma_selector_;
            config.gamma_value = gamma_value_;
            config.white_auto = white_auto_;
            config.white_selector = white_selector_;
            config.stop_grab = stop_grab_;
            config.is_fps_down = is_fps_down_.load();
            config.target_fps = target_fps_.load();
            initialize_flag_ = false;
        }
        if (dev_handle_ == nullptr) return;

        // Switch camera
        if (!config.stop_grab && need_grab_)
        {
            CHECK_MVS(MV_CC_StartGrabbing(dev_handle_), false);
            need_grab_ = false;
        }
        if (config.stop_grab && !need_grab_)
        {
            CHECK_MVS(MV_CC_StopGrabbing(dev_handle_), false);
            need_grab_ = true;
        }

        // Exposure
        if (config.exposure_auto)
        {
            _MVCC_FLOATVALUE_T exposure_time{0};
            //    CHECK_MVS(MV_CC_SetIntValue(dev_handle_, "AutoExposureTimeLowerLimit", config.exposure_min));
            CHECK_MVS(MV_CC_SetIntValue(dev_handle_, "AutoExposureTimeLowerLimit", config.exposure_min), false);
            CHECK_MVS(MV_CC_SetIntValue(dev_handle_, "AutoExposureTimeUpperLimit", config.exposure_max), false);
            CHECK_MVS(MV_CC_SetEnumValue(dev_handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_CONTINUOUS), false);
            CHECK_MVS(MV_CC_GetFloatValue(dev_handle_, "ExposureTime", &exposure_time), false);
            config.exposure_value = exposure_time.fCurValue;
        }
        else
        {
            CHECK_MVS(MV_CC_SetEnumValue(dev_handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF), false);
            CHECK_MVS(MV_CC_SetFloatValue(dev_handle_, "ExposureTime", config.exposure_value), false);
        }

        // Gain
        if (config.gain_auto)
        {
            _MVCC_FLOATVALUE_T gain_value{0};
            CHECK_MVS(MV_CC_SetFloatValue(dev_handle_, "AutoGainLowerLimit", config.gain_min), false);
            CHECK_MVS(MV_CC_SetFloatValue(dev_handle_, "AutoGainUpperLimit", config.gain_max), false);
            CHECK_MVS(MV_CC_SetEnumValue(dev_handle_, "GainAuto", MV_GAIN_MODE_CONTINUOUS), false);
            CHECK_MVS(MV_CC_GetFloatValue(dev_handle_, "Gain", &gain_value), false);
            config.gain_value = gain_value.fCurValue;
        }
        else
        {
            _MVCC_FLOATVALUE_T gain_value{0};
            CHECK_MVS(MV_CC_SetEnumValue(dev_handle_, "GainAuto", MV_GAIN_MODE_OFF), false);
            CHECK_MVS(MV_CC_SetFloatValue(dev_handle_, "Gain", config.gain_value), false);
            CHECK_MVS(MV_CC_GetFloatValue(dev_handle_, "Gain", &gain_value), false);
            config.gain_value = gain_value.fCurValue;
        }

        // Black level
        // Can not be used!
        CHECK_MVS(MV_CC_SetEnumValue(dev_handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_OFF), false);
        switch (config.white_selector)
        {
        case 0:
            CHECK_MVS(MV_CC_SetEnumValue(dev_handle_, "BalanceRatioSelector", 0), false);
            break;
        case 1:
            CHECK_MVS(MV_CC_SetEnumValue(dev_handle_, "BalanceRatioSelector", 1), false);
            break;
        case 2:
            CHECK_MVS(MV_CC_SetEnumValue(dev_handle_, "BalanceRatioSelector", 2), false);
            break;
        default:
            ROS_ERROR("Invalid white_selector value: %d", config.white_selector);
            break;
        }

        // BalanceWhiteAuto 一律 OFF（"Black level Can not be used" 段），white_selector 选通道，
        // 然后把当前 BalanceRatio 回读进 config.white_value 供 rqt 显示。
        // 实测(MV-CS016-10UC)：BalanceRatio 是**整型**节点(实测当前值 1549)，用 GetFloatValue 读返回
        // 0x80000109；因此先按整型读、失败再退回浮点（兼容别的机型），两条路都只看返回码、不刷告警。
        if (config.white_auto)
            CHECK_MVS(MV_CC_SetEnumValue(dev_handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS), false);
        else
            CHECK_MVS(MV_CC_SetEnumValue(dev_handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_OFF), false);

        _MVCC_INTVALUE_T white_value_int{0};
        if (MV_CC_GetIntValue(dev_handle_, "BalanceRatio", &white_value_int) == MV_OK)
        {
            config.white_value = white_value_int.nCurValue;
        }
        else
        {
            _MVCC_FLOATVALUE_T white_value_float{0};
            if (MV_CC_GetFloatValue(dev_handle_, "BalanceRatio", &white_value_float) == MV_OK)
                config.white_value = white_value_float.fCurValue;
        }

        // Gamma：实测本相机(MV-CS016-10UC)在 Bayer 像素格式(RG8/BayerRG8)下 Gamma 的三个节点
        // (GammaEnable/GammaSelector/Gamma)均不可访问，读写都返回 MV_E_GC_ACCESS(0x80000106)。
        // 原因在相机 XML：pIsAvailable = Gamma_LUT_Contrast_NotAva，
        //    Gamma_LUT_Contrast_NotAva = (Is_Bayer_AvaNot = 0) ? ((Is_Bayer_Support = 1) ? 1 : 0) : 1
        // 当前 PixelFormat=0x01080009 时 Is_Bayer_AvaNot=0 而 Is_Bayer_Support=0，故可用性为 0；
        // 换成 RGB8/Mono8 等非 Bayer 格式后三个节点立即变为可读写。
        // 因此这里一律用 warn（CHECK_MVS 第二参数 false）：失败只告警、不抛异常。
        // 若抛出异常，会从 dynamic_reconfigure 回调逃出，导致本函数后续设置全部被丢弃，
        // 甚至因无人捕获而 terminate(节点崩溃)。
        switch (config.gamma_selector)
        {
        case 0:
            CHECK_MVS(MV_CC_SetBoolValue(dev_handle_, "GammaEnable", true), false);
            CHECK_MVS(MV_CC_SetEnumValue(dev_handle_, "GammaSelector", MV_GAMMA_SELECTOR_SRGB), false);
            break;
        case 1:
            CHECK_MVS(MV_CC_SetBoolValue(dev_handle_, "GammaEnable", true), false);
            CHECK_MVS(MV_CC_SetEnumValue(dev_handle_, "GammaSelector", MV_GAMMA_SELECTOR_USER), false);
            // 原来是 true：写失败会抛异常并中断后面所有设置，改为只告警
            CHECK_MVS(MV_CC_SetGamma(dev_handle_, config.gamma_value), false);
            break;
        case 2:
            CHECK_MVS(MV_CC_SetBoolValue(dev_handle_, "GammaEnable", false), false);
            break;
        default:
            ROS_ERROR("Invalid gamma_selector value: %d", config.gamma_selector);
            break;
        }


        // take_photo_ = config.take_photo;
        is_fps_down_.store(config.is_fps_down);
        target_fps_.store(std::max(config.target_fps, 1.0));
        std::lock_guard<std::mutex> lock(fps_down_mutex_);
        next_pub_time_ = ros::WallTime();
        //  Width offset of image
        //  width_ = config.width_offset;
    }

    HKCameraNodelet::~HKCameraNodelet()
    {
        if (timer_.isValid()) timer_.stop();
        camera_change_sub.shutdown();
        camera_stop_sub_.shutdown();
        status_change_srv_.shutdown();
        releaseDevice(true);
        delete[] img_;
        img_ = nullptr;
        delete srv_;
        srv_ = nullptr;
    }

    void HKCameraNodelet::closeDevice(const bool tf)
    {
        // 前置条件：调用方已持 dev_mutex_（releaseDevice() 持锁后调进来，见其注释）
        if (dev_handle_ == nullptr) return;

        // 注意：这里不能再把 MV_CC_IsDeviceConnected(dev_handle_) 当清理的前置条件。
        // 掉线重连时设备恰好"查不到"，旧判断会让整个清理变成空操作，旧句柄既不停止取流也不关闭，
        // 紧接着 MV_CC_CreateHandle 会覆盖并泄漏旧句柄，随后 OpenDevice 返回 0x80000203(设备无访问权限)，
        // 重连就永远失败。所以清理动作无条件执行，错误码只告警。
        (void)tf; // tf 仅为兼容旧调用保留：清理阶段一律 warn-only，保证清理一定执行到底
        closing_ = true; // 通知图像回调线程立即放弃后续帧（无锁标志，避免与 DestroyHandle 抢句柄）
        // 等一下"已经进了回调、还没退出"的那一帧：它可能正拿着这个句柄做 MV_CC_ConvertPixelType，
        // 必须等它退出后才可 StopGrabbing/CloseDevice/DestroyHandle（否则是 use-after-free）。
        drainFrameCallbacks();
        try
        {
            if (!need_grab_) CHECK_MVS(MV_CC_StopGrabbing(dev_handle_), false);
            need_grab_ = true;
            CHECK_MVS(MV_CC_RegisterImageCallBackEx(dev_handle_, nullptr, nullptr), false);
            CHECK_MVS(MV_CC_CloseDevice(dev_handle_), false);
            // 仅仅是关闭句柄，销毁句柄由ReleaseDevice()确定
            // CHECK_MVS(MV_CC_DestroyHandle(dev_handle_), true);
        }
        catch (std::runtime_error& e)
        {
            ROS_ERROR("closeDevice(): %s", e.what());
        }
    }

    void HKCameraNodelet::releaseDevice(const bool tf)
    {
        // 本函数自己持 dev_mutex_：调用方（initializeCamera/timerCallback/析构）必须在**锁外**调用，
        // 否则自锁死。closeDevice() 依赖"本函数已持锁"这一前置条件。
        std::lock_guard<std::mutex> dev_lock(dev_mutex_);
        if (dev_handle_ == nullptr)
        {
            closing_ = false;
            return;
        }
        try
        {
            // 设备已掉线时 MV_CC_IsDeviceConnected() 会失败，但清理必须照做（见 closeDevice 注释）
            closeDevice(tf);
            CHECK_MVS(MV_CC_DestroyHandle(dev_handle_), false);
        }
        catch (std::runtime_error& e)
        {
            ROS_ERROR("releaseDevice(): error %s", e.what());
        }
        // 无论成功与否都必须置空：否则下一次 MV_CC_CreateHandle 会带着旧句柄去创建，
        // 旧设备资源不释放、新句柄也打不开设备(0x80000203)
        setDevHandleLocked(nullptr);
        need_grab_ = false;
        closing_ = false;
        got_frame_ = false; // 新句柄重新开始计帧看门狗
    }

    // 句柄写入口：dev_handle_（锁内读写）与 dev_handle_snapshot_（图像回调热路径无锁读）必须同步
    // 更新，集中到这里改写，避免以后漏掉其中一处。**必须持 dev_mutex_ 调用**。
    void HKCameraNodelet::setDevHandleLocked(void* handle)
    {
        dev_handle_ = handle;
        dev_handle_snapshot_.store(handle, std::memory_order_release);
    }

    // 等待在途图像回调退出。**必须持 dev_mutex_ 调用**，且只在 closeDevice() 里用：
    // 正常 165fps 下在途回调数是 0~1，循环立刻返回；超时（例如 SDK 回调卡死）只告警不阻断，
    // 因为紧接着的 MV_CC_RegisterImageCallBackEx(nullptr) 已保证 SDK 不再投递新回调。
    void HKCameraNodelet::drainFrameCallbacks()
    {
        const int drain_timeout_ms = 500;
        const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(drain_timeout_ms / 1000.0);
        int inflight = frame_callbacks_inflight_.load(std::memory_order_acquire);
        while (inflight > 0 && ros::WallTime::now() < deadline)
        {
            ros::WallDuration(0.001).sleep();
            inflight = frame_callbacks_inflight_.load(std::memory_order_acquire);
        }
        if (inflight > 0)
            ROS_WARN_THROTTLE(1.0, "drainFrameCallbacks(): %d image callback(s) still in flight "
                                    "after %d ms, continue cleanup anyway.", inflight, drain_timeout_ms);
    }

    bool HKCameraNodelet::validateDimensions(void* handle, int width, int height, int offset_x, int offset_y)
    {
        if (width <= 0 || height <= 0 || offset_x < 0 || offset_y < 0)
        {
            ROS_ERROR("validateDimensions(): invalid geometry w=%d h=%d off=(%d,%d)", width, height, offset_x,
                      offset_y);
            return false;
        }
        MVCC_INTVALUE width_limit{0};
        MVCC_INTVALUE height_limit{0};
        if (MV_CC_GetIntValue(handle, "WidthMax", &width_limit) != MV_OK ||
            MV_CC_GetIntValue(handle, "HeightMax", &height_limit) != MV_OK)
        {
            ROS_WARN("validateDimensions(): cannot query WidthMax/HeightMax, skip strict bounds check.");
            return true; // 不阻断，后续 Set 调用仍会由 SDK 校验。
        }
        if (static_cast<uint64_t>(width) + static_cast<uint64_t>(offset_x) > width_limit.nCurValue ||
            static_cast<uint64_t>(height) + static_cast<uint64_t>(offset_y) > height_limit.nCurValue)
        {
            ROS_ERROR("validateDimensions(): geometry out of range: w+offx=%llu(max %u), h+offy=%llu(max %u)",
                      static_cast<unsigned long long>(width + offset_x), width_limit.nCurValue,
                      static_cast<unsigned long long>(height + offset_y), height_limit.nCurValue);
            return false;
        }
        return true;
    }
} // namespace hk_camera
