//
// Created by zihan on 2022/6/2.
//
#include <pluginlib/class_list_macros.h>
#include <hk_camera.h>
#include <utility>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

namespace hk_camera
{
    PLUGINLIB_EXPORT_CLASS(hk_camera::HKCameraNodelet, nodelet::Nodelet)

    HKCameraNodelet::HKCameraNodelet() = default;

    void HKCameraNodelet::onInit()
    {
        nh_ = this->getPrivateNodeHandle();
        node_name_ = nh_.getNamespace();
        image_transport::ImageTransport it(nh_);
        pub_ = it.advertiseCamera("image_raw", 1);
        d_pub_ = it.advertise("image_raw_down", 1);
        this->status_change_srv_ = nh_.advertiseService("exposure_status_switch", &HKCameraNodelet::changeStatusCB,
                                                        this);

        dev_num_ = 0;
        // get param
        {
            nh_.param("camera_frame_id", image_.header.frame_id, std::string("camera_optical_frame"));
            nh_.param("camera_name", camera_name_, std::string("camera"));
            nh_.param("camera_info_url", camera_info_url_, std::string(""));
            nh_.param("image_width", image_width_, 1440);
            nh_.param("image_height", image_height_, 1080);
            nh_.param("image_offset_x", image_offset_x_, 0);
            nh_.param("image_offset_y", image_offset_y_, 0);
            nh_.param("pixel_format", pixel_format_, std::string("bgr8"));
            nh_.param("frame_id", frame_id_, std::string("camera_optical_frame"));
            nh_.param("camera_sn", camera_sn_, std::string(""));
            nh_.param("frame_rate", frame_rate_, 200.0);
            nh_.param("sleep_time", sleep_time_, 0);
            nh_.param("gain_value", gain_value_, 15.0);
            nh_.param("gain_auto", gain_auto_, false);
            nh_.param("gamma_selector", gamma_selector_, 2);
            nh_.param("gamma_value", gamma_value_, 0.5);
            nh_.param("exposure_auto", exposure_auto_, true);
            nh_.param("exposure_value", exposure_value_, 20.0);
            nh_.param("exposure_max", exposure_max_, 3000.0);
            nh_.param("exposure_min", exposure_min_, 50.0);
            nh_.param("white_auto", white_auto_, true);
            nh_.param("white_selector", white_selector_, 0);
            nh_.param("enable_resolution", enable_resolution_, false);
            nh_.param("resolution_ratio_width", resolution_ratio_width_, 1440);
            nh_.param("resolution_ratio_height", resolution_ratio_height_, 1080);
            nh_.param("stop_grab", stop_grab_, false);
            nh_.param("is_fps_down", is_fps_down_, false);
            nh_.param("target_fps", target_fps_, 40.0);
        }

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
        info_.header.frame_id = frame_id_;
        image_.header.frame_id = frame_id_;
        image_.height = image_height_;
        image_.width = image_width_;
        image_.step = image_width_ * 3;
        image_.data.resize(image_.height * image_.step);
        image_.encoding = pixel_format_;
        image_buffer_size_ = image_.height * image_.step;
        img_ = new unsigned char[image_.height * image_.step];
        ROS_INFO("OnINit initializeCamera");

        ros::Time start = ros::Time::now();
        while (!initializeCamera())
        {
            ros::Duration(1).sleep();
            if ((ros::Time::now() - start).toSec() > 10 || !ros::ok())
            {
                ROS_ERROR("Failed to initialize camera. reinitial at timerCB().");
                first_init_ = false;
                break;
            }
        }

        ros::NodeHandle p_nh(nh_, "hk_camera_reconfig");
        pub_rect_ = p_nh.advertise<sensor_msgs::Image>("/image_rect", 1);
        srv_ = new dynamic_reconfigure::Server<CameraConfig>(p_nh);
        dynamic_reconfigure::Server<CameraConfig>::CallbackType cb = boost::bind(
            &HKCameraNodelet::reconfigCB, this, _1, _2);
        srv_->setCallback(cb);

        camera_change_sub = nh_.subscribe("/camera_name", 50, &hk_camera::HKCameraNodelet::cameraChange, this);
        camera_stop_sub_ = nh_.subscribe("/camera_stop", 50, &hk_camera::HKCameraNodelet::cameraStop, this);

        timer_ = nh_.createTimer(ros::Duration(1), &HKCameraNodelet::timerCallback, this);
        ROS_INFO("Camera %s is ready", camera_name_.c_str());
    }

    bool HKCameraNodelet::initializeCamera()
    {
        ROS_WARN("start initializeCamera");
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        try
        {
            int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
            if (nRet != MV_OK)
                throw(nRet);
        }
        catch (int nRet)
        {
            std::cout << "MV_CC_EnumDevices fail! nRet " << std::hex << nRet << std::endl;
            return false;
        }
        //  assert(MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList) == MV_OK);
        try
        {
            if (stDeviceList.nDeviceNum <= 0)
            {
                throw std::runtime_error("stDeviceList.nDeviceNum <= 0");
            }
        }
        catch (std::runtime_error& e)
        {
            ROS_ERROR("%s", e.what());
            return false;
        }

        // Opens the device.
        unsigned int nIndex = 0;
        MVCC_STRINGVALUE dev_sn;
        memset(&dev_sn, 0, sizeof(MVCC_STRINGVALUE));
        ros::Duration(sleep_time_).sleep();
        if (stDeviceList.nDeviceNum > 1)
        {
            if (camera_sn_.empty())
            {
                ROS_ERROR("Multiple cameras found, but camera_sn is empty.");
                // throw std::runtime_error("camera_sn is required when multiple cameras are connected");
                return false;
            }
            for (int not_matched_num = 0; nIndex < stDeviceList.nDeviceNum; nIndex++)
            {
                ROS_WARN("creating handle");
                try{
                    CHECK_MVS(MV_CC_CreateHandle(&dev_handle_, stDeviceList.pDeviceInfo[nIndex]));
                    CHECK_MVS(MV_CC_OpenDevice(dev_handle_));
                    CHECK_MVS(MV_CC_GetStringValue(dev_handle_, "DeviceSerialNumber", &dev_sn));
                }catch (std::runtime_error& e)
                {
                    ROS_ERROR("initialize handle failed:%s", e.what());
                    releaseDevice();
                    return false;
                }
                if (strcmp(dev_sn.chCurValue, (char*)camera_sn_.data()) == 0)
                {
                    ROS_WARN("find target!");
                    dev_num_++;
                    break;
                }
                else
                {
                    releaseDevice();
                    ROS_WARN("wrong target");
                    not_matched_num++;
                    //If all device not match, drop.
                    if (not_matched_num == stDeviceList.nDeviceNum)
                    {
                        ROS_ERROR("Serial number not match!");
                        // throw std::runtime_error("Serial number not match!");
                        releaseDevice();
                        return false;
                    }
                }
            }
        }
        else
        {
            try{
                CHECK_MVS(MV_CC_CreateHandle(&dev_handle_, stDeviceList.pDeviceInfo[nIndex]));
                CHECK_MVS(MV_CC_OpenDevice(dev_handle_));
                CHECK_MVS(MV_CC_GetStringValue(dev_handle_, "DeviceSerialNumber", &dev_sn));
            }catch (std::runtime_error& e)
            {
                ROS_ERROR("initialize failed: %s", e.what());
                releaseDevice();
                return false;
            }
            if (dev_sn.chCurValue[0] == '\0' && stDeviceList.nDeviceNum > 1)
            {
                ROS_ERROR("Device serial number is null.");
                // throw std::runtime_error("Device serial number is null");
                releaseDevice();
                return false;
            }
            dev_num_ = 1;
            //If first init and only one device, use device_sn instead of camera_sn in the config.yaml
            if (!is_sn_init)
                camera_sn_ = std::string(dev_sn.chCurValue);
        }
        //Camera_sn first init complete, won't change camera_sn_ anymore
        is_sn_init = true;

        // if (!dev_sn.chCurValue)
        // {
        //   ROS_ERROR("No camera found, check physical connection.");
        //   // throw std::runtime_error("No camera found");
        //   return false;

        // }

        // Print the camera serial number
        ROS_INFO("Camera Serial Number: %s", dev_sn.chCurValue);


        // Retrieve and print the camera's model name using DeviceModelName
        MVCC_STRINGVALUE model_name;
        memset(&model_name, 0, sizeof(MVCC_STRINGVALUE));
        int nRet = MV_CC_GetStringValue(dev_handle_, "DeviceModelName", &model_name);
        if (nRet == MV_OK)
        {
            // 相机型号
            ROS_INFO("Camera Model: %s", model_name.chCurValue);
        }
        else
        {
            ROS_WARN("Failed to get camera model name. Error code: %x", nRet);
            releaseDevice();
            return false;
        }

        MvGvspPixelType format = PixelType_Gvsp_Undefined;
        if (pixel_format_ == "mono8")
            format = PixelType_Gvsp_Mono8;
        else if (pixel_format_ == "mono16")
            format = PixelType_Gvsp_Mono16;
            // SDK api过老，后续更新
            // 下列对应关系目前就是这样的，不要使用Packed后缀
        else if (pixel_format_ == "bgra8")
            format = PixelType_Gvsp_BayerBG8;
        else if (pixel_format_ == "rgb8")
            format = PixelType_Gvsp_BayerRG8;
        else if (pixel_format_ == "bgr8")
            format = PixelType_Gvsp_BayerGB8;
        else if (format == PixelType_Gvsp_Undefined)
        {
            // static_assert(true, "Illegal format");
            ROS_ERROR("illegal pixel format!");
            releaseDevice();
            return false;
        }
        ROS_INFO("Pixel Format: %s", pixel_format_.c_str());


        try
        {
            // CHECK_MVS(MV_CC_SetEnumValue(dev_handle_,"PixelFormat",format)); // 每个设备都不同，不适用
            CHECK_MVS(MV_CC_SetIntValueEx(dev_handle_, "Width", image_width_));
            CHECK_MVS(MV_CC_SetIntValue(dev_handle_, "Height", image_height_));
            CHECK_MVS(MV_CC_SetIntValue(dev_handle_, "OffsetX", image_offset_x_));
            CHECK_MVS(MV_CC_SetIntValue(dev_handle_, "OffsetY", image_offset_y_));
            //  AcquisitionLineRate ,LineRate can't be set
            //  CHECK_MVS(MV_CC_SetBoolValue(dev_handle_,"AcquisitionLineRateEnable", true));
            //  CHECK_MVS(MV_CC_SetIntValue(dev_handle_,"AcquisitionLineRate", 10));
        }
        catch (std::runtime_error& e)
        {
            ROS_ERROR("initialize failed:%s", e.what());
            releaseDevice();
            return false;
        }

        _MVCC_FLOATVALUE_T frame_rate{0};
        if (!MV_CC_SetFrameRate(dev_handle_, frame_rate_))
        {
            ROS_ERROR("Failed to set targrt frame rate:%f",frame_rate_);
        };
        if (!MV_CC_GetFrameRate(dev_handle_, &frame_rate))
        {
            ROS_ERROR("Failed to get frame rate!");
        }
        ROS_INFO("Frame rate is: %f", frame_rate.fCurValue);

        CHECK_MVS(MV_CC_SetEnumValue(dev_handle_, "TriggerMode", 0));
        CHECK_MVS(MV_CC_RegisterImageCallBackEx(dev_handle_, onFrameCB, this));
        int nRet_temp=MV_CC_StartGrabbing(dev_handle_);
        if (nRet_temp == MV_OK)
        {
            ROS_INFO("Stream On.");
        }
        else
        {
            ROS_ERROR("Stream On failed! nRet_temp: %d", nRet_temp);
            releaseDevice();
            return false;
        }
        return true;
    }

    void HKCameraNodelet::timerCallback(const ros::TimerEvent&)
    {
        // if () {
        //   ROS_WARN("set camera_restart_flag_ true!");
        //   camera_restart_flag_ = true;
        // }
        if (dev_handle_ && !MV_CC_IsDeviceConnected(dev_handle_))
        {
            MV_CC_DEVICE_INFO_LIST stDeviceList;
            memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
            try
            {
                int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
                if (nRet != MV_OK)
                    throw(nRet);
            }
            catch (int nRet)
            {
                std::cout << "MV_CC_EnumDevices fail! nRet " << std::hex << nRet << std::endl;
                // exit(-1);
                return;
            }

            std::cout << "searching target:" << camera_sn_ << std::endl;
            for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                ROS_INFO("device:%d\n", stDeviceList.nDeviceNum);
                if (stDeviceList.pDeviceInfo[i] == nullptr)
                {
                    ROS_INFO("no device.");
                    break;
                }
                const char* sn = reinterpret_cast<const char*>(
                    stDeviceList.pDeviceInfo[i]->SpecialInfo.stUsb3VInfo.chSerialNumber);

                // if device_sn and camera_sn_ not the same, reject
                if (strcmp(sn, camera_sn_.c_str()) == 0)
                {
                    releaseDevice();
                    dev_handle_ = nullptr;
                    initializeCamera();
                    // camera_restart_flag_ = false;
                    break;
                }
            }
        }
        else if (dev_handle_ == nullptr && first_init_ == false)
        {
            initializeCamera();
        }
    }

    bool HKCameraNodelet::changeStatusCB(rm_msgs::StatusChange::Request& change, rm_msgs::StatusChange::Response& res)
    {
        if (change.target)
            nh_.param("exposure_value_windmill", exposure_value_, 3000.0);
        else
            nh_.param("exposure_value", exposure_value_, 20.0);
        try{
            CHECK_MVS(MV_CC_SetEnumValue(dev_handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF));
            CHECK_MVS(MV_CC_SetFloatValue(dev_handle_, "ExposureTime", exposure_value_));
        }catch (std::runtime_error& e)
        {
            ROS_ERROR("changeStatusCB() error :%s", e.what());
        }
        res.switch_is_success = true;
        return true;
    }

    void HKCameraNodelet::cameraChange(const std_msgs::String& camera_change)
    {
        try{
            if (strcmp(camera_change.data.c_str(), node_name_.substr(1).c_str()) == 0)
                CHECK_MVS(MV_CC_StartGrabbing(dev_handle_));
            else
                CHECK_MVS(MV_CC_StopGrabbing(dev_handle_));
        }catch (std::runtime_error& e)
        {
            ROS_ERROR("cameraChange() error: %s", e.what());
        }
    }

    void HKCameraNodelet::cameraStop(const std_msgs::Bool camera_stop_msg_)
    {
        try{
            if (camera_stop_msg_.data == true && strcmp("camera_back", node_name_.substr(1).c_str()) == 0)
                CHECK_MVS(MV_CC_StopGrabbing(dev_handle_));
            else
                CHECK_MVS(MV_CC_StartGrabbing(dev_handle_));
        }catch (std::runtime_error& e)
        {
            ROS_ERROR("cameraStop() error: %s", e.what());
        }
    }


    void HKCameraNodelet::onFrameCB(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser)
    {
        auto* self = static_cast<HKCameraNodelet*>(pUser);
        if (pFrameInfo)
        {
            ros::Time now = ros::Time::now();


            ros::Time now_stamp = ros::Time::now();
            image_.header.stamp = now_stamp;
            info_.header.stamp = now_stamp;


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
            stConvertParam.nDstBufferSize = pFrameInfo->nWidth * pFrameInfo->nHeight * 3;
            int n_Ret_temp = MV_CC_ConvertPixelType(dev_handle_, &stConvertParam);
            if ( n_Ret_temp != MV_OK)
            {
                ROS_ERROR("MV_CC_ConvertPixelType() : return != MV_OK, code: %d", n_Ret_temp);
                return;
            }
            memcpy((char*)(&image_.data[0]), img_, image_.step * image_.height);

            //      if(take_photo_)
            //      {
            //          std::string str;
            //          str = std::to_string(count_);
            //          ROS_INFO("ok");
            //          cv_bridge::CvImagePtr cv_ptr1;
            //          cv_ptr1 = cv_bridge::toCvCopy(image_, "bgr8");
            //          cv::Mat cv_img1;
            //          cv_ptr1->image.copyTo(cv_img1);
            //          cv::imwrite("/home/irving/carphoto/"+str+".jpg",cv_img1);
            //          count_++;
            //      }

            if (enable_resolution_)
            {
                cv_bridge::CvImagePtr cv_ptr;
                cv_ptr = cv_bridge::toCvCopy(image_, "bgr8");
                cv::Mat cv_img;
                cv_ptr->image.copyTo(cv_img);
                sensor_msgs::ImagePtr image_rect_ptr;

                try
                {
                    cv::resize(cv_img, cv_img, cvSize(resolution_ratio_width_, resolution_ratio_height_));
                }
                catch (cv::Exception& e)
                {
                    ROS_ERROR("onFrameCB() cv::resize failed: %s", e.what());
                }
                image_rect_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_img).toImageMsg();
                pub_rect_.publish(image_rect_ptr);

                //    if (strcmp(camera_name_.data(), "hk_right"))
                //    {
                //      cv::Rect rect(0, 0, 1440 - width_, 1080);
                //      cv_img = cv_img(rect);
                //      image_rect_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_img).toImageMsg();
                //      pub_rect_.publish(image_rect_ptr);
                //    }
                //    if (strcmp(camera_name_.data(), "hk_left"))
                //    {
                //      cv::Rect rect(width_, 0, 1440 - width_, 1080);
                //      cv_img = cv_img(rect);
                //      image_rect_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_img).toImageMsg();
                //      pub_rect_.publish(image_rect_ptr);
                //    }
            }
            pub_.publish(image_, info_);

            bool publish_downsampled = false;
            if (self->is_fps_down_)
            {
                const ros::WallTime current_time = ros::WallTime::now();
                const ros::WallDuration interval(1.0 / self->target_fps_);
                {
                    std::lock_guard<std::mutex> lock(self->fps_down_mutex_);
                    if (self->next_pub_time_.isZero())
                        self->next_pub_time_ = current_time;
                    if (current_time >= self->next_pub_time_)
                    {
                        do
                            self->next_pub_time_ += interval;
                        while (self->next_pub_time_ <= current_time);
                        publish_downsampled = true;
                    }
                }
            }
            if (publish_downsampled)
                self->d_pub_.publish(image_);
        }
        else
            ROS_ERROR("Grab image failed!");
    }

    void HKCameraNodelet::reconfigCB(CameraConfig& config, uint32_t level)
    {
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
            config.gamma_selector = gamma_selector_;
            config.gamma_value = gamma_value_;
            config.white_auto = white_auto_;
            config.white_selector = white_selector_;
            config.stop_grab = stop_grab_;
            config.is_fps_down = is_fps_down_;
            config.target_fps = target_fps_;
            initialize_flag_ = false;
        }

        try{
            // Switch camera
            if (!config.stop_grab)
                CHECK_MVS(MV_CC_StartGrabbing(dev_handle_));
            else
                CHECK_MVS(MV_CC_StopGrabbing(dev_handle_));

            // Exposure
            if (config.exposure_auto)
            {
                _MVCC_FLOATVALUE_T exposure_time{0};
                //    CHECK_MVS(MV_CC_SetIntValue(dev_handle_, "AutoExposureTimeLowerLimit", config.exposure_min));
                CHECK_MVS(MV_CC_SetIntValue(dev_handle_, "AutoExposureTimeLowerLimit", config.exposure_min));
                CHECK_MVS(MV_CC_SetIntValue(dev_handle_, "AutoExposureTimeUpperLimit", config.exposure_max));
                CHECK_MVS(MV_CC_SetEnumValue(dev_handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_CONTINUOUS));
                CHECK_MVS(MV_CC_GetFloatValue(dev_handle_, "ExposureTime", &exposure_time));
                config.exposure_value = exposure_time.fCurValue;
            }
            else
            {
                CHECK_MVS(MV_CC_SetEnumValue(dev_handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF));
                CHECK_MVS(MV_CC_SetFloatValue(dev_handle_, "ExposureTime", config.exposure_value));
            }

            // Gain
            if (config.gain_auto)
            {
                _MVCC_FLOATVALUE_T gain_value{0};
                CHECK_MVS(MV_CC_SetFloatValue(dev_handle_, "AutoGainLowerLimit", config.gain_min));
                CHECK_MVS(MV_CC_SetFloatValue(dev_handle_, "AutoGainUpperLimit", config.gain_max));
                CHECK_MVS(MV_CC_SetEnumValue(dev_handle_, "GainAuto", MV_GAIN_MODE_CONTINUOUS));
                CHECK_MVS(MV_CC_GetFloatValue(dev_handle_, "Gain", &gain_value));
                config.gain_value = gain_value.fCurValue;
            }
            else
            {
                _MVCC_FLOATVALUE_T gain_value{0};
                CHECK_MVS(MV_CC_SetEnumValue(dev_handle_, "GainAuto", MV_GAIN_MODE_OFF));
                CHECK_MVS(MV_CC_SetFloatValue(dev_handle_, "Gain", config.gain_value));
                CHECK_MVS(MV_CC_GetFloatValue(dev_handle_, "Gain", &gain_value));
                config.gain_value = gain_value.fCurValue;
            }

            // Black level
            // Can not be used!
            CHECK_MVS(MV_CC_SetEnumValue(dev_handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_OFF));
            switch (config.white_selector)
            {
            case 0:
                CHECK_MVS(MV_CC_SetEnumValue(dev_handle_, "BalanceRatioSelector", 0));
                break;
            case 1:
                CHECK_MVS(MV_CC_SetEnumValue(dev_handle_, "BalanceRatioSelector", 1));
                break;
            case 2:
                CHECK_MVS(MV_CC_SetEnumValue(dev_handle_, "BalanceRatioSelector", 2));
                break;
            default:
                ROS_ERROR("Invalid white_selector value: %d", config.white_selector);
                break;
            }

            _MVCC_INTVALUE_T white_value{0};
            if (config.white_auto)
            {
                CHECK_MVS(MV_CC_SetEnumValue(dev_handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS));
                CHECK_MVS(MV_CC_GetIntValue(dev_handle_, "BalanceRatio", &white_value));
                config.white_value = white_value.nCurValue;
            }
            else
            {
                CHECK_MVS(MV_CC_SetEnumValue(dev_handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_OFF));
                CHECK_MVS(MV_CC_GetIntValue(dev_handle_, "BalanceRatio", &white_value));
                config.white_value = white_value.nCurValue;
            }

            switch (config.gamma_selector)
            {
            case 0:
                CHECK_MVS(MV_CC_SetBoolValue(dev_handle_, "GammaEnable", true));
                CHECK_MVS(MV_CC_SetEnumValue(dev_handle_, "GammaSelector", MV_GAMMA_SELECTOR_SRGB));
                break;
            case 1:
                CHECK_MVS(MV_CC_SetBoolValue(dev_handle_, "GammaEnable", true));
                CHECK_MVS(MV_CC_SetEnumValue(dev_handle_, "GammaSelector", MV_GAMMA_SELECTOR_USER));
                CHECK_MVS(MV_CC_SetGamma(dev_handle_, config.gamma_value));
                break;
            case 2:
                {
                    //      CHECK_MVS(MV_CC_SetBoolValue(dev_handle_, "GammaEnable", false));
                    CHECK_MVS(MV_CC_SetBoolValue(dev_handle_, "GammaEnable", false));
                    CHECK_MVS(MV_CC_SetBoolValue(dev_handle_, "GammaEnable", false));
                    break;
                }
            default:
                ROS_ERROR("Invalid gamma_selector value: %d", config.gamma_selector);
                break;
            }
        }catch (std::runtime_error& e)
        {
            ROS_ERROR("Error in reconfigCB: %s", e.what());
        }

        // take_photo_ = config.take_photo;
        is_fps_down_ = config.is_fps_down;
        target_fps_ = std::max(config.target_fps, 1.0);
        std::lock_guard<std::mutex> lock(fps_down_mutex_);
        next_pub_time_ = ros::WallTime();
        //  Width offset of image
        //  width_ = config.width_offset;
    }

    HKCameraNodelet::~HKCameraNodelet()
    {
        timer_.stop();
        releaseDevice();
        delete[] img_;
        img_ = nullptr;
        delete srv_;
        srv_ = nullptr;
    }

    void HKCameraNodelet::releaseDevice()
    {
        if (dev_handle_)
        {
            try{
                CHECK_MVS(MV_CC_StopGrabbing(dev_handle_));
                CHECK_MVS(MV_CC_RegisterImageCallBackEx(dev_handle_, nullptr, nullptr));
                CHECK_MVS(MV_CC_CloseDevice(dev_handle_));
                CHECK_MVS(MV_CC_DestroyHandle(dev_handle_));
                dev_handle_ = nullptr;
            }catch (std::runtime_error& e)
            {
                ROS_ERROR("Error in releaseDevice: %s", e.what());
            }
        }
    }

    void* HKCameraNodelet::dev_handle_ = nullptr;
    unsigned char* HKCameraNodelet::img_;
    sensor_msgs::Image HKCameraNodelet::image_;
    //sensor_msgs::Image HKCameraNodelet::image_rect;
    image_transport::CameraPublisher HKCameraNodelet::pub_;
    ros::Publisher HKCameraNodelet::pub_rect_;
    sensor_msgs::CameraInfo HKCameraNodelet::info_;
    //int HKCameraNodelet::width_{};
    //std::string HKCameraNodelet::imu_name_;
    //std::string HKCameraNodelet::camera_name_;
    //ros::ServiceClient HKCameraNodelet::imu_trigger_client_;
    //bool HKCameraNodelet::enable_imu_trigger_;
    //bool HKCameraNodelet::trigger_not_sync_ = false;
    //const int HKCameraNodelet::FIFO_SIZE = 1023;
    //int HKCameraNodelet::fifo_front_ = 0;
    // int HKCameraNodelet::fifo_rear_ = 0;
    // bool HKCameraNodelet::take_photo_{};
    // struct TriggerPacket HKCameraNodelet::fifo_[FIFO_SIZE];
    // uint32_t HKCameraNodelet::receive_trigger_counter_ = 0;
    bool HKCameraNodelet::enable_resolution_ = false;
    int HKCameraNodelet::resolution_ratio_width_ = 1440;
    int HKCameraNodelet::resolution_ratio_height_ = 1080;
    // bool HKCameraNodelet::camera_restart_flag_{};
} // namespace hk_camera
