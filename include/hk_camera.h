//
// Created by zihan on 2022/6/2.
//

#ifndef SRC_HK_CAMERA_INCLUDE_GALAXY_CAMERA_H_
#define SRC_HK_CAMERA_INCLUDE_GALAXY_CAMERA_H_
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <hk_camera/CameraConfig.h>
#include <camera_info_manager/camera_info_manager.h>
#include <ros/time.h>
#include <string>
#include <mutex>
#include <atomic>
#include "libMVSapi/MvCameraControl.h"
#include <std_msgs/String.h>
#include <rm_msgs/StatusChange.h>
#include <std_msgs/Bool.h>

// MVS SDK error checking macro
#define CHECK_MVS(func, throw_e)                                        \
    do {                                                                \
        int _ret = (func);                                              \
        if (_ret != MV_OK) {                                            \
            if(throw_e) {                                               \
                ROS_ERROR("%s: %s FAILED at :%d! Error code: 0x%08x",   \
                        __FILE__, #func, __LINE__, _ret);               \
                throw std::runtime_error(#func);                        \
            }else                                                       \
            {                                                           \
                ROS_WARN("%s: %s FAILED at :%d! Error code: 0x%08x",    \
                         __FILE__, #func, __LINE__, _ret);              \
            }                                                           \
        }                                                               \
    } while(0)

namespace hk_camera
{
    class HKCameraNodelet : public nodelet::Nodelet
    {
    public:
        HKCameraNodelet();
        ~HKCameraNodelet() override;

        void onInit() override;
        sensor_msgs::Image image_;
        void timerCallback(const ros::TimerEvent&);

    private:
        void reconfigCB(CameraConfig& config, uint32_t level);
        // reconfigCB 的实际实现：外壳只负责 try/catch 兜底（异常逃出 ROS 回调会 terminate 进程）
        void reconfigApply(CameraConfig& config, uint32_t level);
        void reportParamSources();
        void cameraChange(const std_msgs::String&);
        void cameraStop(const std_msgs::Bool);
        bool initializeCamera(bool from_timerCB = false);
        static MvGvspPixelType getPixelFormat(const std::string& pixelformat);
        // 句柄清理：closeDevice 必须**持 dev_mutex_** 调用（releaseDevice 内部已持）；
        // releaseDevice 自己持锁，**禁止在持锁时调用**（否则自锁死）
        void closeDevice(const bool tf);
        void releaseDevice(const bool tf);
        // 句柄写入口：必须持 dev_mutex_ 调用，负责同步 dev_handle_ 与热路径快照 dev_handle_snapshot_
        void setDevHandleLocked(void* handle);
        // 等待在途图像回调退出（必须持 dev_mutex_ 调用；只在关设备/销毁句柄前调用）
        void drainFrameCallbacks();
        static void __stdcall onExceptionCB(unsigned int nMsgType, void* pUser);
        bool validateDimensions(void* handle,int width, int height, int offset_x, int offset_y);
        bool changeStatusCB(rm_msgs::StatusChange::Request& change, rm_msgs::StatusChange::Response& res);

        // 参数读取统一入口：优先读 <node_ns>/hk_camera/<name>（与 config/*.yaml 的顶层键 hk_camera 对齐），
        // 只有该处未设置时才回退 <node_ns>/<name>（旧 launch 在 <node> 内直接写 <param> 的写法）。
        // 两处都有且取值不同时以 hk_camera/ 为准，启动时由 reportParamSources() 汇总告警。
        template <typename T>
        void getParamAligned(const std::string& name, T& member, const T& default_value)
        {
            if (!params_nh_.param(name, member, default_value))
                nh_.param(name, member, default_value);
        }

        ros::ServiceServer status_change_srv_;

        ros::NodeHandle nh_;
        // 参数命名空间 = <node_ns>/hk_camera。nodelet 的私有命名空间就是 nodelet 名称
        // （launch 里 <node name="hk_camera"> 或 nodelet load 的 __name），因此
        // config/*.yaml 里的顶层键 hk_camera 恰好落在 /hk_camera/hk_camera。
        // 多相机时每台相机在自己节点命名空间下各加载一份 yaml，键名保持不变。
        ros::NodeHandle params_nh_;
        // 相机句柄。**除图像回调热路径外，一律持 dev_mutex_ 访问**（热路径读 dev_handle_snapshot_）。
        void* dev_handle_{nullptr};
        // ---- 并发保护（D 组）----
        // dev_mutex_：只护"句柄生命周期 + 所有 SDK 参数写"。持锁者：
        //   initializeCamera()、releaseDevice()/closeDevice()（内部自持）、timerCallback() 的判定段、
        //   reconfigApply()、changeStatusCB()/cameraChange()/cameraStop()、析构函数。
        //   图像回调(onFrameCB → processFrame)是 165fps 热路径，**绝不持锁**，只用下面三个原子。
        std::mutex dev_mutex_;
        // 句柄快照：与 dev_handle_ 同步写入（setDevHandleLocked() 内，必须持锁），热路径只做 acquire load
        std::atomic<void*> dev_handle_snapshot_{nullptr};
        // 在途图像回调计数：closeDevice() 置 closing_ 后必须等它归零，才可关设备/销毁句柄，
        // 否则回调可能正拿着旧句柄做 MV_CC_ConvertPixelType（use-after-free）
        std::atomic<int> frame_callbacks_inflight_{0};
        // bool camera_restart_flag_;
        dynamic_reconfigure::Server<CameraConfig>* srv_{nullptr};

        boost::shared_ptr<camera_info_manager::CameraInfoManager> info_manager_{nullptr};
        ros::Timer timer_;
        std::string camera_name_{"hk_camera_default"};
        std::string camera_info_url_, pixel_format_, frame_id_, camera_sn_;
        double frame_rate_{200};
        int image_width_{0}, image_height_{0}, image_offset_x_{0}, image_offset_y_{0}, sleep_time_{0};
        double gain_value_{0};
        // 自动增益(GainAuto=Continuous)时的上下限，对应官方节点 AutoGainLowerLimit/AutoGainUpperLimit。
        // 取值默认与 cfg/camera.cfg 一致(0/16dB)，从参数读取后 config/*.yaml 才能真正控制自动增益上限。
        double gain_min_{0};
        double gain_max_{16};
        int gamma_selector_{0};
        double gamma_value_{0};
        bool initialize_flag_ = true;
        bool first_init_{true};
        bool gain_auto_{false};
        bool exposure_auto_{false};
        double exposure_value_{100};
        // 风车/特写等特殊模式下的曝光(us)：由 rm_msgs/StatusChange 服务 exposure_status_switch(target=true) 使用。
        // 单独一个成员而不是复用 exposure_value_，避免状态切换把"正常模式曝光"永久改掉（重连会按它重新设曝光）。
        double exposure_value_windmill_{2000};
        int exposure_max_{0};
        int exposure_min_{0};
        bool white_auto_{false};
        int white_selector_{0};
        bool stop_grab_{false};
        // 取流开关：cameraChange/cameraStop/reconfig/closeDevice 写，timerCallback 与 SDK 回调侧读
        std::atomic<bool> need_grab_{false};
        size_t image_buffer_size_{0};
        unsigned char* img_{nullptr};
        image_transport::CameraPublisher pub_;
        ros::Publisher pub_rect_;
        sensor_msgs::CameraInfo info_;
        bool enable_resolution_{true};
        int resolution_ratio_width_{0};
        int resolution_ratio_height_{0};
        //  bool take_photo_
        void processFrame(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo);
        static void __stdcall onFrameCB(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser);
        bool ensureBufferLocked(uint32_t width, uint32_t height);
        ros::Subscriber camera_change_sub;
        ros::Subscriber camera_stop_sub_;

        image_transport::Publisher d_pub_;
        // 抽帧参数：reconfigCB(配置线程)写、processFrame(图像回调线程)读 ⇒ 原子，热路径不取锁
        std::atomic<double> target_fps_{40.0};
        ros::WallTime next_pub_time_;
        std::atomic<bool> is_fps_down_{false};
        std::mutex fps_down_mutex_;

        // 掉线重连状态。全部为无锁标志：SDK 回调线程只置位，重连动作统一在 timerCallback 里执行。
        // need_reconnect_      : 异常回调(onExceptionCB)探测到设备断开时置位
        // reconnect_in_progress_: 正在重连，避免定时器叠加调用
        // closing_             : 正在关闭/销毁句柄，图像回调此时必须立即返回
        std::atomic<bool> need_reconnect_{false};
        std::atomic<bool> reconnect_in_progress_{false};
        std::atomic<bool> closing_{false};
        ros::WallTime next_reconnect_time_;
        int reconnect_backoff_ms_{1000};

        // 帧看门狗：部分异常（如 USB 总线复位后取流管道已断）设备仍上报"已连接"，
        // 此时只能靠"长时间收不到图像回调"来判定相机异常
        std::atomic<bool> got_frame_{false};
        std::atomic<double> last_frame_sec_{0.0};
        std::atomic<double> stream_on_sec_{0.0};

        std::string node_name_;
    };
} // namespace hk_camera

#endif  // SRC_HK_CAMERA_INCLUDE_GALAXY_CAMERA_H_
