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
#include "libMVSapi/MvCameraControl.h"
#include <std_msgs/String.h>
#include <rm_msgs/StatusChange.h>
#include <std_msgs/Bool.h>

// MVS SDK error checking macro
#define CHECK_MVS(func) \
    do { \
        int _ret = (func); \
        if (_ret != MV_OK) { \
        ROS_ERROR("%s FAILED at :%d! Error code: 0x%08x", \
            #func, __LINE__, _ret); \
        throw std::runtime_error(#func); \
        } \
    } while(0)

namespace hk_camera
{
    class HKCameraNodelet : public nodelet::Nodelet
    {
    public:
        HKCameraNodelet();
        ~HKCameraNodelet() override;

        void onInit() override;
        static sensor_msgs::Image image_;
        void timerCallback(const ros::TimerEvent&);

    private:
        void reconfigCB(CameraConfig& config, uint32_t level);
        void cameraChange(const std_msgs::String&);
        void cameraStop(const std_msgs::Bool);
        bool initializeCamera();
        void releaseDevice();
        bool changeStatusCB(rm_msgs::StatusChange::Request& change, rm_msgs::StatusChange::Response& res);

        ros::ServiceServer status_change_srv_;

        ros::NodeHandle nh_;
        static void* dev_handle_;
        int dev_num_;
        // bool camera_restart_flag_;
        dynamic_reconfigure::Server<CameraConfig>* srv_{};

        boost::shared_ptr<camera_info_manager::CameraInfoManager> info_manager_;
        ros::Timer timer_;
        std::string camera_name_;
        std::string camera_info_url_, pixel_format_, frame_id_, camera_sn_;
        double frame_rate_;
        int image_width_{}, image_height_{}, image_offset_x_{}, image_offset_y_{}, sleep_time_{};
        bool is_sn_init{};
        double gain_value_{};
        int gamma_selector_{};
        double gamma_value_{};
        bool initialize_flag_ = true;
        bool first_init_{true};
        bool gain_auto_{};
        bool exposure_auto_{};
        double exposure_value_{};
        double exposure_max_{};
        double exposure_min_{};
        bool white_auto_{};
        int white_selector_{};
        bool stop_grab_{};
        size_t image_buffer_size_;
        static unsigned char* img_;
        static image_transport::CameraPublisher pub_;
        static ros::Publisher pub_rect_;
        static sensor_msgs::CameraInfo info_;
        static bool enable_resolution_;
        static int resolution_ratio_width_;
        static int resolution_ratio_height_;
        //  bool take_photo_
        static void __stdcall onFrameCB(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser);
        // void processFrame();
        // void processOneFrame();
        ros::Subscriber camera_change_sub;
        ros::Subscriber camera_stop_sub_;

        image_transport::Publisher d_pub_;
        double target_fps_{40.0};
        ros::WallTime next_pub_time_;
        bool is_fps_down_{};
        std::mutex fps_down_mutex_;

        std::string node_name_;
    };
} // namespace hk_camera

#endif  // SRC_HK_CAMERA_INCLUDE_GALAXY_CAMERA_H_
