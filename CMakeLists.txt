cmake_minimum_required(VERSION 3.5)
project(hk_camera)

set(CMAKE_CXX_STANDARD 11)

# 海康 MVS 相机 SDK 动态库绝对路径(/opt/MVS/lib/64)。
# 使用绝对路径而非 link_directories()：后者在 add_library() 之后调用不会
# 根据自己的实际情况更改，一般来说是这个
set(MVS_LIBRARY /opt/MVS/lib/64/libMvCameraControl.so)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        nodelet
        message_generation
        dynamic_reconfigure
        image_transport
        camera_info_manager
        rm_msgs
        cv_bridge
        )
find_package(
        OpenCV REQUIRED
)


generate_dynamic_reconfigure_options(
        cfg/camera.cfg
)

catkin_package(
        LIBRARIES hk_camera
        # rm_msgs(服务 rm_msgs/StatusChange)与 cv_bridge 出现在对外头文件/链接里，按 catkin 规范一并导出。
        # 本工作区目前没有其它包 find_package(hk_camera)，属于补正确性、不影响现有构建。
        CATKIN_DEPENDS nodelet roscpp dynamic_reconfigure
        image_transport camera_info_manager rm_msgs cv_bridge
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        include
        ../hk_camera
)


add_library(hk_camera
        src/hk_camera.cpp
        src/hk_camera_node.cpp
)

# 当你安装了pcl库时：
# 运行时通过 RPATH 定位海康 MVS 动态库(/opt/MVS/lib/64)，避免依赖全局
# LD_LIBRARY_PATH，防止 /opt/MVS/lib/64/libusb-1.0.so.0(旧版) 抢占系统 libusb
# 导致 libpcl_io 报 undefined symbol: libusb_set_option
set_target_properties(hk_camera PROPERTIES
        BUILD_RPATH "/opt/MVS/lib/64"
        INSTALL_RPATH "/opt/MVS/lib/64"
        BUILD_WITH_INSTALL_RPATH FALSE
)

target_link_libraries(hk_camera
        ${MVS_LIBRARY} ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_dependencies(hk_camera ${PROJECT_NAME}_gencfg)

#add_executable(hk_camera_node
#        src/hk_camera_node.cpp)
#add_dependencies(hk_camera_node ${PROJECT_NAME}_gencfg)
#target_link_libraries(hk_camera_node
#        ${MVS_LIBRARY} ${catkin_LIBRARIES} ${OpenCV_LIBS})
