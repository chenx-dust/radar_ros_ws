#ifndef HKCAM_HPP
#define HKCAM_HPP
// HKSDK
#include <MvCameraControl.h>
// ROS
// #include <rm_utils/data.h>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
// #include <rm_interfaces/msg/rmrobot.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <mutex>

namespace hik_camera {

struct HikParams {
    // std::string sn;
    // std::string cam_name;
    // std::string robot_name;
    // std::string camera_info_url;
    // // normal
    // double exposure_time;
    // double gain;
    // double digital_shift;
    // int out_post_top_offset_y;
    // double frame_rate;
    // bool use_sensor_data_qos;

    // frameinfo
    // vision_mode mode;
    // std::atomic<Robot_id_dji> robot_id;
    std::atomic<double> bullet_velocity;
    std::atomic<bool> right_press;
    std::atomic<bool> lobshot;
};

class HikCameraNode : public rclcpp::Node {
public:
    explicit HikCameraNode(const rclcpp::NodeOptions& options);
    ~HikCameraNode() override;

private:
    std::string frame_id;
    int nRet = MV_OK;
    void* camera_handle;
    std::thread capture_thread;
    std::thread monitor_thread;
    HikParams params;
    // 相机图像发布
    image_transport::CameraPublisher image_pub;

    MV_IMAGE_BASIC_INFO img_info;
    MV_CC_PIXEL_CONVERT_PARAM convert_param;
    // 相机发布
    std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager;
    sensor_msgs::msg::CameraInfo camera_info_msg;

    rclcpp::AsyncParametersClient::SharedPtr param_client;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_event_sub;

    int fail_cnt = 0;
    bool rotate_180 = false;
    std::atomic<bool> grab_on = false;
    std::atomic<bool> monitor_on = false;
    std::atomic<bool> camera_failed = false;
    std::atomic<bool> param_changed = false;
    // 只在模式切换的时候更改，避免频繁读写
    void declare_params();
    void init_camera();
    void reset();
    void open_device();
    void close_device();
    void start_grab();
    void stop_grab();
    void set_hk_params();
    void set_grab_params(int offset_x, int offset_y, int roi_width, int roi_height);
    void grab();
    void monitor();
    std::pair<int, int> get_sensor_height_width();
};

class CameraException : public std::exception {
   public:
    std::string info;
    CameraException(const std::string&& _info) : info{_info} {}
    const char* what() const noexcept { return info.c_str(); }
};

}  // namespace hik_camera

#endif