#include <camera_info_manager/camera_info_manager.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("hik_camera");

    RCLCPP_INFO(node->get_logger(), "Starting HikCameraNode!");
    bool use_sensor_data_qos = node->declare_parameter("use_sensor_data_qos", false);
    std::string camera_info_url = node->declare_parameter("camera_info_url", "");
    // 创建pub
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;

    // frame_id 由名命名空间决定
    auto ns = std::string_view(node->get_namespace());
    auto ns_pos = ns.rfind('/');
    std::string frame_id;
    if (ns_pos != std::string_view::npos && ns_pos + 1 < ns.size()) {
        frame_id = ns.substr(ns.rfind('/') + 1);
        frame_id.append("_frame");
    } else {
        frame_id = "default_camera_frame";
    }

    // load camera info
    auto camera_info_manager = std::make_unique<camera_info_manager::CameraInfoManager>(node.get());
    sensor_msgs::msg::CameraInfo camera_info_msg;
    if (camera_info_manager->validateURL(camera_info_url)) {
        camera_info_manager->loadCameraInfo(camera_info_url);
        camera_info_msg = camera_info_manager->getCameraInfo();
        camera_info_msg.header.frame_id = frame_id;
    } else {
        RCLCPP_WARN(node->get_logger(), "Invalid camera info URL: %s",
            camera_info_url.c_str());
    }

    auto camera_info_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info",
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos)));

    auto timer = node->create_wall_timer(std::chrono::seconds(1), [&] {
        camera_info_msg.header.stamp = node->now();
        camera_info_pub->publish(camera_info_msg);
    });

    rclcpp::spin(node);
    
    return EXIT_SUCCESS;
}
