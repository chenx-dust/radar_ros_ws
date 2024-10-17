#include <cv_bridge/cv_bridge.h>
#include <detector/detector_node.h>
#include <utils/common.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace nn_detector;

DetectorNode::DetectorNode(const rclcpp::NodeOptions& options) : Node("nn_detector", options) {
    RCLCPP_INFO(this->get_logger(), "Starting Robomaster Detector Node!");

    bool use_intra = options.use_intra_process_comms();
    if (!use_intra) {
        RCLCPP_WARN(get_logger(), "Not In Intra Process Mode");
    }

    DetectorParams params;
    // load params
    params.armor_config = declare_parameter("armor_detector_config", "");
    params.enable_imshow = declare_parameter("enable_imshow", false);
    params.debug = declare_parameter("debug", false);

    params.node_dir = ament_index_cpp::get_package_share_directory("nn_detector");
    params.logger = get_logger();

    core = std::make_shared<DetectorLib>(params);

    // 注册sub/pub
    detect_service = this->create_service<radar_interface::srv::Detect>(
        "detect_armor", std::bind(&DetectorNode::detect_service_callback, this, std::placeholders::_1, std::placeholders::_2));
}

void DetectorNode::detect_service_callback(const radar_interface::srv::Detect::Request::SharedPtr req,
                                           radar_interface::srv::Detect::Response::SharedPtr rep) {
    // auto img = cv_bridge::toCvShare(img_msg, "bgr8")->image;
    cv::Mat img(req->image.height, req->image.width, encoding2mat_type(req->image.encoding),
                req->image.data.data());
    // For test intra comms
    // RCLCPP_INFO(get_logger(),"PID: %d PTR: %p",getpid(),(void*)img.data);
    *rep = *core->detect(img);
}


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nn_detector::DetectorNode)
// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<DetectorNode>(rclcpp::NodeOptions()));
//     rclcpp::shutdown();
//     return 0;
// }
