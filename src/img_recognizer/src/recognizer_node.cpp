#include "img_recognizer/recognizer_node.hpp"
#include "img_recognizer/target_detector.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Eigen>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>

using namespace img_recognizer;

RecognizerNode::RecognizerNode(const rclcpp::NodeOptions& options)
    : Node("img_recognizer", options)
{
    declare_parameter("target_topic", "/radar/pc_detector/targets");
    declare_parameter("sync_queue_size", 20);
    declare_parameter("crop_side_length", 1500.0);
    declare_parameter("img_compressed", false);
    declare_parameter("jigsaw_size", 3);
    declare_parameter("img_size", 640);

    bool use_intra = options.use_intra_process_comms();
    if (!use_intra) {
        RCLCPP_WARN(get_logger(), "Not In Intra Process Mode");
    }

    auto ns = std::string_view(get_namespace());
    auto ns_pos = ns.rfind('/');
    if (ns_pos != std::string_view::npos && ns_pos + 1 < ns.size()) {
        cam_frame = ns.substr(ns.rfind('/') + 1);
        cam_frame.append("_frame");
    } else {
        cam_frame = "default_camera_frame";
    }

    nn_detector::DetectorParams params;
    // load params
    params.armor_config = declare_parameter("armor_detector_config", "");
    params.enable_imshow = declare_parameter("enable_imshow", false);
    params.debug = declare_parameter("debug", false);

    params.node_dir = ament_index_cpp::get_package_share_directory("nn_detector");
    params.logger = get_logger();

    tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    detector_lib = std::make_shared<nn_detector::DetectorLib>(params);

    auto ex_name = [this](const std::string& x) {
        return rclcpp::expand_topic_or_service_name(x, get_name(), get_namespace());
    };
    cam_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", rclcpp::SystemDefaultsQoS(), std::bind(&RecognizerNode::camera_info_callback, this, std::placeholders::_1));
    if (get_parameter("img_compressed").as_bool())
        img_sub.subscribe(this, ex_name("image"), "compressed");
    else
        img_sub.subscribe(this, ex_name("image"), "raw");
    pc_target_sub.subscribe(this, get_parameter("target_topic").as_string());
    sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(get_parameter("sync_queue_size").as_int()), img_sub, pc_target_sub);
    sync->registerCallback(std::bind(&RecognizerNode::sync_callback, this, std::placeholders::_1, std::placeholders::_2));

    markers_pub = this->create_publisher<foxglove_msgs::msg::ImageMarkerArray>("img_recognizer/markers", rclcpp::QoS(rclcpp::KeepLast(10)));
    annotations_pub = this->create_publisher<foxglove_msgs::msg::ImageAnnotations>("img_recognizer/annotations", rclcpp::QoS(rclcpp::KeepLast(10)));
    detected_targets_pub = this->create_publisher<radar_interface::msg::DetectedTargetArray>("img_recognizer/detected_targets", rclcpp::QoS(rclcpp::KeepLast(10)));

    // nn_helper = std::make_shared<NnHelperNode>(options);

    RCLCPP_INFO(this->get_logger(), "img_recognizer node started.");
}

void RecognizerNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    // 为什么不用 image_transport::CameraSubscriber? 因为我们已经假定了相机的内参是固定的, 不经常变化
    cam_model.fromCameraInfo(msg);
}

void RecognizerNode::sync_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg, const radar_interface::msg::TargetArray::ConstSharedPtr &target_msg)
{
    TargetDetector detector;
    try {
        detector.trans = tf2::transformToEigen(
            tf_buffer->lookupTransform(cam_frame, "world", tf2::TimePointZero, tf2::durationFromSec(1.0)));
    } catch (const tf2::TransformException& e) {
        RCLCPP_ERROR(get_logger(), "%s", e.what());
        return;
    }
    detector.cam_frame = cam_frame;
    detector.node = this->shared_from_this();
    detector.detect_func = std::bind(&nn_detector::DetectorLib::detect, *detector_lib, std::placeholders::_1);
    detector.project_func = std::bind(&image_geometry::PinholeCameraModel::project3dToPixel, &cam_model, std::placeholders::_1);
    detector.jigsaw_size = get_parameter("jigsaw_size").as_int();
    detector.img_size = get_parameter("img_size").as_int();

    auto filtered = detector.filter_targets(cv::Point2i(img_msg->width, img_msg->height), target_msg);
    auto squares = detector.get_squares(filtered);
    auto detect_rep = detector.detect(img_msg, squares);
    auto detected_targets = detector.get_detected_targets(target_msg->header, filtered, squares, detect_rep);
    detected_targets_pub->publish(detected_targets);
    markers_pub->publish(detector.get_markers(squares, detect_rep, detected_targets));
    annotations_pub->publish(detector.get_annotiations(squares, detected_targets));
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(img_recognizer::RecognizerNode)
