#pragma once

#include <vector>
#include <utility>
#include <functional>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <radar_interface/msg/target_array.hpp>
#include <radar_interface/msg/detected_target_array.hpp>
#include <radar_interface/srv/detect.hpp>
#include <foxglove_msgs/msg/image_marker_array.hpp>
#include <foxglove_msgs/msg/image_annotations.hpp>
#include <foxglove_msgs/msg/text_annotation.hpp>
#include <sensor_msgs/msg/image.hpp>

class TargetDetector {
    using Target = radar_interface::msg::Target;
    using TargetArray = radar_interface::msg::TargetArray;
    using Square = std::pair<cv::Point2d, cv::Point2d>;
    using Image = sensor_msgs::msg::Image;
    using Detect = radar_interface::srv::Detect;
    using DetectRepPtr = radar_interface::srv::Detect::Response::SharedPtr;
    using DetectRepArray = std::vector<DetectRepPtr>;
    using DetectFunc = std::function<DetectRepPtr(const cv::Mat&)>;
    using ProjectFunc = std::function<cv::Point2d(const cv::Point3d&)>;
    using DetectedTargetArray = radar_interface::msg::DetectedTargetArray;
private:
    cv::Mat get_jigsaw_img(const cv::Mat& img, const std::vector<Square>& squares);
public:
    rclcpp::Node::SharedPtr node;
    DetectFunc detect_func;
    ProjectFunc project_func;
    Eigen::Isometry3d trans;
    std::string cam_frame;
    // 拼图边长 默认 3x3
    unsigned jigsaw_size = 3;
    // 生成图像的边长, 根据神经网络, 默认 640x640
    unsigned img_size = 640;

    std::vector<Target> filter_targets(const cv::Point2i& size, const TargetArray::ConstSharedPtr& msg);
    std::vector<Square> get_squares(const std::vector<Target>& targets);
    DetectRepArray detect(const Image::ConstSharedPtr& img_msg, const std::vector<Square>& squares);
    DetectedTargetArray get_detected_targets(const std_msgs::msg::Header& header, const std::vector<Target>& targets, const std::vector<Square>& squares, DetectRepArray& detect_rep);
    foxglove_msgs::msg::ImageMarkerArray get_markers(const std::vector<Square>& squares, const DetectRepArray& detect_rep, const DetectedTargetArray& detected_targets);
    foxglove_msgs::msg::ImageAnnotations get_annotiations(const std::vector<Square>& squares, const DetectedTargetArray& detected_targets);
};
