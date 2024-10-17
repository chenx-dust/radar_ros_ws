#pragma once
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <visualization_msgs/msg/image_marker.hpp>
#include <foxglove_msgs/msg/image_marker_array.hpp>
#include <foxglove_msgs/msg/image_annotations.hpp>

#include <radar_interface/msg/target_array.hpp>
#include <radar_interface/msg/detected_target.hpp>
#include <radar_interface/msg/detected_target_array.hpp>

#include <detector/detector_lib.h>


namespace img_recognizer {
class RecognizerNode : public rclcpp::Node {
private:
    std::string cam_frame;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub;
    image_transport::SubscriberFilter img_sub;
    message_filters::Subscriber<radar_interface::msg::TargetArray> pc_target_sub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, radar_interface::msg::TargetArray> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;

    rclcpp::Publisher<foxglove_msgs::msg::ImageMarkerArray>::SharedPtr markers_pub;
    rclcpp::Publisher<foxglove_msgs::msg::ImageAnnotations>::SharedPtr annotations_pub;
    rclcpp::Publisher<radar_interface::msg::DetectedTargetArray>::SharedPtr detected_targets_pub;

    image_geometry::PinholeCameraModel cam_model;
    tf2_ros::Buffer::SharedPtr tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    std::shared_ptr<nn_detector::DetectorLib> detector_lib;
    // std::shared_ptr<NnHelperNode> nn_helper;

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void sync_callback(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg, const radar_interface::msg::TargetArray::ConstSharedPtr& target_msg);
    visualization_msgs::msg::ImageMarker get_marker(cv::Point3d pt);
public:
    RecognizerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
};
}
