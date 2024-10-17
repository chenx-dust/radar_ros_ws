#ifndef _DETECTOR_NODE_H
#define _DETECTOR_NODE_H
#include <filesystem>
// ROS
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <radar_interface/srv/detect.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

// detector
#include <detector/detector.h>
#include <detector/detector_lib.h>
#include <detector/net_decoder.h>

#ifdef TRT
#include <detector/detector_trt.h>
#else
#include <detector/detector_vino.h>
#endif

namespace nn_detector {

class DetectorNode : public rclcpp::Node {
    std::shared_ptr<DetectorLib> core;

    rclcpp::Service<radar_interface::srv::Detect>::SharedPtr detect_service;

    void detect_service_callback(const radar_interface::srv::Detect::Request::SharedPtr req,
                                 radar_interface::srv::Detect::Response::SharedPtr rep);

   public:
    explicit DetectorNode(const rclcpp::NodeOptions& options);
};
};  // namespace nn_detector

#endif