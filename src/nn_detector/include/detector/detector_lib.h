#ifndef _DETECTOR_LIB_H_
#define _DETECTOR_LIB_H_

#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <radar_interface/srv/detect.hpp>

// detector
#include <detector/detector.h>
#include <detector/net_decoder.h>

#ifdef TRT
#include <detector/detector_trt.h>
#else
#include <detector/detector_vino.h>
#endif

namespace nn_detector {

struct DetectorParams {
    std::string camera_name;
    std::string robot_name;
    std::filesystem::path node_dir;
    std::string armor_config;
    vision_mode mode;
    bool enable_imshow = false;
    bool debug = false;
    rclcpp::Logger logger = rclcpp::get_logger("nn_detector_lib");
};

class DetectorLib {
   private:
    DetectorParams params;
    std::shared_ptr<Detector> detector_armor;

#ifdef TRT
    CUcontext thread_ctx;
    CUdevice cuda_device;
#endif

   public:
    explicit DetectorLib(const DetectorParams& params);
    radar_interface::srv::Detect_Response::SharedPtr detect(const cv::Mat& img);
};
};  // namespace nn_detector

#endif
