#ifndef CRH_2023_DETECTOR_VINO_H
#define CRH_2023_DETECTOR_VINO_H

#include <detector/detector.h>

#include <openvino/openvino.hpp>

class DetectorVINO : public NetDetector {
   private:
    ov::Core core;
    ov::CompiledModel compiled_model;
    ov::InferRequest infer_request;

   public:
    DetectorVINO(const std::string &config_file, const std::string &share_dir,
                 const rclcpp::Logger &_logger);
    std::vector<Armor> detect(cv::Mat) override;
};

#endif