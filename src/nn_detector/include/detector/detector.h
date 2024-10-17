#ifndef _DETECTOR_H_
#define _DETECTOR_H_

#include <utils/data.h>
#include <detector/net_decoder.h>
// ROS
#include <opencv2/opencv.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

class Detector {
   protected:
    rclcpp::Logger logger;
    std::string type;
    toml::value config;
    Detector(const std::string &config_file, const std::string &share_dir,
             const rclcpp::Logger &_logger)
        : logger(_logger) {
        config = toml::parse(share_dir + "/" + config_file);
    };

    cv::Mat static_resize(cv::Mat img, int INPUT_H, int INPUT_W);

   public:
    virtual std::vector<Armor> detect(cv::Mat);
    virtual void draw(cv::Mat, const std::vector<Armor> &);
};

class NetDetector : public Detector {
   protected:
    // for drawing
    std::vector<std::string> class_names, color_names, tsize_names;
    int INPUT_W, INPUT_H, NUM_CLASSES, NUM_COLORS;
    float BBOX_CONF_THRESH, NMS_THRESH, MERGE_THRESH;
    int point_num;

    int layer_num;
    std::shared_ptr<NetDecoderBase> decoder;
    std::string model_prefix;

    std::vector<Armor> do_nms(std::vector<Armor> &);
    std::vector<Armor> do_merge_nms(std::vector<Armor> &);
    void img2blob(cv::Mat, std::vector<float> &);
    void img2blob(cv::Mat, float*);
    NetDetector(const std::string &config_file, const std::string &share_dir,
                const rclcpp::Logger &_logger);

   public:
    void draw(cv::Mat, const std::vector<Armor> &) override;
};

#endif