#ifndef CRH_2023_DECODER_HPP_
#define CRH_2023_DECODER_HPP_

#include <utils/data.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <toml.hpp>
#include <vector>

class NetDecoderBase {
   protected:
    int INPUT_W, INPUT_H, NUM_CLASSES, NUM_COLORS;
    float BBOX_CONF_THRESH;
    rclcpp::Logger logger;
    NetDecoderBase(toml::value &config, const rclcpp::Logger &_logger);

   public:
    virtual void decode(int layer_index, const float *prob, std::vector<Armor> &objects) = 0;
    virtual void set_layer_info(int layer_index, const std::vector<size_t> &dimensions) = 0;
    virtual bool check_num_outputs(int num_outputs) = 0;
};

class YOLOv5Decoder : public NetDecoderBase {
   protected:
    struct YOLOv5LayerInfo {
        int index, num_anchors, out_h, out_w, num_outputs, stride;
    };
    std::vector<std::vector<float>> anchors;
    std::vector<YOLOv5LayerInfo> layers;

   public:
    YOLOv5Decoder(toml::value &, const rclcpp::Logger &);
    virtual void decode(int layer_index, const float *prob, std::vector<Armor> &objects) override;
    virtual void set_layer_info(int, const std::vector<size_t> &) override;
    virtual bool check_num_outputs(int num_outputs) override;
};

class YOLOv5_1_Decoder : public YOLOv5Decoder {
   protected:
    int NUM_TSIZES;

   public:
    YOLOv5_1_Decoder(toml::value &, const rclcpp::Logger &);
    virtual void decode(int layer_index, const float *prob, std::vector<Armor> &objects) override;
    virtual bool check_num_outputs(int num_outputs) override;
};

class YOLOv8Decoder : public NetDecoderBase {
   protected:
    int NUM_KPTS, NUM_TSIZES;
    struct YOLOv8LayerInfo {
        int index, num_outputs, stride;
    };
    std::vector<YOLOv8LayerInfo> layers;

   public:
    YOLOv8Decoder(toml::value &, const rclcpp::Logger &);
    virtual void decode(int layer_index, const float *prob, std::vector<Armor> &objects) override;
    virtual void set_layer_info(int, const std::vector<size_t> &) override;
    virtual bool check_num_outputs(int num_outputs) override;
};

inline float sigmoid(float x) { return (1.0 / (1.0 + exp(-x))); }

/**
 * @brief calculate sigmoid values in an array
 *
 * @param src pointer of source array
 * @param dst pointer of destination array
 * @param length number of values
 */
inline void sigmoid(const float *src, float *dst, int length) {
    for (int i = 0; i < length; ++i) {
        dst[i] = (1.0 / (1.0 + exp(-src[i])));
    }
}

#endif  // CRH_2023_DECODER_HPP_