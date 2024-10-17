#include <detector/net_decoder.h>
#include <utils/common.h>

NetDecoderBase::NetDecoderBase(toml::value &config,const rclcpp::Logger& _logger): logger(_logger) {
    INPUT_W = config.at("INPUT_W").as_integer();
    INPUT_H = config.at("INPUT_H").as_integer();

    NUM_CLASSES = config.at("NUM_CLASSES").as_integer();
    NUM_COLORS = config.at("NUM_COLORS").as_integer();

    BBOX_CONF_THRESH = config.at("BBOX_CONF_THRESH").as_floating();
}

YOLOv5Decoder::YOLOv5Decoder(toml::value &config, const rclcpp::Logger& _logger) : NetDecoderBase(config,_logger) {
    anchors = toml::get<decltype(anchors)>(config.at("anchors"));
}

void YOLOv5Decoder::set_layer_info(int layer_index, const std::vector<size_t> &dimensions) {
    assert((int)layers.size() == layer_index && "set_layer_info should be called in order");
    assert(dimensions.size() == 5 && "model should be 5-dim");
    assert(dimensions[2] == dimensions[3] && "model output should be square");
    int stride = INPUT_H / dimensions[2];

    YOLOv5LayerInfo layer = {layer_index,(int)dimensions[1],(int)dimensions[2],(int)dimensions[3],(int)dimensions[4],stride};

    RCLCPP_INFO(logger,"layer %d: num_anchors=%d, out_h=%d, out_w=%d, num_outputs=%d, stride=%d", layer_index, layer.num_anchors, layer.out_h, layer.out_w, layer.num_outputs, layer.stride);
    assert(this->check_num_outputs(layer.num_outputs) && "num_output check failed");

    layers.push_back(layer);
    return;
}

bool YOLOv5Decoder::check_num_outputs(int num_outputs) { return num_outputs == 1 + 4 + 10 + NUM_CLASSES + NUM_COLORS; }

void YOLOv5Decoder::decode(int layer_index, const float *prob, std::vector<Armor> &objects) {
    assert((int)layers.size() > layer_index && "layer_index out of range");
    auto [_, na, out_h, out_w, no, stride] = layers[layer_index];

    std::vector<float> anchor;
    switch (stride) {
        case 8:
            anchor = anchors[0];
            break;
        case 16:
            anchor = anchors[1];
            break;
        case 32:
            anchor = anchors[2];
            break;
        case 4:
            anchor = anchors[3];
            break;
        default:
            assert(false && "Unknown layer stride");
            break;
    }
    std::vector<float> pred_data_v;
    pred_data_v.resize(no);
    float* pred_data = pred_data_v.data();
    // float pred_data[no];
    // [x, y, w, h, conf, (x,y)*5, hot(class), hot(color)]
    for (int a_id = 0; a_id < na; ++a_id) {
        for (int h_id = 0; h_id < out_h; ++h_id) {
            for (int w_id = 0; w_id < out_w; ++w_id) {
                int data_idx = (a_id * out_h * out_w + h_id * out_w + w_id) * no;
                float obj_conf = sigmoid(prob[data_idx + 4]);
                if (obj_conf > BBOX_CONF_THRESH) {
                    // std::cout << obj_conf << std::endl;
                    sigmoid(prob + data_idx, pred_data, 5);
                    sigmoid(prob + data_idx + 15, pred_data + 15, NUM_CLASSES + NUM_COLORS);
                    memcpy(pred_data + 5, prob + data_idx + 5, sizeof(float) * 10);
                    int cls_id = std::distance(pred_data + 15, std::max_element(pred_data + 15, pred_data + 15 + NUM_CLASSES));
                    int col_id = std::distance(pred_data + 15 + NUM_CLASSES, std::max_element(pred_data + 15 + NUM_CLASSES, pred_data + 15 + NUM_CLASSES + NUM_COLORS));

                    double final_conf = obj_conf * sqrt(pred_data[15 + cls_id] * pred_data[15 + NUM_CLASSES + col_id]);
                    if (final_conf > BBOX_CONF_THRESH) {
                        // std::cout << final_conf << " " << col_id << " "
                        //           << cls_id << std::endl;
                        Armor now;
                        float x = (pred_data[0] * 2.0 - 0.5 + w_id) * stride;
                        float y = (pred_data[1] * 2.0 - 0.5 + h_id) * stride;
                        float w = pow(pred_data[2] * 2, 2) * anchor[a_id * 2];
                        float h = pow(pred_data[3] * 2, 2) * anchor[a_id * 2 + 1];

                        for (int p = 0; p < 5; ++p) {
                            float px = (pred_data[5 + p * 2] * anchor[a_id * 2] + w_id * stride);
                            float py = (pred_data[5 + p * 2 + 1] * anchor[a_id * 2 + 1] + h_id * stride);
                            // px = std::max(std::min(px, (float)(INPUT_W)), 0.f);
                            // py = std::max(std::min(py, (float)(INPUT_H)), 0.f);
                            now.pts[p] = cv::Point2f(px, py);
                            // std::cout << px << " " << py  << " ";
                        }
                        // std::cout << std::endl;

                        float x0 = (x - w * 0.5);
                        float y0 = (y - h * 0.5);
                        float x1 = (x + w * 0.5);
                        float y1 = (y + h * 0.5);

                        // x0 = std::max(std::min(x0, (float)(INPUT_W)), 0.f);
                        // y0 = std::max(std::min(y0, (float)(INPUT_H)), 0.f);
                        // x1 = std::max(std::min(x1, (float)(INPUT_W)), 0.f);
                        // y1 = std::max(std::min(y1, (float)(INPUT_H)), 0.f);

                        now.rect = cv::Rect(x0, y0, x1 - x0, y1 - y0);
                        now.conf = final_conf;
                        now.color = col_id;
                        now.type = cls_id;
                        now.size = 0;
                        objects.push_back(now);
                    }
                }
            }
        }
    }
    return;
}

YOLOv5_1_Decoder::YOLOv5_1_Decoder(toml::value &config, const rclcpp::Logger& _logger) : YOLOv5Decoder(config, _logger) {
    NUM_TSIZES = config.at("NUM_TSIZES").as_integer();
}

bool YOLOv5_1_Decoder::check_num_outputs(int num_outputs) { return num_outputs == 1 + 4 + 10 + NUM_CLASSES + NUM_COLORS + NUM_TSIZES; }

void YOLOv5_1_Decoder::decode(int layer_index, const float *prob, std::vector<Armor> &objects) {
    assert((int)layers.size() > layer_index && "layer_index out of range");
    auto [_, na, out_h, out_w, no, stride] = layers[layer_index];

    std::vector<float> anchor;
    switch (stride) {
        case 8:
            anchor = anchors[0];
            break;
        case 16:
            anchor = anchors[1];
            break;
        case 32:
            anchor = anchors[2];
            break;
        case 4:
            anchor = anchors[3];
            break;
        default:
            assert(false && "Unknown layer stride");
            break;
    }
    std::vector<float> pred_data_v;
    pred_data_v.resize(no);
    float* pred_data = pred_data_v.data();
    // [x, y, w, h, conf, (x,y)*5, hot(class), hot(color), hot(tsize)]
    for (int a_id = 0; a_id < na; ++a_id) {
        for (int h_id = 0; h_id < out_h; ++h_id) {
            for (int w_id = 0; w_id < out_w; ++w_id) {
                int data_idx = (a_id * out_h * out_w + h_id * out_w + w_id) * no;
                float obj_conf = sigmoid(prob[data_idx + 4]);
                if (obj_conf > BBOX_CONF_THRESH) {
                    // std::cout << obj_conf << std::endl;
                    sigmoid(prob + data_idx, pred_data, 5);
                    sigmoid(prob + data_idx + 15, pred_data + 15, NUM_CLASSES + NUM_COLORS + NUM_TSIZES);
                    memcpy(pred_data + 5, prob + data_idx + 5, sizeof(float) * 10);
                    int cls_id = std::distance(pred_data + 15, std::max_element(pred_data + 15, pred_data + 15 + NUM_CLASSES));
                    int col_id = std::distance(pred_data + 15 + NUM_CLASSES, std::max_element(pred_data + 15 + NUM_CLASSES, pred_data + 15 + NUM_CLASSES + NUM_COLORS));
                    int ts_id =
                        std::distance(pred_data + 15 + NUM_CLASSES + NUM_COLORS, std::max_element(pred_data + 15 + NUM_CLASSES + NUM_COLORS, pred_data + 15 + NUM_CLASSES + NUM_COLORS + NUM_TSIZES));

                    double final_conf = obj_conf * pow(pred_data[15 + cls_id] * pred_data[15 + NUM_CLASSES + col_id] * pred_data[15 + NUM_CLASSES + NUM_COLORS + ts_id], 1.0 / 3.0);
                    if (final_conf > BBOX_CONF_THRESH) {
                        // std::cout << final_conf << " " << col_id << " "
                        //           << cls_id << std::endl;
                        Armor now;
                        float x = (pred_data[0] * 2.0 - 0.5 + w_id) * stride;
                        float y = (pred_data[1] * 2.0 - 0.5 + h_id) * stride;
                        float w = pow(pred_data[2] * 2, 2) * anchor[a_id * 2];
                        float h = pow(pred_data[3] * 2, 2) * anchor[a_id * 2 + 1];

                        for (int p = 0; p < 5; ++p) {
                            float px = (pred_data[5 + p * 2] * anchor[a_id * 2] + w_id * stride);
                            float py = (pred_data[5 + p * 2 + 1] * anchor[a_id * 2 + 1] + h_id * stride);
                            // px = std::max(std::min(px, (float)(INPUT_W)), 0.f);
                            // py = std::max(std::min(py, (float)(INPUT_H)), 0.f);
                            now.pts[p] = cv::Point2f(px, py);
                            // std::cout << px << " " << py  << " ";
                        }
                        // std::cout << std::endl;

                        float x0 = (x - w * 0.5);
                        float y0 = (y - h * 0.5);
                        float x1 = (x + w * 0.5);
                        float y1 = (y + h * 0.5);

                        // x0 = std::max(std::min(x0, (float)(INPUT_W)), 0.f);
                        // y0 = std::max(std::min(y0, (float)(INPUT_H)), 0.f);
                        // x1 = std::max(std::min(x1, (float)(INPUT_W)), 0.f);
                        // y1 = std::max(std::min(y1, (float)(INPUT_H)), 0.f);

                        now.rect = cv::Rect(x0, y0, x1 - x0, y1 - y0);
                        now.conf = final_conf;
                        now.color = col_id;
                        now.type = cls_id;
                        now.size = ts_id;
                        objects.push_back(now);
                    }
                }
            }
        }
    }
    return;
}

YOLOv8Decoder::YOLOv8Decoder(toml::value &config, const rclcpp::Logger& _logger) : NetDecoderBase(config, _logger) {
    NUM_KPTS = config.at("NUM_KPTS").as_integer();
    NUM_TSIZES = config.at("NUM_TSIZES").as_integer();
}

void YOLOv8Decoder::set_layer_info(int layer_index, const std::vector<size_t> &dims) {
    assert((int)layers.size() == layer_index && "set_layer_info should be called in order");
    assert(dims.size() == 3 && "model should be 3-dim");
    assert(dims[1] == 8400);

    // YOLOv8LayerInfo layer{
    //     .index = layer_index,
    //     .num_outputs = static_cast<int>(dims[2]),
    // };
    YOLOv8LayerInfo layer = {layer_index, (int)dims[2], 0};

    RCLCPP_INFO(logger,"layer %d: num_outputs=%d", layer_index, layer.num_outputs);
    assert(this->check_num_outputs(layer.num_outputs) && "num_output check failed");

    layers.push_back(layer);
    return;
}

bool YOLOv8Decoder::check_num_outputs(int num_outputs) { return num_outputs == NUM_KPTS + NUM_CLASSES + NUM_COLORS + NUM_TSIZES; }

void YOLOv8Decoder::decode(int layer_index, const float *prob, std::vector<Armor> &objects) {
    assert((int)layers.size() > layer_index && "layer_index out of range");
    int no = layers[layer_index].num_outputs;
    std::vector<float> pred_data_v;
    pred_data_v.resize(no);
    float* pred_data = pred_data_v.data();
    // [kpts(8), hot(classes)(8), hot(tsizes)(2), hot(colors)(4)]
    for (int idx = 0; idx < 8400; ++idx) {
        float rough_conf = *std::max_element(&prob[idx * no + NUM_KPTS], &prob[idx * no + no]);

        if (rough_conf > BBOX_CONF_THRESH) {
            std::memcpy(pred_data, &prob[idx * no], no * sizeof(float));
            int cls_id = std::distance(pred_data + NUM_KPTS, std::max_element(pred_data + NUM_KPTS, pred_data + NUM_KPTS + NUM_CLASSES));
            int ts_id = std::distance(pred_data + NUM_KPTS + NUM_CLASSES, std::max_element(pred_data + NUM_KPTS + NUM_CLASSES, pred_data + NUM_KPTS + NUM_CLASSES + NUM_TSIZES));
            int col_id = std::distance(pred_data + NUM_KPTS + NUM_CLASSES + NUM_TSIZES,
                                       std::max_element(pred_data + NUM_KPTS + NUM_CLASSES + NUM_TSIZES, pred_data + NUM_KPTS + NUM_CLASSES + NUM_TSIZES + NUM_COLORS));

            double final_conf = std::min({pred_data[NUM_KPTS + NUM_CLASSES + NUM_TSIZES + col_id], pred_data[NUM_KPTS + cls_id]});
            if (final_conf > BBOX_CONF_THRESH) {
                // std::cout << final_conf << " " << col_id << " "
                //           << cls_id << std::endl;
                Armor now;

                for (int p = 0; p < (NUM_KPTS / 2); ++p) {
                    // float px = std::max(std::min(pred_data[p * 2], (float)(INPUT_W)), 0.f);
                    // float py = std::max(std::min(pred_data[p * 2 + 1], (float)(INPUT_H)), 0.f);
                    float px = pred_data[p * 2];
                    float py = pred_data[p * 2 + 1];
                    now.pts[p] = cv::Point2f(px, py);
                }

                now.rect = cv::Rect(now.pts[0], now.pts[2]);
                now.conf = final_conf;
                now.color = col_id;
                now.type = cls_id;
                now.size = ts_id;
                objects.push_back(now);
            }
        }
    }
}