#include <utils/common.h>
#include <detector/detector.h>

std::vector<Armor> Detector::detect(cv::Mat) {
    RCLCPP_ERROR(logger, "ERROR base detector detect");
    return {};
}

void Detector::draw(cv::Mat, const std::vector<Armor>&) {
    RCLCPP_ERROR(logger, "ERROR base detector draw");
}

cv::Mat Detector::static_resize(cv::Mat img, int INPUT_H, int INPUT_W) {
    float r = std::min(INPUT_W / (img.cols * 1.0), INPUT_H / (img.rows * 1.0));
    // r = std::min(r, 1.0f);
    int unpad_w = r * img.cols;
    int unpad_h = r * img.rows;
    cv::Mat re(unpad_h, unpad_w, CV_8UC3);
    cv::resize(img, re, re.size());
    // cv::Mat out(INPUT_W, INPUT_H, CV_8UC3, cv::Scalar(114, 114, 114));
    cv::Mat out(INPUT_H, INPUT_W, CV_8UC3, cv::Scalar(114, 114, 114));
    re.copyTo(out(cv::Rect(0, 0, re.cols, re.rows)));
    return out;
}

NetDetector::NetDetector(const std::string& config_file, const std::string& share_dir,
                         const rclcpp::Logger& _logger)
    : Detector(config_file, share_dir, _logger) {
    type = config.at("type").as_string();

    INPUT_W = config.at("INPUT_W").as_integer();
    INPUT_H = config.at("INPUT_H").as_integer();

    NUM_CLASSES = config.at("NUM_CLASSES").as_integer();
    NUM_COLORS = config.at("NUM_COLORS").as_integer();

    BBOX_CONF_THRESH = config.at("BBOX_CONF_THRESH").as_floating();
    NMS_THRESH = config.at("NMS_THRESH").as_floating();
    MERGE_THRESH = config.at("MERGE_THRESH").as_floating();

    point_num = config.at("point_num").as_integer();
    class_names = toml::get<std::vector<std::string>>(config.at("class_names"));
    color_names = toml::get<std::vector<std::string>>(config.at("color_names"));
    tsize_names = toml::get<std::vector<std::string>>(config.at("tsize_names"));

    model_prefix = share_dir + "/" + std::string(config.at("model_prefix").as_string());

    if (type == "V8") {
        this->decoder = std::make_shared<YOLOv8Decoder>(config, logger);
    } else if (type == "V5.1") {
        this->decoder = std::make_shared<YOLOv5_1_Decoder>(config, logger);
    } else if (type == "V5") {
        this->decoder = std::make_shared<YOLOv5Decoder>(config, logger);
    } else {
        RCLCPP_ERROR(logger, "Invalid type for NetDecoder: %s", type.c_str());
        rclcpp::shutdown();
    }
}

std::vector<Armor> NetDetector::do_nms(std::vector<Armor>& objects) {
    std::vector<int> classIds;
    std::vector<int> indices;
    std::vector<float> confidences;
    std::vector<cv::Rect> bboxes;
    std::vector<Armor> result;
    for (size_t i = 0; i < objects.size(); ++i) {
        bboxes.push_back(objects[i].rect);
        confidences.push_back(objects[i].conf);
        int cls_id = objects[i].color * 9 + objects[i].type;
        classIds.push_back(cls_id);
    }
    cv::dnn::NMSBoxes(bboxes, confidences, BBOX_CONF_THRESH, NMS_THRESH, indices);
    for (size_t i = 0; i < indices.size(); ++i) {
        result.push_back(objects[indices[i]]);
    }
    return result;
}

float calc_iou(const Armor& a, const Armor& b) {
    cv::Rect_<float> inter = a.rect & b.rect;
    float inter_area = inter.area();
    float union_area = a.rect.area() + b.rect.area() - inter_area;
    return inter_area / union_area;
}

/**
 * @brief do Merge Non-Maximum Suppression on detected objects, avoiding overlapped objects
 *
 * @param objects all detected objects
 * @return std::vector<armor::Armor> result
 */
std::vector<Armor> NetDetector::do_merge_nms(std::vector<Armor>& objects) {
    struct pick_merge_store {
        int id;
        std::vector<cv::Point2f> merge_pts;
        std::vector<float> merge_confs;
    };
    struct armor_compare {
        bool operator()(const Armor& a, const Armor& b) { return a.conf > b.conf; }
    };

    std::vector<Armor> result;
    std::vector<pick_merge_store> picked;

    std::sort(objects.begin(), objects.end(), armor_compare());
    for (int i = 0; i < (int)objects.size(); ++i) {
        const Armor& now = objects[i];
        bool keep = 1;
        for (int j = 0; j < (int)picked.size(); ++j) {
            const Armor& pre = objects[picked[j].id];
            float iou = calc_iou(now, pre);

            // store for merge_nms
            if (iou > NMS_THRESH || isnan(iou)) {
                keep = 0;
                if (iou > MERGE_THRESH && now.color == pre.color && now.type == pre.type &&
                    now.size == pre.size) {
                    picked[j].merge_confs.push_back(now.conf);
                    for (int k = 0; k < 5; ++k) {
                        picked[j].merge_pts.push_back(now.pts[k]);
                    }
                }
                break;
            }
        }
        if (keep) {
            picked.push_back({i, {}, {}});
        }
    }
    for (int i = 0; i < (int)picked.size(); ++i) {
        int merge_num = picked[i].merge_confs.size();
        Armor now = objects[picked[i].id];
        double conf_sum = now.conf;
        for (int j = 0; j < 5; ++j) now.pts[j] *= now.conf;
        for (int j = 0; j < merge_num; ++j) {
            for (int k = 0; k < 5; ++k) {
                now.pts[k] += picked[i].merge_pts[j * 5 + k] * picked[i].merge_confs[j];
            }
            conf_sum += picked[i].merge_confs[j];
        }
        for (int j = 0; j < 5; ++j) now.pts[j] /= conf_sum;
        result.emplace_back(now);
    }
    return result;
}

void NetDetector::img2blob(cv::Mat img, std::vector<float> &blob_data) {
    // cv::dnn::blobFromImage(img, 1./255, img.size, cv::Scalar(), true);
    // return;
    img2blob(img, blob_data.data());
}

void NetDetector::img2blob(cv::Mat img, float* blob_data) {
    // cv::dnn::blobFromImage(img, 1./255, img.size, cv::Scalar(), true);
    // return;
    int img_h = img.rows;
    int img_w = img.cols;

    for (int i = 0, row = 0; row < img_h; ++row) {
        uchar* uc_pixel = img.data + row * img.step;
        for (int col = 0; col < img_w; ++col) {
            // 三通道
            blob_data[i] = (float)uc_pixel[2] / 255.0;
            blob_data[i + img_h * img_w] = (float)uc_pixel[1] / 255.0;
            blob_data[i + 2 * img_h * img_w] = (float)uc_pixel[0] / 255.0;
            uc_pixel += 3;
            ++i;
        }
    }
}

void NetDetector::draw(cv::Mat img, const std::vector<Armor>& objects) {
    for (auto& obj : objects) {
        char text[256];

        cv::Scalar color = cv::Scalar(0, 1, 0);
        float c_mean = cv::mean(color)[0];
        cv::Scalar txt_color;
        if (c_mean > 0.5) {
            txt_color = cv::Scalar(0, 0, 0);
        } else {
            txt_color = cv::Scalar(255, 255, 255);
        }

        cv::Point2f obj_pts[5];
        for (int p = 0; p < point_num; ++p) {
            obj_pts[p] = obj.pts[p];
        }
        // cv::rectangle(image, obj.rect, color * 255, 2);
        for (int p = 0; p < point_num; ++p) {
            cv::line(img, obj_pts[p], obj_pts[(p + 1) % point_num], color * 255, 2);
        }
        // for (int p = 0; p < 5; ++p) {
        //     cv::circle(image, obj.pts[p], 2, cv::Scalar(0, 0, 255));
        // }

        sprintf(text, "%s%s%s %.1f%%", color_names[obj.color].c_str(),
                tsize_names[obj.size].c_str(), class_names[obj.type].c_str(), obj.conf * 100);

        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseLine);

        cv::Scalar txt_bk_color = color * 0.7 * 255;

        int x = obj.rect.x;
        int y = obj.rect.y + 1;

        x = std::max(std::min(x, img.cols), 0);
        y = std::max(std::min(y, img.rows), 0);

        cv::rectangle(
            img,
            cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),
            txt_bk_color, -1);

        cv::putText(img, text, cv::Point(x, y + label_size.height), cv::FONT_HERSHEY_SIMPLEX, 0.4,
                    txt_color, 1);
    }
}