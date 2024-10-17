#include <ranges>
#include <cv_bridge/cv_bridge.h>
#include <img_recognizer/target_detector.hpp>
#include <ctime>

std::vector<TargetDetector::Target> TargetDetector::filter_targets(const cv::Point2i& img_size, const TargetArray::ConstSharedPtr& msg)
{
    std::vector<TargetDetector::Target> rtn;
    for (auto& target : msg->targets) {
        Eigen::Vector3d cam_pt3_eigen = trans * Eigen::Vector3d(target.position[0], target.position[1], target.calc_z);
        cv::Point3d cam_pt3(cam_pt3_eigen.x(), cam_pt3_eigen.y(), cam_pt3_eigen.z());
        cv::Point2d cam_pt2 = project_func(cam_pt3);
        if (cam_pt2.x >= 0 && cam_pt2.y >= 0 && cam_pt2.x < img_size.x && cam_pt2.y < img_size.y)
            rtn.emplace_back(target);
    }
    return rtn;
}

cv::Mat TargetDetector::get_jigsaw_img(const cv::Mat& img, const std::vector<Square>& squares)
{
    cv::Mat jigsaw_img = cv::Mat::zeros(img_size, img_size, CV_8UC3);
    for (unsigned i = 0; i * jigsaw_size < squares.size(); ++i) {
        for (unsigned j = 0; j < jigsaw_size && i * jigsaw_size + j < squares.size(); ++j) {
            auto s = squares[i * jigsaw_size + j];
            if (s.first.x >= s.second.x || s.first.y >= s.second.y)
                continue;
            cv::Mat roi_img;
            if (s.first.x < 0 || s.first.y < 0 || s.second.x >= img.cols || s.second.y >= img.rows) {
                cv::Point2i p1(std::max(0, int(s.first.x)), std::max(0, int(s.first.y)));
                cv::Point2i p2(std::min(img.cols - 1, int(s.second.x)), std::min(img.rows - 1, int(s.second.y)));
                if (p1.x >= p2.x || p1.y >= p2.y)
                    continue;
                int max_size = std::max(p2.x - p1.x, p2.y - p1.y);
                roi_img = cv::Mat::zeros(max_size, max_size, CV_8UC3);
                cv::Mat roi = img(cv::Rect(p1.x, p1.y, p2.x - p1.x, p2.y - p1.y));
                roi.copyTo(roi_img(cv::Rect(0, 0, roi.cols, roi.rows)));
            } else {
                cv::Rect roi(s.first.x, s.first.y, s.second.x - s.first.x, s.second.y - s.first.y);
                roi_img = img(roi);
            }
            cv::resize(roi_img, roi_img, cv::Size(img_size / jigsaw_size, img_size / jigsaw_size));
            roi_img.copyTo(jigsaw_img(cv::Rect(j * img_size / jigsaw_size, i * img_size / jigsaw_size, img_size / jigsaw_size, img_size / jigsaw_size)));
        }
    }
    // static cv::VideoWriter writer("detect.mkv", cv::VideoWriter::fourcc('X', '2', '6', '4'), 24, cv::Size(img_size, img_size));
    // writer.write(jigsaw_img);
    // cv::imwrite("dataset/" + std::to_string(std::time(0)) + ".png", jigsaw_img);
    return jigsaw_img;
}

std::vector<TargetDetector::Square> TargetDetector::get_squares(const std::vector<Target>& targets)
{
    double side_length_k = node->get_parameter("crop_side_length").as_double();
    std::vector<Square> squares;
    for (const auto& target : targets) {
        Eigen::Vector3d cam_pt3_eigen = trans * Eigen::Vector3d(target.position[0], target.position[1], target.calc_z);
        cv::Point3d cam_pt3(cam_pt3_eigen.x(), cam_pt3_eigen.y(), cam_pt3_eigen.z());
        cv::Point2d cam_pt2 = project_func(cam_pt3);
        double side_length = side_length_k / std::sqrt(cam_pt3.x * cam_pt3.x + cam_pt3.y * cam_pt3.y + cam_pt3.z * cam_pt3.z);
        Square square { cv::Point2d(cam_pt2.x - side_length / 2, cam_pt2.y - side_length / 2),
            cv::Point2d(cam_pt2.x + side_length / 2, cam_pt2.y + side_length / 2) };
        squares.push_back(square);
    }
    return squares;
}

TargetDetector::DetectRepArray TargetDetector::detect(const Image::ConstSharedPtr& img_msg, const std::vector<Square>& squares)
{
    cv::Mat img = cv_bridge::toCvShare(img_msg, "bgr8")->image;
    // 我好想用 C++20 阿啊啊啊啊
    std::vector<Square> one_jigsaw;
    DetectRepArray detect_rep;
    for (const auto& square : squares) {
        if (one_jigsaw.size() == jigsaw_size * jigsaw_size) {
            detect_rep.push_back(detect_func(get_jigsaw_img(img, one_jigsaw)));
            one_jigsaw.clear();
        }
        one_jigsaw.push_back(square);
    }
    if (!one_jigsaw.empty())
        detect_rep.push_back(detect_func(get_jigsaw_img(img, one_jigsaw)));
    return detect_rep;
}

TargetDetector::DetectedTargetArray TargetDetector::get_detected_targets(const std_msgs::msg::Header& header, const std::vector<Target>& targets, const std::vector<Square>& squares, DetectRepArray& detect_rep)
{
    // NOTE: 这里会将 armor 的角点转换到相机坐标系下
    assert(targets.size() == squares.size());
    DetectedTargetArray detected_targets;
    detected_targets.header = header;
    // jig_n: 拼图序号
    for (unsigned jig_n = 0; jig_n < detect_rep.size(); ++jig_n) {
        // jig_m: 拼图内序号
        for (unsigned jig_m = 0; jig_n * jigsaw_size * jigsaw_size + jig_m < squares.size() && jig_m < jigsaw_size * jigsaw_size; ++jig_m) {
            radar_interface::msg::DetectedTarget detected_target;
            detected_target.target = targets[jig_n * jigsaw_size * jigsaw_size + jig_m];
            detected_target.color = -1;
            detected_target.type = -1;
            double min_dist_sqr = 1e9;
            unsigned jig_block_size = img_size / jigsaw_size;
            for (auto& target : detect_rep[jig_n]->detected_armors) {
                // 如果目标不在拼图块内, 则跳过
                unsigned jig_x = jig_m % jigsaw_size, jig_y = jig_m / jigsaw_size;
                if (target.xywh[0] - target.xywh[2] / 2 < jig_x * jig_block_size || target.xywh[0] + target.xywh[2] / 2 > (jig_x + 1) * jig_block_size ||
                    target.xywh[1] - target.xywh[3] / 2 < jig_y * jig_block_size || target.xywh[1] + target.xywh[3] / 2 > (jig_y + 1) * jig_block_size)
                    continue;
                // 转换到以拼图块左上角为原点坐标
                auto to_jig_coord = [&](cv::Point2d pt) {
                    return cv::Point2d(pt.x - jig_x * jig_block_size, pt.y - jig_y * jig_block_size);
                };
                // 缩放, 并位移到真实尺寸
                // FIXME: 在边缘时会飞出框外
                // 好像不是雷达的问题, 是网络的问题
                auto to_real = [&](cv::Point2d pt) {
                    auto now_square = squares[jig_n * jigsaw_size * jigsaw_size + jig_m];
                    return cv::Point2d(pt.x * (double)(now_square.second.x - now_square.first.x) / (double)jig_block_size + now_square.first.x,
                        pt.y * (double)(now_square.second.y - now_square.first.y) / (double)jig_block_size + now_square.first.y);
                };
                for (auto& pt : target.pts) {
                    auto pt_cv = to_real(to_jig_coord({ pt.x, pt.y }));
                    pt.x = pt_cv.x, pt.y = pt_cv.y;
                }
                
                cv::Point2d square_centre((jig_x + 0.5) * jig_block_size, (jig_y + 0.5) * jig_block_size);
                double dist_sqr = (target.xywh[0] - square_centre.x) * (target.xywh[0] - square_centre.x) + (target.xywh[1] - square_centre.y) * (target.xywh[1] - square_centre.y);
                if (dist_sqr < min_dist_sqr) {
                    min_dist_sqr = dist_sqr;
                    detected_target.color = target.color;
                    detected_target.type = target.type;
                }
            }
            detected_targets.targets.push_back(detected_target);
        }
    }
    return detected_targets;
}

foxglove_msgs::msg::ImageMarkerArray TargetDetector::get_markers(const std::vector<Square>& squares, const DetectRepArray& detect_rep, const DetectedTargetArray& detected_targets)
{
    assert(squares.size() == detected_targets.targets.size());
    foxglove_msgs::msg::ImageMarkerArray markers;
    // 对象框 (squares)
    for (unsigned i = 0; i < squares.size(); ++i) {
            visualization_msgs::msg::ImageMarker marker;
            marker.header.frame_id = cam_frame;
            marker.header.stamp = detected_targets.header.stamp;
            marker.ns = "squares";
            marker.type = visualization_msgs::msg::ImageMarker::POLYGON;
            marker.action = visualization_msgs::msg::ImageMarker::ADD;
            marker.points.resize(4);
            marker.points[0].x = squares[i].first.x;
            marker.points[0].y = squares[i].first.y;
            marker.points[1].x = squares[i].second.x;
            marker.points[1].y = squares[i].first.y;
            marker.points[2].x = squares[i].second.x;
            marker.points[2].y = squares[i].second.y;
            marker.points[3].x = squares[i].first.x;
            marker.points[3].y = squares[i].second.y;
            marker.outline_color.a = 1.0;
            switch (detected_targets.targets[i].color) {
            case radar_interface::msg::Armor::COLOR_RED:
                marker.outline_color.r = 1.0;
                marker.outline_color.g = 0.0;
                marker.outline_color.b = 0.0;
                break;
            case radar_interface::msg::Armor::COLOR_BLUE:
                marker.outline_color.r = 0.0;
                marker.outline_color.g = 0.0;
                marker.outline_color.b = 1.0;
                break;
            case radar_interface::msg::Armor::COLOR_PURPLE:
                marker.outline_color.r = 1.0;
                marker.outline_color.g = 0.0;
                marker.outline_color.b = 1.0;
                break;
            default: // UNKNOWN
                marker.outline_color.r = 1.0;
                marker.outline_color.g = 1.0;
                marker.outline_color.b = 1.0;
                break;
            }
            marker.scale = 3;
            marker.filled = false;
            markers.markers.push_back(marker);
    }
    // 识别结果 (detect_rep)
    for (auto& rep : detect_rep) {
        for (auto& armor : rep->detected_armors) {
            visualization_msgs::msg::ImageMarker marker;
            marker.header.frame_id = cam_frame;
            marker.header.stamp = detected_targets.header.stamp;
            marker.ns = "detect_rep";
            marker.type = visualization_msgs::msg::ImageMarker::POLYGON;
            marker.action = visualization_msgs::msg::ImageMarker::ADD;
            marker.points.resize(4);
            marker.points[0].x = armor.pts[0].x;
            marker.points[0].y = armor.pts[0].y;
            marker.points[1].x = armor.pts[1].x;
            marker.points[1].y = armor.pts[1].y;
            marker.points[2].x = armor.pts[2].x;
            marker.points[2].y = armor.pts[2].y;
            marker.points[3].x = armor.pts[3].x;
            marker.points[3].y = armor.pts[3].y;
            marker.outline_color.a = 1.0;
            switch (armor.color) {
            case radar_interface::msg::Armor::COLOR_RED:
                marker.outline_color.r = 1.0;
                marker.outline_color.g = 0.0;
                marker.outline_color.b = 0.0;
                break;
            case radar_interface::msg::Armor::COLOR_BLUE:
                marker.outline_color.r = 0.0;
                marker.outline_color.g = 0.0;
                marker.outline_color.b = 1.0;
                break;
            case radar_interface::msg::Armor::COLOR_PURPLE:
                marker.outline_color.r = 1.0;
                marker.outline_color.g = 0.0;
                marker.outline_color.b = 1.0;
                break;
            default: // UNKNOWN
                marker.outline_color.r = 1.0;
                marker.outline_color.g = 1.0;
                marker.outline_color.b = 1.0;
                break;
            }
            marker.scale = 3;
            marker.filled = false;
            markers.markers.push_back(marker);
        }
    }
    return markers;
}

foxglove_msgs::msg::ImageAnnotations TargetDetector::get_annotiations(const std::vector<Square>& squares, const DetectedTargetArray& detected_targets)
{
    assert(squares.size() == detected_targets.targets.size());
    foxglove_msgs::msg::ImageAnnotations annotations;
    for (unsigned i = 0; i < squares.size(); ++i) {
        foxglove_msgs::msg::TextAnnotation annotation;
        annotation.text = "Color: ";
        switch (detected_targets.targets[i].color) {
        case radar_interface::msg::Armor::COLOR_RED:
            annotation.text += "RED";
            annotation.background_color.r = 1.0;
            annotation.background_color.g = 0.0;
            annotation.background_color.b = 0.0;
            annotation.background_color.a = 1.0;
            break;
        case radar_interface::msg::Armor::COLOR_BLUE:
            annotation.text += "BLUE";
            annotation.background_color.r = 0.0;
            annotation.background_color.g = 0.0;
            annotation.background_color.b = 1.0;
            annotation.background_color.a = 1.0;
            break;
        case radar_interface::msg::Armor::COLOR_PURPLE:
            annotation.text += "PURPLE";
            annotation.background_color.r = 1.0;
            annotation.background_color.g = 0.0;
            annotation.background_color.b = 1.0;
            annotation.background_color.a = 1.0;
            break;
        default: // UNKNOWN
            annotation.text += "UNKNOWN";
            annotation.background_color.a = 0.0;
            break;
        }
        annotation.text += " Type: ";
        switch (detected_targets.targets[i].type) {
        case radar_interface::msg::Armor::TYPE_SENTRY:
            annotation.text += "SENTRY";
            break;
        case radar_interface::msg::Armor::TYPE_HERO:
            annotation.text += "HERO";
            break;
        case radar_interface::msg::Armor::TYPE_ENGINEER:
            annotation.text += "ENGINEER";
            break;
        case radar_interface::msg::Armor::TYPE_INF_3:
            annotation.text += "INFANTRY 3";
            break;
        case radar_interface::msg::Armor::TYPE_INF_4:
            annotation.text += "INFANTRY 4";
            break;
        case radar_interface::msg::Armor::TYPE_INF_5:
            annotation.text += "INFANTRY 5";
            break;
        case radar_interface::msg::Armor::TYPE_0:
            annotation.text += "0";
            break;
        case radar_interface::msg::Armor::TYPE_BS:
            annotation.text += "Bs";
            break;
        case radar_interface::msg::Armor::TYPE_BB:
            annotation.text += "Bb";
            break;
        default: // UNKNOWN
            annotation.text += "UNKNOWN";
            break;
        }
        annotation.position.x = squares[i].first.x;
        annotation.position.y = squares[i].first.y - 15;
        annotation.text_color.r = 1.0;
        annotation.text_color.g = 1.0;
        annotation.text_color.b = 1.0;
        annotation.text_color.a = 1.0;
        annotation.font_size = 30;
        annotations.texts.push_back(annotation);
    }
    return annotations;
}
