#include <utils/common.h>
#include <utils/data.h>

geometry_msgs::msg::Point cvpoint_2_geopoint(const cv::Point2d& cvpt) {
    geometry_msgs::msg::Point res;
    res.x = cvpt.x;
    res.y = cvpt.y;
    res.z = 0;
    return res;
}

geometry_msgs::msg::Point cvpoint_2_geopoint(const cv::Point3d& cvpt) {
    geometry_msgs::msg::Point res;
    res.x = cvpt.x;
    res.y = cvpt.y;
    res.z = cvpt.z;
    return res;
}

cv::Point2f geopoint_2_cvpt2f(const geometry_msgs::msg::Point& geopt) {
    return cv::Point2f(geopt.x, geopt.y);
}

cv::Rect xywh_2_rect(float x, float y, float w, float h) {
    return cv::Rect(x - w * 0.5, y - h * 0.5, w, h);
}

cv::Rect xywh_2_rect(float xywh[4]) { return xywh_2_rect(xywh[0], xywh[1], xywh[2], xywh[3]); }

cv::Rect xywh_2_rect(const std::array<float, 4UL>& xywh) {
    return xywh_2_rect(xywh[0], xywh[1], xywh[2], xywh[3]);
}

cv::Rect xywh_2_rect(const std::array<double, 4UL>& xywh) {
    return xywh_2_rect(xywh[0], xywh[1], xywh[2], xywh[3]);
}
std::array<double, 4UL> rect_2_xywh(const cv::Rect& rect_) {
    double now_x = rect_.x + rect_.width / 2;
    double now_y = rect_.y + rect_.height / 2;
    return {now_x, now_y, (double)rect_.width, (double)rect_.height};
}

int encoding2mat_type(const std::string& encoding) {
    if (encoding == "mono8") {
        return CV_8UC1;
    } else if (encoding == "bgr8") {
        return CV_8UC3;
    } else if (encoding == "mono16") {
        return CV_16SC1;
    } else if (encoding == "rgba8") {
        return CV_8UC4;
    }
    throw std::runtime_error("Unsupported mat type");
}

std::string mat_type2encoding(int mat_type) {
    switch (mat_type) {
        case CV_8UC1:
            return "mono8";
        case CV_8UC3:
            return "bgr8";
        case CV_16SC1:
            return "mono16";
        case CV_8UC4:
            return "rgba8";
        default:
            throw std::runtime_error("Unsupported encoding type");
    }
}