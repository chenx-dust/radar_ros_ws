#ifndef _DETECTOR_COMMON_H
#define _DETECTOR_COMMON_H

#include <geometry_msgs/msg/point.hpp>
#include <opencv2/opencv.hpp>

geometry_msgs::msg::Point cvpoint_2_geopoint(const cv::Point2d &cvpt);

geometry_msgs::msg::Point cvpoint_2_geopoint(const cv::Point3d &cvpt);

cv::Point2f geopoint_2_cvpt2f(const geometry_msgs::msg::Point &geopt);

cv::Rect xywh_2_rect(float x, float y, float w, float h);

cv::Rect xywh_2_rect(float xywh[4]);

cv::Rect xywh_2_rect(const std::array<float, 4UL> &xywh);

cv::Rect xywh_2_rect(const std::array<double, 4UL> &xywh);
std::array<double, 4UL> rect_2_xywh(const cv::Rect &rect_);
int encoding2mat_type(const std::string &encoding);

std::string mat_type2encoding(int mat_type);

#endif