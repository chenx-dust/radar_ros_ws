#pragma once

#include <opencv2/opencv.hpp>

#include "target_matcher/matcher_node.hpp"

class Visualization {
private:
    cv::Mat canvas;

    void create_canvas(unsigned left_num, unsigned right_num);
    unsigned get_height(unsigned idx, unsigned tot);
    void draw_right(const radar_interface::msg::MatchResult& result);
    void draw_left(const MatcherNode::TargetValueMap& targets_value_map, unsigned max_value);
    cv::Scalar color_map(unsigned value, unsigned max_value);
    void draw_line(unsigned left_idx, unsigned left_tot, unsigned right_idx, cv::Scalar color, int value, int thickness = 5);

public:
    cv::Mat draw(const MatcherNode::TargetValueMap& targets_value_map, const radar_interface::msg::MatchResult& result, unsigned max_value);
};