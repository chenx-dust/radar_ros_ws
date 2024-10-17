#include "target_matcher/visualization.hpp"

void Visualization::create_canvas(unsigned left_num, unsigned right_num)
{
    unsigned width = 1000;
    unsigned height = (std::max(left_num, right_num) + 1) * 100;
    canvas =  cv::Mat(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
}

unsigned Visualization::get_height(unsigned idx, unsigned tot)
{
    return canvas.rows / 2 + int((idx - tot / 2.0) * 100) + 50;
}

void Visualization::draw_right(const radar_interface::msg::MatchResult& result)
{
    for (int i = 0; i < 12; ++i)
    {
        cv::Scalar color = i < 6 ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255);
        cv::circle(canvas, cv::Point(700, get_height(i, 12)), 30, color, -1);
        cv::putText(canvas, std::to_string(i % 6), cv::Point(750, get_height(i, 12)), cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 0, 0), 2);
        cv::putText(canvas, "=>" + std::to_string(i < 6 ? result.blue[i].id : result.red[i - 6].id), cv::Point(800, get_height(i, 12)), cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 0, 0), 2);
       }
}

void Visualization::draw_left(const MatcherNode::TargetValueMap& targets_value_map, unsigned max_value)
{
    unsigned idx = 0;
    for (const auto& [id, value] : targets_value_map)
    {
        cv::circle(canvas, cv::Point(150, get_height(idx, targets_value_map.size())), 30, cv::Scalar(0, 0, 0), -1);
        cv::putText(canvas, std::to_string(id), cv::Point(30, get_height(idx, targets_value_map.size())), cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 0, 0), 2);
        for (int i = 0; i < 12; ++i) {
            if (value[i] == 0)
                continue;
            cv::Scalar color = color_map(value[i], max_value);
            draw_line(idx, targets_value_map.size(), i, color, value[i]);
        }
        ++idx;
    }
}

cv::Scalar Visualization::color_map(unsigned value, unsigned max_value)
{
    unsigned r = 255 * value / max_value;
    unsigned b = 255 * (max_value - value) / max_value;
    return cv::Scalar(b, 0, r);
}

void Visualization::draw_line(unsigned left_idx, unsigned left_tot, unsigned right_idx, cv::Scalar color, int value, int thickness)
{
    cv::Point pt1(200, get_height(left_idx, left_tot));
    cv::Point pt2(650, get_height(right_idx, 12));
    cv::line(canvas, pt1, pt2, color, thickness);
    cv::putText(canvas, std::to_string(value), (pt1 + 3 * pt2) / 4, cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 0, 0), 2);
}

cv::Mat Visualization::draw(const MatcherNode::TargetValueMap& targets_value_map, const radar_interface::msg::MatchResult& result, unsigned max_value)
{
    create_canvas(targets_value_map.size(), 12);
    draw_right(result);
    draw_left(targets_value_map, max_value);
    return canvas;
}
