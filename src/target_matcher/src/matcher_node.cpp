#include "target_matcher/matcher_node.hpp"
#include "target_matcher/utils.hpp"
#include "target_matcher/visualization.hpp"

#include <cv_bridge/cv_bridge.h>
#include <dlib/optimization.h>
#include <unordered_set>

/// 这里不做消息的时间戳同步，是为了提高实时性

MatcherNode::MatcherNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("matcher", options)
{
    for (unsigned i = 0; i < 6; ++i)
        result.red[i].id = -1, result.blue[i].id = -1;

    declare_parameter("value_max", 10000);
    declare_parameter("value_inc", 40);
    declare_parameter("value_dec", 10);
    declare_parameter("value_fail_dec", 1);
    declare_parameter("value_match_limit", 100);
    declare_parameter("fail_dec_min", 100);
    declare_parameter("feedback_inc", 1000);
    declare_parameter("feedback_dec", 500);
    declare_parameter("pos_reinforce_inc", 10);
    declare_parameter("pos_reinforce_max", 100);
    declare_parameter("pub_timeout", 200);
    declare_parameter("vis_per_pub", 1);
    declare_parameter("pos_reinforce_timeout", 500);
    declare_parameter("uncertainty_limit", 40);
    std::vector<std::string> img_ns = declare_parameter("img_ns", std::vector<std::string> {"/radar"});

    target_sub = this->create_subscription<radar_interface::msg::TargetArray>(
        "pc_detector/targets", rclcpp::SystemDefaultsQoS(), std::bind(&MatcherNode::target_callback, this, std::placeholders::_1));
    // detected_target_sub = this->create_subscription<radar_interface::msg::DetectedTargetArray>(
    //     "img_recognizer/detected_targets", rclcpp::SystemDefaultsQoS(), std::bind(&MatcherNode::detected_target_callback, this, std::placeholders::_1));
    for (const auto& img : img_ns)
        detected_target_subs.push_back(this->create_subscription<radar_interface::msg::DetectedTargetArray>(
            img + "/img_recognizer/detected_targets", rclcpp::SystemDefaultsQoS(), std::bind(&MatcherNode::detected_target_callback, this, std::placeholders::_1)));
    feedback_sub = this->create_subscription<radar_interface::msg::FeedbackTargetArray>(
        "matcher/feedback", rclcpp::SystemDefaultsQoS(), std::bind(&MatcherNode::feedback_callback, this, std::placeholders::_1));
    match_result_pub = this->create_publisher<radar_interface::msg::MatchResult>("matcher/match_result", rclcpp::SystemDefaultsQoS());
    vis_pub = this->create_publisher<sensor_msgs::msg::Image>("matcher/visualization", rclcpp::SystemDefaultsQoS());
    vis_timer = this->create_wall_timer(std::chrono::milliseconds(get_parameter("pub_timeout").as_int()), std::bind(&MatcherNode::vis_timer_callback, this));
    pos_reinforce_timer = this->create_wall_timer(std::chrono::milliseconds(get_parameter("pos_reinforce_timeout").as_int()), std::bind(&MatcherNode::pos_reinforce_timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "target_matcher node started.");
}

void MatcherNode::target_callback(const radar_interface::msg::TargetArray::SharedPtr msg)
{
    last_targets = *msg;
    // 用于存储正在跟踪的目标
    std::unordered_set<unsigned> tracked_targets;
    for (const auto& target : msg->targets)
        tracked_targets.insert(target.id);
    // 将当前的 map 中没有被跟踪的目标删除
    for (auto it = targets_value_map.begin(); it != targets_value_map.end();) {
        // RCLCPP_INFO(this->get_logger(), "Checking result_visualizertarget %u.", it->first);
        if (tracked_targets.find(it->first) == tracked_targets.end()) {
            RCLCPP_INFO(this->get_logger(), "Target %ld removed.", it->first);
            it = targets_value_map.erase(it);
        }
        else
            ++it;
    }
    // 将当前的目标添加到 map 中
    for (const auto& target : msg->targets)
    {
        if (targets_value_map.find(target.id) == targets_value_map.end()) {
            RCLCPP_INFO(this->get_logger(), "Target %lu added.", target.id);
            targets_value_map[target.id] = { 0 };
        }
    }
    match_and_pub(msg);
}

void MatcherNode::detected_target_callback(const radar_interface::msg::DetectedTargetArray::SharedPtr msg)
{
    auto fail_dec = [&](ValueArray& value) { // 没有识别成功的
        unsigned value_num = 0;
        for (auto v : value)
            if (v > 0)
                ++value_num;
        int min_value = value_num > 1 ? 0 : get_parameter("fail_dec_min").as_int();
        for (auto& v : value)
            v = std::max(std::min(int(v), min_value), int(v - get_parameter("value_fail_dec").as_int()));
    };
    std::unordered_set<unsigned> checked_ids; // 存放已被检查过的 id
    // 遍历所有检测到的目标
    for (const auto& detected_target : msg->targets) {
        checked_ids.insert(detected_target.target.id);
        // 找到 targets_value_map 中对应的目标
        auto it = targets_value_map.find(detected_target.target.id);
        if (it != targets_value_map.end())
        {
            auto& value = it->second;
            int idx = to_idx(detected_target.color, detected_target.type);
            if (idx == -1) {    // 检测到的目标颜色或类型未知
                fail_dec(value);
                continue;
            }
            // 增强被识别到的类型，削弱其他类型
            value[idx] = std::min(value[idx] + get_parameter("value_inc").as_int(), get_parameter("value_max").as_int());
            for (int i = 0; i < 12; ++i) {
                if (i != idx)
                    value[i] = std::max(0, int(value[i] - get_parameter("value_dec").as_int()));
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Detected target %lu not found in targets_value_map.", detected_target.target.id);
        }
    }
    for (auto [id, value] : targets_value_map) {
        if (checked_ids.find(id) == checked_ids.end())
            fail_dec(value);
    }
}

void MatcherNode::feedback_callback(const radar_interface::msg::FeedbackTargetArray::SharedPtr msg)
{
    for (const auto& feedback_target : msg->targets) {
        auto it = targets_value_map.find(feedback_target.id);
        if (it != targets_value_map.end()) {
            auto& value = it->second;
            int idx = to_idx(feedback_target.color, feedback_target.type);
            if (idx == -1) {
                RCLCPP_WARN(this->get_logger(), "Feedback target %lu has unknown color or type.", feedback_target.id);
                continue;
            }
            if (feedback_target.is_right)
                value[idx] = std::min(value[idx] + get_parameter("feedback_inc").as_int(), get_parameter("value_max").as_int());
            else
                value[idx] = std::max(value[idx] - get_parameter("feedback_dec").as_int(), 0l);
        } else {
            RCLCPP_WARN(this->get_logger(), "Feedback target %lu not found in targets_value_map.", feedback_target.id);
        }
    }
}

void MatcherNode::match_and_pub(const radar_interface::msg::TargetArray::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Matching %lu targets.", msg->targets.size());

    // auto print_mat = [](const dlib::matrix<long>& mat) {
    //     std::string print_str = "Matrix: (" + std::to_string(mat.nr()) + ", " + std::to_string(mat.nc()) + ")";
    //     for (unsigned i = 0; i < mat.nr(); ++i) {
    //         print_str += "\n";
    //         for (unsigned j = 0; j < mat.nc(); ++j)
    //             print_str += std::to_string(mat(i, j)) + ", ";
    //     }
    //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", print_str.c_str());
    // };

    // 构造矩阵 (目标在 msg 中的编号, 12 个兵result种)
    // 如果目标个数大于 12, 扩展矩阵
    unsigned size = std::max(msg->targets.size(), 12ul);
    dlib::matrix<long> cost_matrix(size, size);
    // 不是方阵似乎有 bug
    int value_match_limit = get_parameter("value_match_limit").as_int();
    for (unsigned t = 0; t < msg->targets.size(); ++t)
    {
        auto& target = msg->targets[t];
        auto it = targets_value_map.find(target.id);
        if (it != targets_value_map.end())
        {
            auto& value = it->second;
            for (unsigned i = 0; i < 12; ++i)
                cost_matrix(t, i) = value[i] >= value_match_limit ? value[i] : 0;
            for (unsigned i = 12; i < size; ++i)
                cost_matrix(t, i) = 0;
        }
        else
        {
            for (unsigned i = 0; i < 12; ++i)
                cost_matrix(t, i) = 0;
        }
    }
    for (unsigned t = msg->targets.size(); t < size; ++t)
    {
        for (unsigned i = 0; i < size; ++i)
            cost_matrix(t, i) = 0;
    }
    // print_mat(cost_matrix);

    // 结果 index 是目标编号, value 是兵种编号
    auto assignment = dlib::max_cost_assignment(cost_matrix);
    // std::string print_str = "Assignment: ";
    // for (auto a : assignment)
    //     print_str += std::to_string(a) + ", ";
    // print_str += "Cost: " + std::to_string(dlib::assignment_cost(cost_matrix, assignment));
    // RCLCPP_INFO(this->get_logger(), "%s", print_str.c_str());

    for (unsigned i = 0; i < assignment.size(); ++i) {
        if (assignment[i] >= 12)
            assignment[i] = -1;
        else if (i >= msg->targets.size())
            assignment[i] = -2;
        else if (targets_value_map.find(msg->targets[i].id) == targets_value_map.end())
            assignment[i] = -3;
        else if (targets_value_map[msg->targets[i].id][assignment[i]] == 0)
            assignment[i] = -4;
    }
    // print_str = "Assignment Edited: ";
    // for (auto a : assignment)
    //     print_str += std::to_string(a) + ", ";
    // RCLCPP_INFO(this->get_logger(), "%s", print_str.c_str());

    for (unsigned i = 0; i < 6; ++i)
        result.red[i].id = -1, result.blue[i].id = -1;
    for (unsigned i = 0; i < assignment.size(); ++i) {
        if (assignment[i] >= 0) {
            auto& target = msg->targets[i];
            if (target.uncertainty >= get_parameter("uncertainty_limit").as_int())
                continue;
            if (assignment[i] < 6) {
                result.blue[assignment[i]].id = target.id;
                result.blue[assignment[i]].position = target.position;
            } else {
                result.red[assignment[i] - 6].id = target.id;
                result.red[assignment[i] - 6].position = target.position;
            }
        }
    }
}

void MatcherNode::vis_timer_callback()
{
    static int count = 0;
    if (count >= get_parameter("vis_per_pub").as_int()) {
        sensor_msgs::msg::Image::SharedPtr img;
        Visualization visualization;
        cv::Mat canvas = visualization.draw(targets_value_map, result, get_parameter("value_max").as_int());
        img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", canvas).toImageMsg();
        vis_pub->publish(*img);
        count = 0;
    }
    match_result_pub->publish(result);
    ++count;
}
