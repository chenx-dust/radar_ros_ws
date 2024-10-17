#include "target_multiplexer/target_multiplexer.hpp"

using namespace target_multiplexer;

uint16_t MultiplexerNode::get_map_id(unsigned ori_id, team_color color_)
{
    constexpr uint16_t id_map[6] = { 7, 1, 2, 3, 4, 5 };
    switch (color_) {
    // 这是由我方颜色判断地方颜色
    case team_color::C_BLUE:
        // 这是敌方红色编号
        return id_map[ori_id];
    case team_color::C_RED:
        // 这是敌方蓝色编号
        return id_map[ori_id] + 100;
    default:
        return -1;
    }
}

void MultiplexerNode::update_banned()
{
    size_t ban_time = get_parameter("ban_time").as_int();

    for (auto it = banned_targets.begin(); it != banned_targets.end();) {
        if (it->second >= ban_time)
            it = banned_targets.erase(it);
        else
            ++it->second, ++it;
    }
}

void MultiplexerNode::update_guessing()
{
    static int times = 0;
    if (times < get_parameter("guess_time").as_int()) {
        ++times;
        return;
    }
    times = 0;
    update_banned();

    if (last_hl_num < get_parameter("guess_hl_num").as_int()) {
        // 如果高亮个数不足，不猜
        guessing_id = -1;
        return;
    }
    // 排除掉有匹配的和被 ban 的
    std::set<int64_t> has_matched;
    for (unsigned i = 0; i < 6; ++i) {
        has_matched.emplace(last_match_result.red[i].id);
        has_matched.emplace(last_match_result.blue[i].id);
    }
    for (const auto& [id, _] : banned_targets)
        has_matched.emplace(id);

    double half_thres = get_parameter("half_thres").as_double();
    constexpr double HALF_SIDE = 14;

    // 判断是否在我方半场，剔除
    std::function<bool(const radar_interface::msg::Target&)> check_self_half_side;
    switch (color) {
    case radar_interface::team_color::C_BLUE:
        check_self_half_side = [=](const radar_interface::msg::Target& tg) {
            return tg.position[0] > HALF_SIDE + half_thres;
        };
        break;
    case radar_interface::team_color::C_RED:
        check_self_half_side = [=](const radar_interface::msg::Target& tg) {
            return tg.position[0] < HALF_SIDE - half_thres;
        };
        break;
    default:
        check_self_half_side = [](const radar_interface::msg::Target&) {return false;};
        break;
    }

    for (const auto& tg : last_detected.targets) {
        if (has_matched.count(tg.id))
            continue;
        if (check_self_half_side(tg))
            continue;
        guessing_id = tg.id;
        break;
    }
    RCLCPP_INFO(get_logger(), "guessing id: %ld", guessing_id);
}

void MultiplexerNode::multiplexer()
{
#define NEXT                                         \
    redo_idx = redo_idx == -1 ? send_idx : redo_idx; \
    next_iter();                                     \
    goto redo;

    static bool double_send_sign = false;
    static int send_idx = 0;
    bool stop_iter = false;

    auto next_iter = [&]() {
        /// 轮换
        if (!stop_iter) {
            ++send_idx;
            if (send_idx >= robot_num)
                send_idx = 0;
        }
    };

    int redo_idx = -1;
redo:
    if (redo_idx == send_idx) {
        // 轮了一遍没的发就退出
        RCLCPP_WARN(get_logger(), "Cannot multiplex output");
        if (full_high_light[send_idx] == FULL_HIGHLIGHT_STATUS::HIGHLIGHT)  // 避免因为满高亮而不发送数据
            full_high_light[send_idx] = FULL_HIGHLIGHT_STATUS::SKIPPED;
        else
            return;
    }

    radar_interface::msg::MapRobotData msg;
    msg.target_robot_id = get_map_id(send_idx, color);

    radar_interface::msg::MatchedTarget last_match;
    switch (color) {
    case radar_interface::team_color::C_BLUE:
        last_match = last_match_result.red[send_idx];
        break;
    case radar_interface::team_color::C_RED:
        last_match = last_match_result.blue[send_idx];
        break;
    default:
        RCLCPP_WARN(get_logger(), "Unknown color!");
        return;
    }

    if (last_match.id != -1) {
        // 如果存在已匹配目标
        msg.target_position_x = last_match.position[0];
        msg.target_position_y = last_match.position[1];
        last_pub_id[send_idx] = last_match.id;

        if (full_high_light[send_idx] == FULL_HIGHLIGHT_STATUS::SKIPPED) {// 对于满高亮的不继续发送，留出带宽
            NEXT
        }

        if (double_send_sign)           // 已经两次发送，不再发送
            double_send_sign = false;
        // 判断如果进度很小就发两次
        else if (last_mark.mark_progress[send_idx] < get_parameter("double_send_thres").as_int())
            double_send_sign = true, stop_iter = true;
    // } else if (guessing_id != -1 && !keep_guess[send_idx]) {
    } else if (guessing_id != -1) {
        // 如果正在猜测目标
        radar_interface::msg::Target target;
        // 检查该可能目标是否还在被跟踪
        bool found = false;
        for (const auto& t : last_detected.targets) {
            if (t.id == static_cast<uint64_t>(guessing_id)) {
                target = t, found = true;
                break;
            }
        }
        // 不跟了就不发了
        if (!found) {
            guessing_id = -1;
            goto redo;
        }
        msg.target_position_x = target.position[0];
        msg.target_position_y = target.position[1];
        last_pub_id[send_idx] = guessing_id;
    // } else {
    //     if (blind_guess[send_idx].size() == 0){
    //         NEXT
    //     }
    //     size_t guess_idx = 0;
    //     if (last_pub_id[send_idx] < -1)
    //         guess_idx = decode_idx(last_pub_id[send_idx]);
    //     if (!keep_guess[send_idx])
    //         ++guess_idx, keep_guess[send_idx] = true;
    //     if (guess_idx >= blind_guess[send_idx].size())
    //         guess_idx = 0;

    //     auto guess_pos = blind_guess[send_idx][guess_idx];
    //     if (color == team_color::C_RED)
    //         std::swap(guess_pos.first, guess_pos.second);
    //     msg.target_position_x = guess_pos.first;
    //     msg.target_position_y = guess_pos.second;
    //     last_pub_id[send_idx] = encode_idx(guess_idx);
    //     RCLCPP_DEBUG(get_logger(), "Blind guessing for %d: %lu", send_idx, guess_idx);
    // }
    } else {
        NEXT
    }

    RCLCPP_DEBUG(get_logger(), "Map: id: %d, x: %f, y: %f", msg.target_robot_id, msg.target_position_x, msg.target_position_y);
    map_pub->publish(msg);

    next_iter();
}

void MultiplexerNode::radar_mark_callback(const radar_interface::msg::RadarMarkData& msg)
{
    radar_interface::msg::FeedbackTargetArray fb_array;
    bool has_guess_result = false;

    team_color enemy_color;
    switch (color) {
    case radar_interface::team_color::C_BLUE:
        enemy_color = team_color::C_RED;
        break;
    case radar_interface::team_color::C_RED:
        enemy_color = team_color::C_BLUE;
        break;
    default:
        RCLCPP_WARN(get_logger(), "Unknown color!");
        return;
    }

    last_hl_num = 0;
    for (unsigned i = 0; i < 6; ++i) {
        int64_t diff = msg.mark_progress[i] - last_mark.mark_progress[i];

        if (msg.mark_progress[i] >= radar_interface::msg::RadarMarkData::HIGH_LIGHT_PROGRESS)
            ++last_hl_num;

        radar_interface::msg::FeedbackTarget fb;
        fb.id = last_pub_id[i];
        fb.type = i;
        fb.color = enemy_color;

        // keep_guess[i] = false;
        // if ((diff >= 0 && msg.mark_progress[i] != 0) || msg.mark_progress[i] == radar_interface::msg::RadarMarkData::MAX_PROGRESS) {
        //     fb.is_right = true;
        //     RCLCPP_INFO(get_logger(), "Right map: type: %d, id: %ld, progress: %d", i, last_pub_id[i], msg.mark_progress[i]);
        //     if (last_pub_id[i] == guessing_id)
        //         has_guess_result = true;
        //     else if (last_pub_id[i] < -1)   // for blind guess
        //         keep_guess[i] = true;
        // } else if (diff < 0 || msg.mark_progress[i] == 0) {
        //     fb.is_right = false;
        //     RCLCPP_DEBUG(get_logger(), "Wrong map: type: %d, id: %ld, progress: %d", i, last_pub_id[i], msg.mark_progress[i]);
        //     if (last_pub_id[i] < -1) // for blind guess
        //         keep_guess[i] = false;
        // }
        if (diff > 0) {
            fb.is_right = true;
            RCLCPP_INFO(get_logger(), "Right map: type: %d, id: %ld, progress: %d", i, last_pub_id[i], msg.mark_progress[i]);
            if (last_pub_id[i] == guessing_id)
                has_guess_result = true;
            // else if (last_pub_id[i] < -1) // for blind guess
            //     keep_guess[i] = true;
            if (last_pub_id[i] > -1)
                fb_array.targets.push_back(fb);
        }

        if (msg.mark_progress[i] == radar_interface::msg::RadarMarkData::MAX_PROGRESS)
            full_high_light[i] = FULL_HIGHLIGHT_STATUS::HIGHLIGHT;
        else
            full_high_light[i] = FULL_HIGHLIGHT_STATUS::NONE;

        // if (last_pub_id[i] > -1)
        //     fb_array.targets.push_back(fb);
    }
    if (!has_guess_result && guessing_id != -1) {
        banned_targets.emplace(guessing_id, 0);
        RCLCPP_INFO(get_logger(), "ban id: %ld", guessing_id);
    }
    if (fb_array.targets.size() > 0)
        feedback_pub->publish(fb_array);
    update_guessing();
    last_mark = msg;
}
