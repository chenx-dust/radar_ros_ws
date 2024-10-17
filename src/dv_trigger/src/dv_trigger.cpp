#include <memory>
#include <dv_trigger/dv_trigger.hpp>



DvTriggerNode::DvTriggerNode()
    : rclcpp::Node("dv_trigger")
{
    for (int i = 0; i < 6; ++i)
        last_hp.red_robot_hp[i] = 0, last_hp.blue_robot_hp[i] = 0;

    dv_context.used_chances = 0;
    dv_context.waiting_for_check = false;

    declare_parameter("time_force_trigger", 70);
    declare_parameter("cross_bots_trigger", 2);
    declare_parameter("hp_diff_trigger", 10);
    declare_parameter("hp_diff_ticks", 3);
    declare_parameter("keyboard_trigger", 'L');
    declare_parameter("dv_check_time", 5);
    declare_parameter("dv_max", 2);
    auto ignore = declare_parameter("bot_ignore", std::vector<long> { 0, 2 });
    for (const auto& i : ignore)
        bot_ignore.insert(i);

    pub_custom_info = create_publisher<std_msgs::msg::String>("judge/custom_info", rclcpp::SystemDefaultsQoS());
    pub_radar_cmd = create_publisher<std_msgs::msg::UInt8>("judge/radar_cmd", rclcpp::SystemDefaultsQoS());
    
    sub_radar_info = create_subscription<radar_interface::msg::RadarInfo>("judge/radar_info", rclcpp::SystemDefaultsQoS(), std::bind(&DvTriggerNode::radar_info_callback, this, std::placeholders::_1));
    sub_team_color = create_subscription<radar_interface::team_color::msg>("judge/color", rclcpp::SystemDefaultsQoS(), std::bind(&DvTriggerNode::color_callback, this, std::placeholders::_1));
    sub_match_result = create_subscription<radar_interface::msg::MatchResult>("matcher/match_result", rclcpp::SystemDefaultsQoS(), std::bind(&DvTriggerNode::calc_robots_count, this, std::placeholders::_1));
    sub_time = create_subscription<std_msgs::msg::UInt16>("judge/remain_time", rclcpp::SystemDefaultsQoS(), std::bind(&DvTriggerNode::time_callback, this, std::placeholders::_1));
    sub_hp = create_subscription<radar_interface::msg::GameRobotHP>("judge/game_robot_hp", rclcpp::SystemDefaultsQoS(), std::bind(&DvTriggerNode::hp_callback, this, std::placeholders::_1));
    sub_key = create_subscription<radar_interface::msg::MapCommand>("judge/map_keyboard", rclcpp::SystemDefaultsQoS(), std::bind(&DvTriggerNode::map_keyboard_callback, this, std::placeholders::_1));
    sub_radar_mark = create_subscription<radar_interface::msg::RadarMarkData>("judge/radar_mark_data", rclcpp::SystemDefaultsQoS(), std::bind(&DvTriggerNode::radar_mark_callback, this, std::placeholders::_1));
}

bool DvTriggerNode::dv_available()
{
    return dv_context.waiting_for_check > 0 && !dv_context.is_dv_trigered && (dv_context.now_chances > 0);
}

void DvTriggerNode::trigger_dv(const std::string_view& reason)
{
    if (!dv_available()) {
        RCLCPP_WARN(get_logger(), "Trigger Failed. Trigger reason: %s", reason.data());
        return;
    }
    auto radar_cmd = std_msgs::msg::UInt8();
    radar_cmd.data = std::min(long(dv_context.used_chances + 1), get_parameter("dv_max").as_int());
    dv_context.waiting_for_check = get_parameter("dv_check_time").as_int();
    pub_radar_cmd->publish(radar_cmd);

    auto message = std_msgs::msg::String();
    message.data = "【触发】双倍易伤：";
    message.data += reason;
    pub_custom_info->publish(message);
    RCLCPP_INFO(get_logger(), "Trigger double vulnerability: %d / %d, reason: %s", dv_context.used_chances, dv_context.now_chances, reason.data());
}

void DvTriggerNode::radar_info_callback(const radar_interface::msg::RadarInfo& info){
    dv_context.now_chances = info.dv_chances;
    dv_context.is_dv_trigered = info.dv_triggered;

    if (dv_context.waiting_for_check > 0) {
        if (dv_context.is_dv_trigered) {
            dv_context.waiting_for_check = 0;
            ++dv_context.used_chances;
            RCLCPP_INFO(get_logger(), "Double vulnerability successfully triggered");
        }
        else
            --dv_context.waiting_for_check;
    }
    if (dv_context.used_chances > info.dv_chances) {
        dv_context.used_chances = info.dv_chances;
        RCLCPP_WARN(get_logger(), "Double vulnerability has wrong response");
    }
    static bool already_pub = false;
    if (!dv_context.is_dv_trigered && dv_context.now_chances > dv_context.used_chances) {
        if (already_pub)
            return;
        already_pub = true;
        auto message = std_msgs::msg::String();
        message.data = "【可用】双倍易伤";
        pub_custom_info->publish(message);
    } else
        already_pub = false;
    check_trigger();
}

void DvTriggerNode::color_callback(const radar_interface::team_color::msg& color_)
{
    radar_interface::team_color::ENUM last_color = color;
    color = static_cast<radar_interface::team_color::ENUM>(color_.data);
    if (last_color != color)
        switch (color) {
        case radar_interface::team_color::C_RED:
            RCLCPP_INFO(get_logger(), "WE ARE <<<RED>>>");
            break;
        case radar_interface::team_color::C_BLUE:
            RCLCPP_INFO(get_logger(), "WE ARE <<<BLUE>>>");
            break;
        default:
            RCLCPP_WARN(get_logger(), "Unknow the radar id");
            break;
        }
}

void DvTriggerNode::time_callback(const std_msgs::msg::UInt16& time)
{
    static uint16_t last_time = 0;
    if (time.data > last_time || time.data > 400)
        dv_context.used_chances = 0;    // 重置 trick
    last_time = time.data;
    // RCLCPP_INFO(get_logger(), "Remain Time: %u", time.data);
    if (time.data < get_parameter("time_force_trigger").as_int()) {
        // RCLCPP_INFO(get_logger(), "Double vulnerability triggered: Time is up (%u s)", time.data);
        trigger_dv("比赛即将结束");
    }
}

void DvTriggerNode::hp_callback(const radar_interface::msg::GameRobotHP& hp)
{
    last_hp = hp;
    check_battle(); // 3 ticks/s, last_hp = 0t, hp_queue.top = 3t, -> 1s
    hp_queue.push(hp);
    auto size = get_parameter("hp_diff_ticks").as_int();
    while (hp_queue.size() > static_cast<uint64_t>(size))
        hp_queue.pop();
}

void DvTriggerNode::radar_mark_callback(const radar_interface::msg::RadarMarkData& mark)
{
    is_highlight = false;
    for (const auto& progress : mark.mark_progress)
        if (progress > 0) {
            is_highlight = true;
            return;
        }
}

void DvTriggerNode::map_keyboard_callback(const radar_interface::msg::MapCommand& key)
{
    if (key.cmd_keyboard == get_parameter("keyboard_trigger").as_int())
        trigger_dv("手动触发");
}

void DvTriggerNode::check_battle()
{
    auto hp_diff = get_parameter("hp_diff_trigger").as_int();
    in_battle = false;
    if (hp_queue.empty())
        return;
    for (int i = 0; i < 6; ++i) {
        if (bot_ignore.count(i))
            continue;
        if (last_hp.red_robot_hp[i] - hp_queue.front().red_robot_hp[i] > hp_diff)
            in_battle = true;
        if (last_hp.blue_robot_hp[i] - hp_queue.front().blue_robot_hp[i] > hp_diff)
            in_battle = true;
        if (in_battle)
            break;
    }
    if (in_battle)
        RCLCPP_INFO(get_logger(), "IN BATTLE");
}

void DvTriggerNode::calc_robots_count(const radar_interface::msg::MatchResult& result)
{
    constexpr float HALF_SIDE = 14.;
    // int red_bot_in_blue_ = 0, blue_bot_in_red_ = 0;
    for (unsigned i = 0; i < 6; ++i) {
        if (result.red[i].id == -1)
            continue;
        if (last_hp.red_robot_hp[i] == 0)
            red_bot_in_blue[i] = false;
        else
            red_bot_in_blue[i] = result.red[i].position[0] > HALF_SIDE;
        // if (result.red[i].position[0] > HALF_SIDE)
        //     ++red_bot_in_blue_;
    }
    for (unsigned i = 0; i < 6; ++i) {
        if (result.blue[i].id == -1)
            continue;
        if (last_hp.blue_robot_hp[i] == 0)
            blue_bot_in_red[i] = false;
        else
            blue_bot_in_red[i] = result.blue[i].position[0] <= HALF_SIDE;
        // if (result.blue[i].position[0] <= HALF_SIDE)
        //     ++blue_bot_in_red_;
    }
    // RCLCPP_INFO(get_logger(), "red_in_blue: %d blue_in_red: %d", red_bot_in_blue_, blue_bot_in_red_);
}

void DvTriggerNode::check_trigger()
{
    if (!in_battle)
        return;
    if (!is_highlight)
        return;

    unsigned cross_bots = get_parameter("cross_bots_trigger").as_int();
    if (color == radar_interface::team_color::C_BLUE) {
        unsigned tot_blue_in_red = 0, tot_red_in_blue = 0;
        for (bool i : blue_bot_in_red)
            if (i) ++tot_blue_in_red;
        for (bool i : red_bot_in_blue)
            if (i) ++tot_red_in_blue;
        if (tot_blue_in_red >= cross_bots)
            trigger_dv("我方前压");
        else if (tot_red_in_blue >= cross_bots)
            trigger_dv("对方前压");
    } else if (color == radar_interface::team_color::C_RED) {
        unsigned tot_blue_in_red = 0, tot_red_in_blue = 0;
        for (bool i : blue_bot_in_red)
            if (i)
                ++tot_blue_in_red;
        for (bool i : red_bot_in_blue)
            if (i)
                ++tot_red_in_blue;
        if (tot_red_in_blue >= cross_bots)
            trigger_dv("我方前压");
        else if (tot_blue_in_red >= cross_bots)
            trigger_dv("对方前压");
    }
}
