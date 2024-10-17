#include "judge_bridge/judge_bridge.hpp"
#include "judge_bridge/protocol.hpp"
#include "judge_bridge/decode.hpp"
#include "judge_bridge/serial.hpp"
#include <boost/locale.hpp>
#include <boost/locale/encoding.hpp>
#include <boost/locale/encoding_errors.hpp>
#include <boost/locale/encoding_utf.hpp>
#include <codecvt>
#include <locale>
#include <cstdint>
#include <functional>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>


void JudgeBridgeNode::filter_handler(JudgeSerial::JudgePair message){
    switch (message.first) {
    case CMD_ID::DETECT_PROCESS:{
        auto mark_data = reinterpret_cast<radar_mark_data_t*>(message.second.data());
        pub_radar_mark_data->publish(decode_radar_mark_data(*mark_data));
        }
        break;
    case CMD_ID::RADAR_INFO:{
        auto radar_info = reinterpret_cast<radar_info_t*>(message.second.data());
        auto radar_info_msg = decode_radar_info(*radar_info);
        pub_radar_info->publish(radar_info_msg);
        }
        break;
    case CMD_ID::ROBOT_STATUS:
        robot_status_callback(*reinterpret_cast<robot_status_t*>(message.second.data()));
        break;
    case CMD_ID::MAP_COMMAND:
        map_command_callback(*reinterpret_cast<map_command_t*>(message.second.data()));
        break;
    case CMD_ID::GAME_STATUS:
        game_status_callback(*reinterpret_cast<game_status_t*>(message.second.data()));
        break;
    case CMD_ID::GAME_ROBOT_HP:
        game_robot_hp_callback(*reinterpret_cast<game_robot_HP_t*>(message.second.data()));
        break;
    case CMD_ID::INTERACTION_DATA:
        interaction_data_callback(message.second);
        break;
    default:
        RCLCPP_DEBUG(rclcpp::get_logger("command"), "redundant commands");           
        return;
    }
}

void JudgeBridgeNode::send_radar_cmd(const std_msgs::msg::UInt8 &radar_cmd)
{
    robot_interaction_dv_data_t dv_data;
    dv_data.header.data_cmd_id = RADAR_CMD;
    dv_data.header.sender_id = RADAR_ID[color];
    dv_data.header.receiver_id = 0x8080;
    dv_data.radar_cmd = radar_cmd.data;
    judge_serial->write(CMD_ID::INTERACTION_DATA, reinterpret_cast<uint8_t*>(&dv_data), sizeof(dv_data));
    RCLCPP_INFO(get_logger(), "DV: %d", radar_cmd.data);
}

void JudgeBridgeNode::send_custom_info(const std::string& str)
{
    if (color == team_color::UNKNOWN) {
        RCLCPP_WARN(get_logger(), "Unkown Color!");
        return;
    }
    custom_info_t custom_info {};
    custom_info.sender_id = RADAR_ID[color];
    custom_info.receiver_id = AERIAL_CLIENT[color];
    std::u16string u16_info = boost::locale::conv::utf_to_utf<char16_t>(str);
    if (u16_info.size() > 15)
        RCLCPP_WARN(get_logger(), "Custom info too long!");
    u16_info.resize(15);
    
    for (unsigned i = 0; i < 15; ++i) {
        char16_t ch = u16_info[i];
        custom_info.user_data[2 * i] = static_cast<uint8_t>(ch & 0xFF);
        custom_info.user_data[2 * i + 1] = static_cast<uint8_t>((ch >> 8) & 0xFF);
    }
    judge_serial->write(CMD_ID::SEND_CUSTOM_INFO, reinterpret_cast<uint8_t*>(&custom_info), sizeof(custom_info));
}

void JudgeBridgeNode::map_command_callback(const map_command_t& cmd)
{
    RCLCPP_INFO(get_logger(), "keyboard: %#x "
                              "x: %f, y: %f, id: %#x",
        cmd.cmd_keyboard, cmd.target_position_x, cmd.target_position_y, cmd.target_robot_id);
}

void JudgeBridgeNode::robot_status_callback(const robot_status_t& robot_data)
{
    team_color::msg color_msg;
    uint8_t radar_id = robot_data.robot_id;
    switch (radar_id) {
    case RADAR_ID::R_RED: {
        if (color != team_color::C_RED)
            RCLCPP_INFO(get_logger(), "WE ARE <<<RED>>>");
        color = team_color::C_RED;
    } break;
    case RADAR_ID::R_BLUE: {
        if (color != team_color::C_BLUE)
            RCLCPP_INFO(get_logger(), "WE ARE <<<BLUE>>>");
        color = team_color::C_BLUE;
    } break;
    default:
        RCLCPP_WARN(get_logger(), "Unknow the radar id");
        return;
    }
    color_msg.data = static_cast<bool>(color);
    pub_color->publish(color_msg);
}

void JudgeBridgeNode::game_status_callback(const game_status_t& status)
{
    RCLCPP_INFO(get_logger(), "game_status: game_type_and_progress: %d, remain_time: %d", status.game_type_and_progress, status.stage_remain_time);
    if ((status.game_type_and_progress >> 4) == 4) {
        RCLCPP_INFO(get_logger(), "game in battle");
        auto remain_time_msg = std_msgs::msg::UInt16();
        remain_time_msg.data = status.stage_remain_time;
        pub_remain_time->publish(remain_time_msg);
    }
}

void JudgeBridgeNode::game_robot_hp_callback(const game_robot_HP_t& hp)
{
    msg::GameRobotHP msg;
    msg.red_robot_hp = {
        hp.red_sentry_robot_HP,
        hp.red_hero_robot_HP,
        hp.red_engineer_robot_HP,
        hp.red_standard_3_robot_HP,
        hp.red_standard_4_robot_HP,
        hp.red_standard_5_robot_HP,
    };
    msg.red_base = hp.red_base_HP;
    msg.red_outpost = hp.red_outpost_HP;
    msg.blue_robot_hp = {
        hp.blue_sentry_robot_HP,
        hp.blue_hero_robot_HP,
        hp.blue_engineer_robot_HP,
        hp.blue_standard_3_robot_HP,
        hp.blue_standard_4_robot_HP,
        hp.blue_standard_5_robot_HP,
    };
    msg.blue_base = hp.blue_base_HP;
    msg.blue_outpost = hp.blue_outpost_HP;
    pub_game_robot_hp->publish(msg);
}

void JudgeBridgeNode::interaction_data_callback(const std::vector<uint8_t>& data)
{
    auto header = reinterpret_cast<const robot_interaction_header_t*>(data.data());
    if (header->data_cmd_id == INTERACTION_CMD::MAP_KEYBOARD) {
        auto map_interaction = reinterpret_cast<const robot_interaction_map_data_t*>(data.data());
        radar_interface::msg::MapCommand msg;
        msg.target_position_x = map_interaction->map_cmd.target_position_x;
        msg.target_position_y = map_interaction->map_cmd.target_position_y;
        msg.target_robot_id = map_interaction->map_cmd.target_robot_id;
        msg.cmd_keyboard = map_interaction->map_cmd.cmd_keyboard;
        msg.cmd_source = map_interaction->map_cmd.cmd_source;
        pub_map_keyboard->publish(msg);
        RCLCPP_INFO(get_logger(), "Transferred Key: %d", map_interaction->map_cmd.cmd_keyboard);
    } else if (header->data_cmd_id == INTERACTION_CMD::UWB_DATA) {
        auto uwb = reinterpret_cast<const robot_interaction_uwb_t*>(data.data());
        radar_interface::msg::UwbData msg;
        msg.hero_x = uwb->hero_x;
        msg.hero_y = uwb->hero_y;
        msg.engineer_x = uwb->engineer_x;
        msg.engineer_y = uwb->engineer_y;
        msg.standard_3_x = uwb->standard_3_x;
        msg.standard_3_y = uwb->standard_3_y;
        msg.standard_4_x = uwb->standard_4_x;
        msg.standard_4_y = uwb->standard_4_y;
        msg.standard_5_x = uwb->standard_5_x;
        msg.standard_5_y = uwb->standard_5_y;
        pub_uwb_data->publish(msg);
        RCLCPP_INFO(get_logger(), "UWB Received");
    }
}

void JudgeBridgeNode::send_sentry_data(const radar_interface::msg::MatchResult& topic_message)
{
    if (color == team_color::UNKNOWN) {
        RCLCPP_WARN(get_logger(), "Unkown Color!");
        return;
    }
    robot_interaction_sentry_data_t interaction_data;
    interaction_data.header.data_cmd_id = INTERACTION_CMD::SENTRY_DATA;

    switch (color) {
    case team_color::C_RED:
        interaction_data.header.sender_id = RADAR_ID::R_RED;
        interaction_data.header.receiver_id = SENTRY_ID[team_color::C_RED];
        break;
    case team_color::C_BLUE:
        interaction_data.header.sender_id = RADAR_ID::R_BLUE;
        interaction_data.header.receiver_id = SENTRY_ID[team_color::C_BLUE];
        break;
    default:
        RCLCPP_WARN_ONCE(get_logger(), "Unknow the radar color");
        return;
    }
    uint8_t len = 0;

    for (uint8_t index = 0; index < topic_message.red.size(); index++) {
        const auto& red = topic_message.red[index];
        if (red.id == -1)
            continue;
        interaction_data.custom_data[len].robot_id = RED_ROBOT[index];
        interaction_data.custom_data[len].pos_x = red.position[0];
        interaction_data.custom_data[len].pos_y = red.position[1];
        ++len;
    }

    for (uint8_t index = 0; index < topic_message.blue.size(); index++) {
        const auto& blue = topic_message.red[index];
        if (blue.id == -1)
            continue;
        interaction_data.custom_data[len].robot_id = BLUE_ROBOT[index];
        interaction_data.custom_data[len].pos_x = blue.position[0];
        interaction_data.custom_data[len].pos_y = blue.position[1];
        ++len;
    }

    interaction_data.arr_len = len;
    judge_serial->write(CMD_ID::INTERACTION_DATA, reinterpret_cast<uint8_t*>(&interaction_data),
        sizeof(robot_interaction_sentry_data_t) - (12 - len) * sizeof(robot_interaction_sentry_data_t::robot_pos));
}

void JudgeBridgeNode::send_map_robot_data(const radar_interface::msg::MatchResult& msg)
{
    map_robot_data_t map_robot_data;
    constexpr uint16_t default_red_x = 210, default_red_y = 110;
    constexpr uint16_t default_blue_x = 2800 - 210, default_blue_y = 1500 - 110;
    switch (color) {
    case team_color::C_RED:
        map_robot_data.sentry_position_x = msg.blue[0].id != -1 ? msg.blue[0].position[0] * 100 : default_blue_x;
        map_robot_data.sentry_position_y = msg.blue[0].id != -1 ? msg.blue[0].position[1] * 100 : default_blue_y;
        map_robot_data.hero_position_x = msg.blue[1].id != -1 ? msg.blue[1].position[0] * 100 : default_blue_x;
        map_robot_data.hero_position_y = msg.blue[1].id != -1 ? msg.blue[1].position[1] * 100 : default_blue_y;
        map_robot_data.engineer_position_x = msg.blue[2].id != -1 ? msg.blue[2].position[0] * 100 : default_blue_x;
        map_robot_data.engineer_position_y = msg.blue[2].id != -1 ? msg.blue[2].position[1] * 100 : default_blue_y;
        map_robot_data.infantry_3_position_x = msg.blue[3].id != -1 ? msg.blue[3].position[0] * 100 : default_blue_x;
        map_robot_data.infantry_3_position_y = msg.blue[3].id != -1 ? msg.blue[3].position[1] * 100 : default_blue_y;
        map_robot_data.infantry_4_position_x = msg.blue[4].id != -1 ? msg.blue[4].position[0] * 100 : default_blue_x;
        map_robot_data.infantry_4_position_y = msg.blue[4].id != -1 ? msg.blue[4].position[1] * 100 : default_blue_y;
        map_robot_data.infantry_5_position_x = msg.blue[5].id != -1 ? msg.blue[5].position[0] * 100 : default_blue_x;
        map_robot_data.infantry_5_position_y = msg.blue[5].id != -1 ? msg.blue[5].position[1] * 100 : default_blue_y;
        break;
    case team_color::C_BLUE:
        map_robot_data.sentry_position_x = msg.red[0].id != -1 ? msg.red[0].position[0] * 100 : default_red_x;
        map_robot_data.sentry_position_y = msg.red[0].id != -1 ? msg.red[0].position[1] * 100 : default_red_y;
        map_robot_data.hero_position_x = msg.red[1].id != -1 ? msg.red[1].position[0] * 100 : default_red_x;
        map_robot_data.hero_position_y = msg.red[1].id != -1 ? msg.red[1].position[1] * 100 : default_red_y;
        map_robot_data.engineer_position_x = msg.red[2].id != -1 ? msg.red[2].position[0] * 100 : default_red_x;
        map_robot_data.engineer_position_y = msg.red[2].id != -1 ? msg.red[2].position[1] * 100 : default_red_y;
        map_robot_data.infantry_3_position_x = msg.red[3].id != -1 ? msg.red[3].position[0] * 100 : default_red_x;
        map_robot_data.infantry_3_position_y = msg.red[3].id != -1 ? msg.red[3].position[1] * 100 : default_red_y;
        map_robot_data.infantry_4_position_x = msg.red[4].id != -1 ? msg.red[4].position[0] * 100 : default_red_x;
        map_robot_data.infantry_4_position_y = msg.red[4].id != -1 ? msg.red[4].position[1] * 100 : default_red_y;
        map_robot_data.infantry_5_position_x = msg.red[5].id != -1 ? msg.red[5].position[0] * 100 : default_red_x;
        map_robot_data.infantry_5_position_y = msg.red[5].id != -1 ? msg.red[5].position[1] * 100 : default_red_y;
        break;
    default:
        return;
    }
    judge_serial->write(CMD_ID::ROBOT_MAP, reinterpret_cast<uint8_t*>(&map_robot_data), sizeof(map_robot_data));
}

void JudgeBridgeNode::init_serial()
{
    if (judge_serial)
        judge_serial.reset();
    while (!judge_serial) {
        std::string serial_port = get_parameter("serial_port").as_string();
        bool enable_recorder = get_parameter("enable_recorder").as_bool();
        try {
            judge_serial = std::make_unique<JudgeSerial>(serial_port, enable_recorder);
        } catch (boost::system::system_error& e) {
            RCLCPP_WARN(get_logger(), "Connect to Serial %s failed, e.what(): %s", serial_port.c_str(), e.what());
            if (!rclcpp::ok())
                throw e;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
}

JudgeBridgeNode::JudgeBridgeNode()
    : rclcpp::Node("judge_bridge")
{
    declare_parameter("serial_port", "/dev/ttyUSB0");
    declare_parameter("enable_recorder", false);
    init_serial();

    pub_radar_mark_data = create_publisher<radar_interface::msg::RadarMarkData>("judge/radar_mark_data", rclcpp::SystemDefaultsQoS());
    pub_radar_info = create_publisher<radar_interface::msg::RadarInfo>("judge/radar_info", rclcpp::SystemDefaultsQoS());
    pub_color = create_publisher<radar_interface::team_color::msg>("judge/color", rclcpp::SystemDefaultsQoS());
    pub_remain_time = create_publisher<std_msgs::msg::UInt16>("judge/remain_time", rclcpp::SystemDefaultsQoS());
    pub_game_robot_hp = create_publisher<radar_interface::msg::GameRobotHP>("judge/game_robot_hp", rclcpp::SystemDefaultsQoS());
    pub_map_keyboard = create_publisher<radar_interface::msg::MapCommand>("judge/map_keyboard", rclcpp::SystemDefaultsQoS());
    pub_uwb_data = create_publisher<radar_interface::msg::UwbData>("judge/uwb_data", rclcpp::SystemDefaultsQoS());

    sub_radar_cmd = create_subscription<std_msgs::msg::UInt8>("judge/radar_cmd", rclcpp::SystemDefaultsQoS(), std::bind(&JudgeBridgeNode::send_radar_cmd, this, std::placeholders::_1));
    sub_match_result = create_subscription<radar_interface::msg::MatchResult>("matcher/match_result", rclcpp::SystemDefaultsQoS(), std::bind(&JudgeBridgeNode::send_sentry_data, this, std::placeholders::_1));
    sub_map_robot_data = create_subscription<radar_interface::msg::MatchResult>("matcher/match_result", rclcpp::SystemDefaultsQoS(), std::bind(&JudgeBridgeNode::send_map_robot_data, this, std::placeholders::_1));
    sub_custom_info = create_subscription<std_msgs::msg::String>("judge/custom_info", rclcpp::SystemDefaultsQoS(),
        [this](const std_msgs::msg::String& msg) {
            RCLCPP_INFO(get_logger(), "Custom Info: %s", msg.data.c_str());
            send_custom_info(msg.data);
        });

    read_thread = std::thread([&]() {
        while (rclcpp::ok()) {
            try {
                if (!judge_serial)
                    throw std::runtime_error("judge_serial not exists");
                auto [id, data] = judge_serial->read();
                RCLCPP_DEBUG(get_logger(), "Received msg: %#x", id);
                filter_handler({ id, data });
            } catch (std::runtime_error& e) {
                RCLCPP_WARN(get_logger(), "Error: %s", e.what());
                init_serial();
            }
        }
    });
}
