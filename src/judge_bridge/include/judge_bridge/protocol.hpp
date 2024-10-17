#pragma once

#include <atomic>
#include <cstdint>
#include <boost/endian.hpp>

#pragma pack(1)
struct frame_header_t {
    uint8_t sof;
    uint16_t data_length;
    uint8_t package_sequence;
    uint8_t CRC8;
};

// 定义于裁判系统手册
struct radar_mark_data_t {
    uint8_t mark_hero_progress;
    uint8_t mark_engineer_progress;
    uint8_t mark_standard_3_progress;
    uint8_t mark_standard_4_progress;
    uint8_t mark_standard_5_progress;
    uint8_t mark_sentry_progress;
};

struct radar_info_t {
    uint8_t radar_info;
};

struct radar_cmd_t {
    uint8_t radar_cmd;
};

struct map_robot_data_t {
    uint16_t hero_position_x;
    uint16_t hero_position_y;
    uint16_t engineer_position_x;
    uint16_t engineer_position_y;
    uint16_t infantry_3_position_x;
    uint16_t infantry_3_position_y;
    uint16_t infantry_4_position_x;
    uint16_t infantry_4_position_y;
    uint16_t infantry_5_position_x;
    uint16_t infantry_5_position_y;
    uint16_t sentry_position_x;
    uint16_t sentry_position_y;
};

// 2023
// struct map_command_t
// {
//     float target_position_x;
//     float target_position_y;
//     float target_position_z;
//     uint8_t cmd_keyboard;
//     uint16_t target_robot_id;
// };
// 2024
struct map_command_t {
    float target_position_x;
    float target_position_y;
    uint8_t cmd_keyboard;
    uint8_t target_robot_id;
    uint8_t cmd_source;
};

struct custom_info_t
{ 
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[30];
};

struct robot_status_t
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP; 
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit; 

    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1; 
    uint8_t power_management_shooter_output : 1;
};

enum INTERACTION_CMD {
    RADAR_CMD = 0x0121,
    SENTRY_DATA = 0x0200,
    MAP_KEYBOARD = 0x0202,
    UWB_DATA = 0x0203,
};

struct robot_interaction_header_t {
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
};

struct robot_interaction_dv_data_t {
    // uint16_t data_cmd_id = RADAR_CMD;
    // uint16_t sender_id;
    // uint16_t receiver_id = 0x8080;
    robot_interaction_header_t header;
    uint8_t radar_cmd;
};

struct robot_interaction_map_data_t {
    // 0x0202
    robot_interaction_header_t header;
    map_command_t map_cmd;
};

struct robot_interaction_sentry_data_t {
    robot_interaction_header_t header;
    uint8_t arr_len;
    struct robot_pos {
        uint8_t robot_id;           
        float pos_x;                // (m)
        float pos_y;                // (m)
    } custom_data[12];
};

struct robot_interaction_uwb_t {
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float standard_5_x;
    float standard_5_y;
};

struct game_status_t
{ 
    uint8_t game_type_and_progress; 
    uint16_t stage_remain_time; 
    uint64_t SyncTimeStamp; 
}; 

struct game_robot_HP_t
{ 
    uint16_t red_hero_robot_HP; 
    uint16_t red_engineer_robot_HP; 
    uint16_t red_standard_3_robot_HP; 
    uint16_t red_standard_4_robot_HP; 
    uint16_t red_standard_5_robot_HP; 
    uint16_t red_sentry_robot_HP; 
    uint16_t red_outpost_HP; 
    uint16_t red_base_HP; 
    uint16_t blue_hero_robot_HP; 
    uint16_t blue_engineer_robot_HP; 
    uint16_t blue_standard_3_robot_HP; 
    uint16_t blue_standard_4_robot_HP; 
    uint16_t blue_standard_5_robot_HP; 
    uint16_t blue_sentry_robot_HP; 
    uint16_t blue_outpost_HP; 
    uint16_t blue_base_HP; 
}; 

#pragma pack()

// 具体十六进制码参见裁判系统手册
enum CMD_ID {
    GAME_STATUS = 0x0001,
    GAME_ROBOT_HP = 0x0003,
    ROBOT_STATUS = 0x0201,
    DETECT_PROCESS = 0x020C,
    RADAR_INFO = 0x020E,
    ROBOT_MAP = 0x0305,
    MAP_COMMAND = 0x0303,
    INTERACTION_DATA = 0x0301,
    SEND_CUSTOM_INFO = 0x0308,
};

enum RADAR_ID {
    R_RED = 9,
    R_BLUE = 109,
};

// BLUE, RED
constexpr uint8_t SENTRY_ID[] = {107, 7};
constexpr uint8_t HERO_ID[] = {101, 1};
constexpr uint8_t ENGINEER_ID[] = {102, 2};
constexpr uint8_t STANDARD_1_ID[] = {103, 3};
constexpr uint8_t STANDARD_2_ID[] = {104, 4};
constexpr uint8_t STANDARD_3_ID[] = {105, 5};

constexpr uint8_t RED_ROBOT[] = {7, 1, 2, 3, 4, 5};
constexpr uint8_t BLUE_ROBOT[] = {107, 101, 102, 103, 104, 105};


// blue, red
constexpr uint16_t AERIAL_CLIENT[2] = { 0x016A, 0x0106, };
constexpr uint16_t RADAR_ID[2] = { 109, 9, };
