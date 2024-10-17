#pragma once

#include "judge_bridge/protocol.hpp"

#include <radar_interface/msg/detail/map_command__struct.hpp>
#include <radar_interface/msg/detail/radar_info__struct.hpp>
#include <radar_interface/msg/detail/radar_mark_data__struct.hpp>
#include <radar_interface/msg/radar_info.hpp>
#include <radar_interface/msg/radar_mark_data.hpp>
#include <radar_interface/msg/map_robot_data.hpp>
#include <radar_interface/msg/map_command.hpp>


inline radar_interface::msg::RadarInfo decode_radar_info(const radar_info_t& ori) {
    radar_interface::msg::RadarInfo rtn;
    rtn.dv_chances = ori.radar_info & 0x3;
    rtn.dv_triggered = (ori.radar_info >> 2) & 1;
    return rtn;
}

inline radar_interface::msg::RadarMarkData decode_radar_mark_data(const radar_mark_data_t& ori){
    radar_interface::msg::RadarMarkData rtn;
    rtn.mark_progress = {
        ori.mark_sentry_progress,
        ori.mark_hero_progress,
        ori.mark_engineer_progress,
        ori.mark_standard_3_progress,
        ori.mark_standard_4_progress,
        ori.mark_standard_5_progress,
    };
    return rtn;
}

// inline radar_interface::msg::MapRobotData decode_map_robot_data(const map_robot_data_t& ori){
//     radar_interface::msg::MapRobotData rtn;
//     rtn.target_robot_id = ori.target_robot_id;
//     rtn.target_position_x = ori.target_position_x;
//     rtn.target_position_y = ori.target_position_y;
//     return rtn;
// }
