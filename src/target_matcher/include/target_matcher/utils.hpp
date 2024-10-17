#pragma once

#include <radar_interface/msg/armor.hpp>

/// @brief 装甲板到匹配用的索引
/// @return -1: 无效装甲板 0-11: 装甲板索引
inline int to_idx(int color, int type)
{
    int rtn;
    switch (color) {
    case radar_interface::msg::Armor::COLOR_BLUE:
        rtn = 0;
        break;
    case radar_interface::msg::Armor::COLOR_RED:
        rtn = 6;
        break;
    default:
        return -1;
    }
    if (type < 0 || type > 5) {
        return -1;
    } else {
        return rtn + type;
    }
}

inline int get_color(int idx)
{
    if (idx < 6) {
        return radar_interface::msg::Armor::COLOR_BLUE;
    } else if (idx < 12) {
        return radar_interface::msg::Armor::COLOR_RED;
    } else {
        return -1;
    }
}

inline int get_type(int idx)
{
    if (idx < 0 || idx > 11) {
        return -1;
    } else {
        return idx % 6;
    }
}
