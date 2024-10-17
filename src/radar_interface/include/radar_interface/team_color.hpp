#pragma once

#include <std_msgs/msg/bool.hpp>

namespace radar_interface {
namespace team_color {
    enum ENUM {
        C_BLUE = 0,
        C_RED = 1,
        UNKNOWN = 2,
    };

    using msg = std_msgs::msg::Bool;
}
}
