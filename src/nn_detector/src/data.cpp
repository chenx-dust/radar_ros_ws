#include <utils/data.h>

#if __has_include(<opencv2/opencv.hpp>)
#include <utils/common.h>
ArmorMsg Armor2Msg(const Armor& armor) {
    ArmorMsg msg;
    msg.color = armor.color;
    msg.conf = armor.conf;
    msg.size = armor.size;
    msg.type = armor.type;
    msg.xywh = rect_2_xywh(armor.rect);
    for (int i = 0; i < 5; ++i) {
        msg.pts[i] = cvpoint_2_geopoint(armor.pts[i]);
    }
    return msg;
}

Armor Msg2Armor(const ArmorMsg& armor_msg) {
    Armor armor;
    armor.color = armor_msg.color;
    armor.conf = armor_msg.conf;
    armor.size = armor_msg.size;
    armor.type = armor_msg.type;
    armor.rect = xywh_2_rect(armor_msg.xywh);
    for (int i = 0; i < 5; ++i) {
        armor.pts[i] = geopoint_2_cvpt2f(armor_msg.pts[i]);
    }
    return armor;
}

#endif

uint8_t get_rmcv_id(uint8_t id) {
    // ID=color(0~3:B R N P)*9+type(0~8:0 1 2 3 4 5 O Bs Bb)
    constexpr uint8_t offset = 9;
    switch (id) {
        case Robot_id_dji::BLUE_SENTRY:
            return 0 + 0;
        case Robot_id_dji::BLUE_HERO:
            return 0 + 1;
        case Robot_id_dji::BLUE_ENGINEER:
            return 0 + 2;
        case Robot_id_dji::BLUE_STANDARD_1:
            return 0 + 3;
        case Robot_id_dji::BLUE_STANDARD_2:
            return 0 + 4;
        case Robot_id_dji::BLUE_STANDARD_3:
            return 0 + 5;
        case Robot_id_dji::BLUE_AERIAL:
            return 0 + 6;
        case Robot_id_dji::RED_SENTRY:
            return offset + 0;
        case Robot_id_dji::RED_HERO:
            return offset + 1;
        case Robot_id_dji::RED_ENGINEER:
            return offset + 2;
        case Robot_id_dji::RED_STANDARD_1:
            return offset + 3;
        case Robot_id_dji::RED_STANDARD_2:
            return offset + 4;
        case Robot_id_dji::RED_STANDARD_3:
            return offset + 5;
        case Robot_id_dji::RED_AERIAL:
            return offset + 6;
        default:  // Unknown
            return UNKNOWN_ID;
    }
}

vision_mode cast_run_mode(uint8_t mode) {
    switch (mode) {
        case 0:
            return AUTO_AIM;
        case 1:
            return AUTO_AIM;
        case 2:
            return S_WM;
        case 3:
            return B_WM;
        case 4:
            return AUTO_AIM;
        default:
            return Unknown;
    }
}

vision_mode string2mode(const std::string& mode_str) {
    if (mode_str == "NO_AIM")
        return NO_AIM;
    else if (mode_str == "AUTO_AIM")
        return AUTO_AIM;
    else if (mode_str == "ANTI_ROT")
        return ANTI_ROT;
    else if (mode_str == "S_WM")
        return S_WM;
    else if (mode_str == "B_WM")
        return B_WM;
    else if (mode_str == "HALT")
        return HALT;
    else
        return Unknown;
}

std::string mode2string(uint8_t mode) {
    switch (mode) {
        case NO_AIM:
            return "NO_AIM";
        case AUTO_AIM:
            return "AUTO_AIM";
        case ANTI_ROT:
            return "ANTI_ROT";
        case S_WM:
            return "S_WM";
        case B_WM:
            return "B_WM";
        case HALT:
            return "HALT";
        default:
            return "UNKNOWN";
    }
}
