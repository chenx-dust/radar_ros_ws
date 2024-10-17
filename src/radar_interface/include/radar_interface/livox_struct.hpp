#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace radar_interface {
#pragma pack(push, 1)
typedef struct {
    float x; /**< X axis, Unit:m */
    float y; /**< Y axis, Unit:m */
    float z; /**< Z axis, Unit:m */
    float reflectivity; /**< Reflectivity   */
    uint8_t tag; /**< Livox point tag   */
    uint8_t resv; /**< Reserved   */
    double timestamp; /**< Timestamp of point*/
} LivoxPointXyzrtlt;
#pragma pack(pop)

inline bool check_lidar_msg(const sensor_msgs::msg::PointCloud2& msg)
{
    // Size check
    if (msg.point_step != sizeof(radar_interface::LivoxPointXyzrtlt)) {
        RCLCPP_ERROR(rclcpp::get_logger("check_lidar_msg"), "point cloud point_step error: %d", msg.point_step);
        return false;
    }
    if (msg.row_step != msg.point_step * msg.width) {
        RCLCPP_ERROR(rclcpp::get_logger("check_lidar_msg"), "point cloud row_step error: %d", msg.row_step);
        return false;
    }
    if (msg.data.size() != msg.row_step * msg.height) {
        RCLCPP_ERROR(rclcpp::get_logger("check_lidar_msg"), "point cloud data size error: %lu", msg.data.size());
        return false;
    }
    // Field check
    if (msg.fields.size() != 7) {
        RCLCPP_ERROR(rclcpp::get_logger("check_lidar_msg"), "point cloud fields size error: %lu", msg.fields.size());
        return false;
    }
    if (msg.fields[0].name != "x" || msg.fields[0].offset != 0 || msg.fields[0].datatype != sensor_msgs::msg::PointField::FLOAT32 || msg.fields[0].count != 1) {
        RCLCPP_ERROR(rclcpp::get_logger("check_lidar_msg"), "point cloud fields[0] error");
        return false;
    }
    if (msg.fields[1].name != "y" || msg.fields[1].offset != 4 || msg.fields[1].datatype != sensor_msgs::msg::PointField::FLOAT32 || msg.fields[1].count != 1) {
        RCLCPP_ERROR(rclcpp::get_logger("check_lidar_msg"), "point cloud fields[1] error");
        return false;
    }
    if (msg.fields[2].name != "z" || msg.fields[2].offset != 8 || msg.fields[2].datatype != sensor_msgs::msg::PointField::FLOAT32 || msg.fields[2].count != 1) {
        RCLCPP_ERROR(rclcpp::get_logger("check_lidar_msg"), "point cloud fields[2] error");
        return false;
    }
    if (msg.fields[3].name != "intensity" || msg.fields[3].offset != 12 || msg.fields[3].datatype != sensor_msgs::msg::PointField::FLOAT32 || msg.fields[3].count != 1) {
        RCLCPP_ERROR(rclcpp::get_logger("check_lidar_msg"), "point cloud fields[3] error");
        return false;
    }
    if (msg.fields[4].name != "tag" || msg.fields[4].offset != 16 || msg.fields[4].datatype != sensor_msgs::msg::PointField::UINT8 || msg.fields[4].count != 1) {
        RCLCPP_ERROR(rclcpp::get_logger("check_lidar_msg"), "point cloud fields[4] error");
        return false;
    }
    // if (msg.fields[5].name != "tag" || msg.fields[5].offset != 17 || msg.fields[5].datatype != sensor_msgs::msg::PointField::UINT8 || msg.fields[4].count != 1) {
    //     RCLCPP_ERROR(rclcpp::get_logger("check_lidar_msg"), "point cloud fields[4] error");
    //     return false;
    // }
    if (msg.fields[6].name != "timestamp" || msg.fields[6].offset != 18 || msg.fields[6].datatype != sensor_msgs::msg::PointField::FLOAT64 || msg.fields[5].count != 1) {
        RCLCPP_ERROR(rclcpp::get_logger("check_lidar_msg"), "point cloud fields[6] error");
        return false;
    }
    return true;
}
}