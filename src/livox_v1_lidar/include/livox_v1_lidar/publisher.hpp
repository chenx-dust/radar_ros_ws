#pragma once

#include <livox_v1_lidar/protocal.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <radar_interface/livox_struct.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <thread>

#include <boost/asio.hpp>

namespace livox_v1_lidar {

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;
constexpr size_t pc_msg_size = 1380;


class LidarPublisher : public rclcpp::Node {
private:
    std::string frame_id;
    boost::asio::io_context ctx;
    boost::asio::ip::address_v4 local_ip;
    boost::asio::ip::address_v4 dest_ip;
    uint16_t local_port;
    std::optional<boost::asio::ip::udp::socket> socket;
    std::atomic_bool need_reconnect = true;

    void recv_spin();
    void heartbeat_spin();

    size_t batch_dot_num;

    rclcpp::Publisher<PointCloud2>::SharedPtr pc_pub;

    rclcpp::TimerBase::SharedPtr timer;
    std::thread recv_thread, heartbeat_thread;
    PointCloud2::SharedPtr pc_msg;

    void process_type2(const protocal::data_header& header, const protocal::type2_span& data)
    {
        for (size_t i = 0; i < protocal::dot_num; ++i) {
            pc2_write({
                data[i].x.value() / 1000.0f,
                data[i].y.value() / 1000.0f,
                data[i].z.value() / 1000.0f,
                static_cast<float>(data[i].reflectivity.value()),
                data[i].tag.value(),
                0x00,
                static_cast<double>(header.timestamp.value()), // TODO: 需要验证时间戳含义
            });
        }
    }

    void pc2_init()
    {
        pc_msg = std::make_shared<PointCloud2>();
        sensor_msgs::PointCloud2Modifier modifier(*pc_msg);
        modifier.setPointCloud2Fields(7,
            "x", 1, PointField::FLOAT32,
            "y", 1, PointField::FLOAT32,
            "z", 1, PointField::FLOAT32,
            "intensity", 1, PointField::FLOAT32,
            "tag", 1, PointField::UINT8,
            "resv", 1, PointField::UINT8,
            "timestamp", 1, PointField::FLOAT64);
        pc_msg->header.frame_id.assign(frame_id);
        pc_msg->height = 1;
        pc_msg->width = batch_dot_num;
        pc_msg->row_step = pc_msg->width * pc_msg->point_step;
        pc_msg->is_bigendian = false;
        pc_msg->is_dense = true;
        pc_msg->data.resize(pc_msg->row_step);
        // pc_msg->header.stamp = now();
    }

    void pc2_write(const radar_interface::LivoxPointXyzrtlt& pt)
    {
        /// @brief 写入点云数据
        auto data = reinterpret_cast<radar_interface::LivoxPointXyzrtlt*>(pc_msg->data.data());
        static size_t it = 0;
        data[it++] = pt;
        // 满了就发布点云
        if (it == pc_msg->width) {
            pc_msg->header.stamp = now();
            pc_pub->publish(*pc_msg);
            it = 0;
        }
    }

public:
    LidarPublisher(const rclcpp::NodeOptions& options);
};
}
