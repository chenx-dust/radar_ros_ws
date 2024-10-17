#pragma once

#include <queue>
#include <radar_interface/livox_struct.hpp>
#include <radar_interface/msg/target_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <deque>
#include <future>
#include <memory>
#include <optional>
#include <vector>

#include <open3d/Open3D.h>

#include "VoxelGrid.h"
#include "TargetMap.h"
#include "Utility.h"


namespace pc_detector {
struct UnpackedPcMsg {
    /// @brief 解包后的点云消息
    rclcpp::Time timestamp;
    std::vector<Eigen::Vector3d> points;
};

struct LidarContext {
    using SharedPtr = std::shared_ptr<LidarContext>;
    Eigen::Isometry3d trans;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber;
    VoxelGrid voxel_grid;
};

class DetectorNode final : public rclcpp::Node {
private:
    tf2_ros::Buffer::SharedPtr tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    bool mesh_filter_mode;
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh_ori;
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh_filter;
    std::shared_ptr<open3d::geometry::PointCloud> pc_filter;

    std::vector<LidarContext::SharedPtr> lidars;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pc_publisher;
    std::deque<UnpackedPcMsg> pc_buffer;
    rclcpp::Publisher<radar_interface::msg::TargetArray>::SharedPtr target_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher;

    TargetMap target_map;

    void update_parameters();
    void pc_recv_callback(const sensor_msgs::msg::PointCloud2& msg, const LidarContext::SharedPtr l_ctx);
    void pub_solved_pc(const std::vector<Eigen::Vector3d>& points, const std::vector<int>& clustered_labels, const std::vector<int>& tracking_ids);
    void pub_targets(const rclcpp::Time& time);

    void solve();

    void prepare_lidars();
    void prepare_meshes();
    void prepare_topics();
    void prepare_voxel_grid(LidarContext& l_ctx);
    void prepare_detector();
    void prepare_kalman_filter();

public:
    DetectorNode(const rclcpp::NodeOptions& options);
};
}
