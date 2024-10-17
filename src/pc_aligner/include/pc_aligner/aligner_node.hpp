#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/empty.hpp>
#include <radar_interface/srv/auto_align.hpp>

#include <open3d/Open3D.h>

struct PcSampleContext
{
    /// @brief 采样点云的上下文
    std::shared_ptr<open3d::geometry::PointCloud> recv_pc;
    size_t sample_size;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub;
    std::function<void(std::shared_ptr<open3d::geometry::PointCloud>)> next_step;
};

class AlignerNode : public rclcpp::Node
{
private:
    bool align_using_mesh;
    tf2_ros::Buffer::SharedPtr tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    rclcpp::Service<radar_interface::srv::AutoAlign>::SharedPtr auto_align_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr manual_align_service;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
    rclcpp::TimerBase::SharedPtr tf_pub_timer;

    std::shared_ptr<open3d::geometry::TriangleMesh> mesh_ori;
    std::shared_ptr<open3d::geometry::PointCloud> pc_align;
    std::shared_ptr<PcSampleContext> pc_sample_context;
    geometry_msgs::msg::TransformStamped::SharedPtr world_tf, middle_tf;

    // @brief 拿到相机相对雷达的坐标，通过 tf2 对外发布
    // @details 相机相对雷达的坐标通过 ros parameter 获取，由于两者相对位置由物理条件决定，
    //          一般情况不会发生改变，故通过静态广播发布
    void auto_align_service_callback(const std::shared_ptr<radar_interface::srv::AutoAlign_Request> = {},
    std::shared_ptr<radar_interface::srv::AutoAlign_Response> = {});
    void manual_align_service_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request = {},
        std::shared_ptr<std_srvs::srv::Empty::Response> response = {});

    void start_sample(std::function<void(std::shared_ptr<open3d::geometry::PointCloud>)> callback);
    void sample_sub_callback(const sensor_msgs::msg::PointCloud2& msg);
    void timer_callback();
    Eigen::Isometry3d get_middle_trans();
    void prepare_meshes();
    void auto_align(std::shared_ptr<open3d::geometry::PointCloud> sample_pc, Eigen::Isometry3d pnp_trans, bool keep_last = false);

    void manual_align(std::shared_ptr<open3d::geometry::PointCloud> pc2align);
    std::shared_ptr<const open3d::geometry::PointCloud> read_preselected_pc();
    Eigen::Matrix4d manual_trans_pre(std::shared_ptr<const open3d::geometry::PointCloud> pc2align);
    Eigen::Matrix4d manual_trans_mesh(std::shared_ptr<const open3d::geometry::PointCloud> pc2align, std::shared_ptr<const open3d::geometry::PointCloud> mesh_pc);
    void pub_add_points(std::shared_ptr<const open3d::geometry::PointCloud> pc);
    void pub_del_points(unsigned size);

    void startup_callback(std::shared_ptr<open3d::geometry::PointCloud> sample_pc);

public:
    AlignerNode();
};
