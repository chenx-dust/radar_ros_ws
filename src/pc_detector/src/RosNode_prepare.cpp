#include "RosNode.h"

#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/qos.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

using namespace pc_detector;

DetectorNode::DetectorNode(const rclcpp::NodeOptions& options)
    : Node("pc_detector", options)
{
    prepare_meshes();
    prepare_lidars();
    prepare_detector();
    prepare_kalman_filter();
    update_parameters();
    prepare_topics();
}

void DetectorNode::prepare_lidars()
{
    declare_parameter("node.lidars", std::vector<std::string> { "lidar" });

    tf_buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    auto arr = get_parameter("node.lidars").as_string_array();

    for (const auto& lidar_name : arr) {
        RCLCPP_INFO(get_logger(), "Preparing lidar: %s", lidar_name.c_str());
        std::string tf_name = lidar_name + "_frame";
        std::string topic_name = lidar_name + "/pc_raw";
        auto l_ctx = std::make_shared<LidarContext>();

        // 获取世界变换
        while (true) {
            if (!rclcpp::ok())
                throw std::runtime_error("Interrupted while waiting for transform: " + tf_name);
            try {
                l_ctx->trans = tf2::transformToEigen(
                    tf_buffer->lookupTransform("world", tf_name, tf2::TimePointZero, tf2::durationFromSec(1.0)));
                break;
            } catch (const tf2::TransformException& e) {
                RCLCPP_WARN(get_logger(), "Waiting for transform: %s. Exception: %s", tf_name.c_str(), e.what());
            }
        }

        // 订阅点云
        l_ctx->subscriber = create_subscription<sensor_msgs::msg::PointCloud2>(topic_name, rclcpp::QoS(rclcpp::KeepLast(10)),
            [this, l_ctx](const sensor_msgs::msg::PointCloud2& pc) { pc_recv_callback(pc, l_ctx); });

        prepare_voxel_grid(*l_ctx);

        lidars.push_back(l_ctx);
    }
    // RCLCPP_INFO(get_logger(), "cam2lidar tf received.");
}

void DetectorNode::prepare_meshes()
{
    declare_parameter("mesh.mesh_ori", "bg2align.stl");
    declare_parameter("mesh.mesh_filter", "bg2filter.stl");
    declare_parameter("mesh.pc_filter", "bg2filter.pcd");
    declare_parameter("mesh.init_translate", std::vector<double> { 0., 0., 0. });

    std::string filter_mode = declare_parameter("mesh.filter_mode", "mesh"); // option: mesh, pointcloud
    if (filter_mode == "mesh")
        mesh_filter_mode = true;
    else if (filter_mode == "pointcloud")
        mesh_filter_mode = false;
    else {
        RCLCPP_WARN(get_logger(), "Unknown filter_mode: %s", filter_mode.c_str());
        mesh_filter_mode = true;
    }

    auto init_translate_v = get_parameter("mesh.init_translate").as_double_array();
    Eigen::Vector3d init_translate(init_translate_v[0], init_translate_v[1], init_translate_v[2]);

    std::filesystem::path resource_path = std::filesystem::path(ament_index_cpp::get_package_share_directory("radar_bringup")) / "resource";
    RCLCPP_INFO(get_logger(), "meshes path: %s", resource_path.c_str());
    mesh_ori = open3d::io::CreateMeshFromFile(resource_path / get_parameter("mesh.mesh_ori").as_string());
    if (!mesh_ori) {
        RCLCPP_ERROR(get_logger(), "mesh_ori is null");
        return;
    }
    mesh_ori->ComputeVertexNormals();
    mesh_ori->ComputeTriangleNormals();
    mesh_ori->Scale(0.001, Eigen::Vector3d::Zero());
    mesh_ori->Translate(init_translate);

    if (mesh_filter_mode) {
        std::string mesh_filename = resource_path / get_parameter("mesh.mesh_filter").as_string();
        RCLCPP_INFO(get_logger(), "mesh_filter: %s", mesh_filename.c_str());
        mesh_filter = open3d::io::CreateMeshFromFile(mesh_filename);
        if (!mesh_filter) {
            RCLCPP_ERROR(get_logger(), "mesh_filter is null");
            return;
        }
        mesh_filter->Scale(0.001, Eigen::Vector3d::Zero());
        mesh_filter->Translate(init_translate);
    } else {
        std::string pc_filename = resource_path / get_parameter("mesh.pc_filter").as_string();
        RCLCPP_INFO(get_logger(), "pc_filter: %s", pc_filename.c_str());
        pc_filter = open3d::io::CreatePointCloudFromFile(pc_filename);
        if (!pc_filter) {
            RCLCPP_ERROR(get_logger(), "pc_filter is null");
            return;
        }
    }

    RCLCPP_INFO(get_logger(), "meshes prepared");
}

void DetectorNode::prepare_topics()
{
    declare_parameter("node.buffer_frame_size", 100);

    filtered_pc_publisher = create_publisher<sensor_msgs::msg::PointCloud2>("pc_detector/pc_filtered", rclcpp::QoS(rclcpp::KeepLast(10)));
    marker_publisher = create_publisher<visualization_msgs::msg::MarkerArray>("pc_detector/markers", rclcpp::QoS(rclcpp::KeepLast(50)));
    target_publisher = create_publisher<radar_interface::msg::TargetArray>("pc_detector/targets", rclcpp::QoS(rclcpp::KeepLast(10)));
    RCLCPP_INFO(get_logger(), "topics prepared");
}

void DetectorNode::prepare_voxel_grid(LidarContext& l_ctx)
{
    static auto voxel_size = declare_parameter("voxel_grid.voxel_size", 0.1);
    static auto dilate_size = declare_parameter("voxel_grid.dilate_size", 1);
    static auto size_min = declare_parameter("voxel_grid.size.min", std::vector<double> { 0.150, 0.150, 0. });
    static auto size_max = declare_parameter("voxel_grid.size.max", std::vector<double> { 27.850, 14.850, 1.500 });
    static auto occupy_expand = declare_parameter("voxel_grid.occupy_expand", 0.15);

    VoxelGrid& voxel_grid = l_ctx.voxel_grid;
    voxel_grid.initialize({ { size_min[0], size_min[1], size_min[2] },
        { size_max[0], size_max[1], size_max[2] },
        voxel_size });

    if (mesh_filter_mode)
        voxel_grid.occupy_by_mesh_filter(mesh_filter, occupy_expand);
    else
        voxel_grid.occupy_by_pc(pc_filter, dilate_size);
    voxel_grid.occupy_by_mesh(mesh_ori, l_ctx.trans * Eigen::Vector3d::Zero(), dilate_size);

    // open3d::visualization::DrawGeometries({ mesh_ori, std::make_shared<open3d::geometry::VoxelGrid>(voxel_grid.o3d_grid) }, "Voxel Grid", 1920, 1080);
}

Z_Map::SharedPtr TargetMap::Target::z_map;

void DetectorNode::prepare_detector()
{
    declare_parameters("target_map", std::map<std::string, int> {
        { "flag_size", 25 },
        { "last_frames", 100 },
        { "combine_limit", 15 },
        { "separate_limit", 8 },
        { "init_lost", 12 },
    });
    declare_parameters("target_map", std::map<std::string, double> {
        { "dist_thres", 20.0 },
        { "combine_dist", 8.0 },
        { "force_combine_dist", 0.1 },
        { "cc_thres", 0.05 },
        { "project_z", 0.3 }
    });
    declare_parameters("clustering", std::map<std::string, int> {
        { "normal.min_points", 8 },
        { "loose.min_points", 5 },
        { "strict.min_points", 12 },
    });
    declare_parameters("clustering", std::map<std::string, double> {
        { "z_zip", 0.5 },
        { "loose.box_expand", 0.1 },
        { "normal.eps", 0.2 },
        { "loose.eps", 0.25 },
        { "strict.eps", 0.15 },
    });

    auto min = get_parameter("voxel_grid.size.min").as_double_array();
    auto max = get_parameter("voxel_grid.size.max").as_double_array();
    TargetMap::Target::z_map = std::make_shared<Z_Map>(
        Eigen::Vector3d { min[0], min[1], min[2] },
        Eigen::Vector3d { max[0], max[1], max[2] },
        get_parameter("voxel_grid.voxel_size").as_double(),
        mesh_ori);
}

void DetectorNode::prepare_kalman_filter()
{
    declare_parameters("kalman_filter", std::map<std::string, double> {
        { "q.position", 0.00005 },
        { "q.velocity", 0.00002 },
        { "q.pos_vel", 0.00002 },
        { "r.position", 0.002 },
        { "r.velocity", 0.015 },
        { "cov_factor", 1.0 },
        { "decay_rate", 0.005 },
        { "max_velocity", 10.0 },
    });
    declare_parameter("kalman_filter.stop_p_time", 10);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pc_detector::DetectorNode)
