#include "pc_aligner/aligner_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <memory>
#include <open3d/geometry/PointCloud.h>
#include <open3d/visualization/utility/ColorMap.h>
#include <radar_interface/livox_struct.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <vector>

AlignerNode::AlignerNode() : Node("pc_aligner")
{
    declare_parameter("sample_lidar", "lidar");
    declare_parameter("init_sample", 100000);
    declare_parameter("startup_manual_align", false);

    declare_parameter("mesh_sample", 10000);
    // 默认场地大小为 28 * 15(m)
    declare_parameter("crop_box.min", std::vector<double> { 0.150, 0.150, 0. });
    declare_parameter("crop_box.max", std::vector<double> { 27.850, 14.850, 1.500 });
    declare_parameter("max_corr_dist", 5.);
    declare_parameter("max_iteration", 30);

    declare_parameter("use_preselect", false);
    declare_parameter("preselect_pcd", "preselect.pcd");

    declare_parameter("vis_auto", true);
    declare_parameter("align_model", "mesh");
    declare_parameter("mesh", "bg2align.stl");
    declare_parameter("pointcloud", "bg2align.pcd");

    declare_parameter("manual_crop.min", std::vector<double> { 0, -15., -15. });
    declare_parameter("manual_crop.max", std::vector<double> { 25., 15., 15. });

    declare_parameter("tf_pub_interval_ms", 1000);
    // x, y, z, qw, qx, qy, qz
    auto init_trans = declare_parameter("init_trans", std::vector<double> { 0., 0., 0., 1., 0., 0., 0. });

    middle_tf = std::make_shared<geometry_msgs::msg::TransformStamped>();
    middle_tf->transform.translation.x = init_trans[0];
    middle_tf->transform.translation.y = init_trans[1];
    middle_tf->transform.translation.z = init_trans[2];
    middle_tf->transform.rotation.w = init_trans[3];
    middle_tf->transform.rotation.x = init_trans[4];
    middle_tf->transform.rotation.y = init_trans[5];
    middle_tf->transform.rotation.z = init_trans[6];

    tf_buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    marker_pub = create_publisher<visualization_msgs::msg::MarkerArray>("selected_point", tf2_ros::StaticBroadcasterQoS());
    prepare_meshes();

    tf_pub_timer = create_wall_timer(std::chrono::milliseconds(get_parameter("tf_pub_interval_ms").as_int()), std::bind(&AlignerNode::timer_callback, this));

    start_sample(std::bind(&AlignerNode::startup_callback, this, std::placeholders::_1));
    manual_align_service = create_service<std_srvs::srv::Empty>("manual_align", std::bind(&AlignerNode::manual_align_service_callback, this, std::placeholders::_1, std::placeholders::_2));
    auto_align_service = create_service<radar_interface::srv::AutoAlign>("auto_align", std::bind(&AlignerNode::auto_align_service_callback, this, std::placeholders::_1, std::placeholders::_2));
}

void AlignerNode::start_sample(std::function<void(std::shared_ptr<open3d::geometry::PointCloud>)> callback)
{
    if (pc_sample_context) {
        RCLCPP_ERROR(get_logger(), "pc_sample_context already created");
        return;
    }
    pc_sample_context = std::make_shared<PcSampleContext>(
        PcSampleContext {
            std::make_shared<open3d::geometry::PointCloud>(),
            static_cast<size_t>(get_parameter("init_sample").as_int()),
            create_subscription<sensor_msgs::msg::PointCloud2>(
                get_parameter("sample_lidar").as_string() + "/pc_raw", rclcpp::SystemDefaultsQoS(),
                std::bind(&AlignerNode::sample_sub_callback, this, std::placeholders::_1)),
            callback });
    pc_sample_context->recv_pc->points_.reserve(pc_sample_context->sample_size);
}

void AlignerNode::timer_callback()
{
    if (world_tf) {
        world_tf->header.stamp = now();
        tf_broadcaster->sendTransform(*world_tf);
    }
    if (middle_tf) {
        middle_tf->header.stamp = now();
        tf_broadcaster->sendTransform(*middle_tf);
    }
}

void AlignerNode::auto_align_service_callback(const std::shared_ptr<radar_interface::srv::AutoAlign_Request> request,
    std::shared_ptr<radar_interface::srv::AutoAlign_Response>)
{
    RCLCPP_INFO(get_logger(), "auto align service called");
    if (request) {
        set_parameter(rclcpp::Parameter("max_corr_dist", rclcpp::ParameterValue(request->max_corr_dist)));
        set_parameter(rclcpp::Parameter("max_iteration", rclcpp::ParameterValue(request->max_iteration)));
    }
    start_sample(std::bind(&AlignerNode::auto_align, this, std::placeholders::_1, get_middle_trans(), request->inherit_last));
}

void AlignerNode::manual_align_service_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
{
    RCLCPP_INFO(get_logger(), "manual align service called");
    start_sample(std::bind(&AlignerNode::manual_align, this, std::placeholders::_1));
}

void AlignerNode::startup_callback(std::shared_ptr<open3d::geometry::PointCloud> sample_pc)
{
    if (get_parameter("startup_manual_align").as_bool())
        manual_align(sample_pc);
    auto_align(sample_pc, get_middle_trans());
}

Eigen::Isometry3d AlignerNode::get_middle_trans()
{
    if (middle_tf)
        return tf2::transformToEigen(*middle_tf).inverse();
    while (rclcpp::ok())
    {
        try {
            return tf2::transformToEigen(tf_buffer->lookupTransform(
                "middle",
                get_parameter("sample_lidar").as_string() + "_frame",
                tf2::TimePointZero, tf2::durationFromSec(1.0)));
        } catch (const tf2::TransformException& e) {
            RCLCPP_ERROR(get_logger(), "Waiting for Middle Trans. Exception: %s", e.what());
        }
    }
    throw std::runtime_error("Failed to get Middle trans.");
}

void AlignerNode::prepare_meshes()
{
    std::string align_model = get_parameter("align_model").as_string();
    std::filesystem::path meshes_path = std::filesystem::path(ament_index_cpp::get_package_share_directory("radar_bringup")) / "resource";

    RCLCPP_INFO(get_logger(), "align model: %s", align_model.c_str());
    RCLCPP_INFO(get_logger(), "meshes path: %s", meshes_path.c_str());
    if (align_model == "mesh") {
        align_using_mesh = true;
        mesh_ori = open3d::io::CreateMeshFromFile(meshes_path / get_parameter("mesh").as_string());
        mesh_ori->ComputeVertexNormals();
        mesh_ori->ComputeTriangleNormals();
        mesh_ori->Scale(0.001, Eigen::Vector3d::Zero());
    } else if (align_model == "pointcloud") {
        align_using_mesh = false;
        pc_align = open3d::io::CreatePointCloudFromFile(meshes_path / get_parameter("pointcloud").as_string());
        // pc_align->EstimateNormals();
    } else {
        RCLCPP_WARN(get_logger(), "Unknown align_model: %s", align_model.c_str());
        align_using_mesh = true;
    }

    RCLCPP_INFO(get_logger(), "meshes prepared");
}

void AlignerNode::sample_sub_callback(const sensor_msgs::msg::PointCloud2 &msg)
{
    if (!pc_sample_context)
    {
        RCLCPP_ERROR(get_logger(), "pc_sample_context not created");
        return;
    }
    if (!radar_interface::check_lidar_msg(msg))
        return;
    const radar_interface::LivoxPointXyzrtlt *points = reinterpret_cast<const radar_interface::LivoxPointXyzrtlt *>(msg.data.data());
    for (size_t i = 0; i < msg.height * msg.width; ++i)
    {
        pc_sample_context->recv_pc->points_.emplace_back(points[i].x, points[i].y, points[i].z);
        pc_sample_context->recv_pc->colors_.emplace_back(open3d::visualization::GetGlobalColorMap()->GetColor(points[i].reflectivity / 150.0));
    }
    if (pc_sample_context->recv_pc->points_.size() >= pc_sample_context->sample_size)
    {
        pc_sample_context->recv_pc->points_.resize(pc_sample_context->sample_size);
        pc_sample_context->recv_pc->colors_.resize(pc_sample_context->sample_size);
        pc_sample_context->pc_sub.reset();
        pc_sample_context->next_step(pc_sample_context->recv_pc);
        pc_sample_context.reset();
        RCLCPP_INFO(get_logger(), "point sample complete");
    }
}
