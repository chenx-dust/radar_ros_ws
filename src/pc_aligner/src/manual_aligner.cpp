#include "pc_aligner/aligner_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <open3d/Open3D.h>
#include <tf2_eigen/tf2_eigen.hpp>

std::vector<size_t> select_points(std::shared_ptr<const open3d::geometry::PointCloud> pcd)
{
    RCLCPP_INFO(rclcpp::get_logger("pick_point"), "Select points...");
    RCLCPP_INFO(rclcpp::get_logger("pick_point"), "  Use [shift + left click] to pick points.");
    RCLCPP_INFO(rclcpp::get_logger("pick_point"), "  Use [shift + right click] to undo point picking.");
    RCLCPP_INFO(rclcpp::get_logger("pick_point"), "  After picking points, press 'Q' to close the window.");

    open3d::visualization::VisualizerWithEditing vis_;
    vis_.CreateVisualizerWindow("Select Points", 1920, 1080);
    vis_.AddGeometry(pcd);
    vis_.Run();
    vis_.DestroyVisualizerWindow();
    return vis_.GetPickedPoints();
}

std::shared_ptr<const open3d::geometry::PointCloud> AlignerNode::read_preselected_pc()
{
    std::filesystem::path pc_path = std::filesystem::path(ament_index_cpp::get_package_share_directory("radar_bringup")) / "resource" / get_parameter("preselect_pcd").as_string();
    auto pc2align = open3d::io::CreatePointCloudFromFile(pc_path.string());
    return pc2align;
}

/// @brief 利用预选择的点进行手动配准获取初始变换矩阵
Eigen::Matrix4d AlignerNode::manual_trans_pre(std::shared_ptr<const open3d::geometry::PointCloud> pc2align)
{
    auto preselected = read_preselected_pc();
    pub_add_points(preselected);
    auto picked_pc = select_points(pc2align);
    pub_del_points(preselected->points_.size());
    if (picked_pc.size() < 3)
        throw std::runtime_error("Too few points selected.");
    if (preselected->points_.size() < 3)
        throw std::runtime_error("Too few preselected points.");
    if (picked_pc.size() != preselected->points_.size())
        throw std::runtime_error("Selected points number not match.");
    std::vector<Eigen::Vector2i> correspondences;
    for (size_t i = 0; i < picked_pc.size(); ++i)
        correspondences.emplace_back(picked_pc[i], i);
    open3d::pipelines::registration::TransformationEstimationPointToPoint pointToPoint;
    return pointToPoint.ComputeTransformation(*pc2align, *preselected, correspondences);
}

/// @brief 完全手动配准获取初始变换矩阵
Eigen::Matrix4d AlignerNode::manual_trans_mesh(std::shared_ptr<const open3d::geometry::PointCloud> pc2align, std::shared_ptr<const open3d::geometry::PointCloud> mesh_pc)
{
    auto picked_mesh = select_points(mesh_pc);
    auto picked_pc = select_points(pc2align);
    if (picked_pc.size() < 3 || picked_mesh.size() < 3)
        throw std::runtime_error("Too few points selected.");
    if (picked_pc.size() != picked_mesh.size())
        throw std::runtime_error("Selected points number not match.");
    std::vector<Eigen::Vector2i> correspondences;
    for (size_t i = 0; i < picked_pc.size(); ++i)
        correspondences.emplace_back(picked_pc[i], picked_mesh[i]);
    open3d::pipelines::registration::TransformationEstimationPointToPoint pointToPoint;
    return pointToPoint.ComputeTransformation(*pc2align, *mesh_pc, correspondences);
}

void AlignerNode::manual_align(std::shared_ptr<open3d::geometry::PointCloud> pc2align_)
{
    RCLCPP_INFO(get_logger(), "Manual align...");

    auto min = get_parameter("manual_crop.min").as_double_array();
    auto max = get_parameter("manual_crop.max").as_double_array();

    open3d::geometry::AxisAlignedBoundingBox aabb = {
        { min[0], min[1], min[2] },
        { max[0], max[1], max[2] },
    };
    auto pc2align = pc2align_->Crop(aabb);
    // pc2align->EstimateNormals();

    Eigen::Matrix4d trans;

    retry:
    try {
        if (get_parameter("use_preselect").as_bool())
            trans = manual_trans_pre(pc2align);
        else {
            std::shared_ptr<open3d::geometry::PointCloud> model_pc;
            if (align_using_mesh) {
                model_pc = mesh_ori->SamplePointsUniformly(get_parameter("mesh_sample").as_int());
                model_pc->colors_.resize(model_pc->points_.size());
                for (size_t i = 0; i < model_pc->points_.size(); ++i) {
                    if (model_pc->points_[i](0) < 14)
                        model_pc->colors_[i] = { 1., 0., 0. };   // red
                    else
                        model_pc->colors_[i] = { 0., 0., 1. };   // blue
                }
            } else {
                model_pc = pc_align;
            }
            trans = manual_trans_mesh(pc2align, model_pc);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Manual align failed: %s", e.what());
        goto retry;
    }

    middle_tf = std::make_shared<geometry_msgs::msg::TransformStamped>();
    middle_tf->header.stamp = now();
    middle_tf->header.frame_id = get_parameter("sample_lidar").as_string() + "_frame";
    middle_tf->child_frame_id = "middle";
    middle_tf->transform = tf2::eigenToTransform(Eigen::Affine3d(trans).inverse()).transform;

    tf_broadcaster->sendTransform(*middle_tf);
    RCLCPP_INFO(get_logger(), "Manual align done.");
}

void AlignerNode::pub_add_points(std::shared_ptr<const open3d::geometry::PointCloud> pc)
{
    visualization_msgs::msg::MarkerArray marker_array;
    rclcpp::Time time = rclcpp::Clock().now();

    auto add_point = [&](unsigned id, Eigen::Vector3d pos) {
        visualization_msgs::msg::Marker marker_sphere, marker_text;
        float size = 0.5;

        marker_sphere.header.frame_id.assign("world");
        marker_sphere.header.stamp = time;
        marker_sphere.ns = "sphere";
        marker_sphere.id = id;
        marker_sphere.type = visualization_msgs::msg::Marker::SPHERE;
        marker_sphere.action = visualization_msgs::msg::Marker::ADD;
        marker_sphere.scale.x = size;
        marker_sphere.scale.y = size;
        marker_sphere.scale.z = size;
        marker_sphere.pose.position.x = pos(0);
        marker_sphere.pose.position.y = pos(1);
        marker_sphere.pose.position.z = pos(2);
        marker_sphere.pose.orientation.w = 1.;
        marker_sphere.pose.orientation.x = 0.;
        marker_sphere.pose.orientation.y = 0.;
        marker_sphere.pose.orientation.z = 0.;
        if (pos(0) < 14)
            marker_sphere.color.r = 1.0, marker_sphere.color.b = 0.0;
        else
            marker_sphere.color.r = 0.0, marker_sphere.color.b = 1.0;
        marker_sphere.color.a = 0.8;
        marker_sphere.color.g = 0.0;
        marker_array.markers.push_back(marker_sphere);

        marker_text.header.frame_id.assign("world");
        marker_text.header.stamp = time;
        marker_text.ns = "text";
        marker_text.id = id;
        marker_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker_text.action = visualization_msgs::msg::Marker::ADD;
        marker_text.scale.z = 0.8;
        marker_text.pose.position.x = pos(0);
        marker_text.pose.position.y = pos(1);
        marker_text.pose.position.z = pos(2) + size;
        marker_text.pose.orientation.w = 1.;
        marker_text.pose.orientation.x = 0.;
        marker_text.pose.orientation.y = 0.;
        marker_text.pose.orientation.z = 0.;
        marker_text.color.a = 0.8;
        marker_text.color.r = 1.0;
        marker_text.color.g = 1.0;
        marker_text.color.b = 1.0;
        marker_text.text = "point: " + std::to_string(id);
        marker_array.markers.push_back(marker_text);
    };

    for (unsigned i = 0; i < pc->points_.size(); ++i)
        add_point(i, pc->points_[i]);

    marker_pub->publish(marker_array);
}

void AlignerNode::pub_del_points(unsigned size)
{
    visualization_msgs::msg::MarkerArray marker_array;
    rclcpp::Time time = rclcpp::Clock().now();

    auto remove_point = [&](unsigned id) {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id.assign("world");
        marker.header.stamp = time;
        marker.ns = "sphere";
        marker.id = id;
        marker.action = visualization_msgs::msg::Marker::DELETE;
        marker_array.markers.push_back(marker);

        marker.ns = "text";
        marker.id = id;
        marker.action = visualization_msgs::msg::Marker::DELETE;
        marker_array.markers.push_back(marker);
    };

    for (unsigned i = 0; i < size; ++i)
        remove_point(i);

    marker_pub->publish(marker_array);
}
