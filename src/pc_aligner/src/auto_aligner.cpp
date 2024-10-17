#include "pc_aligner/aligner_node.hpp"
#include <tf2_eigen/tf2_eigen.hpp>

void AlignerNode::auto_align(std::shared_ptr<open3d::geometry::PointCloud> sample_pc, Eigen::Isometry3d middle_trans, bool keep_last)
{
    RCLCPP_INFO(get_logger(), "Auto align...");
    // 将 radar 坐标系转换成 pnp 坐标系
    sample_pc->Transform(middle_trans.matrix());

    // 将 mesh 变成点云，传入参数为点云中点的数量

    // open3d::visualization::DrawGeometries({mesh_pc}, "mesh_pc", 1920, 1080);

    std::shared_ptr<open3d::geometry::PointCloud> to_align_pc;

    if (align_using_mesh) {
        auto mesh_pc = mesh_ori->SamplePointsUniformly(get_parameter("mesh_sample").as_int());
        to_align_pc = std::make_shared<open3d::geometry::PointCloud>();
        for (size_t i = 0; i < mesh_pc->points_.size(); ++i) {
            // TODO: 需要修改 z 轴余量大小
            // 将 mesh 中 z>0 部分截取出来
            if (mesh_pc->points_.at(i).z() >= -1e-3) {
                to_align_pc->points_.push_back(mesh_pc->points_.at(i));
                to_align_pc->normals_.push_back(mesh_pc->normals_.at(i));
            }
        }
        // open3d::visualization::DrawGeometries({cropped_mesh_pc}, "cropped_mesh_pc", 1920, 1080);
    } else {
        to_align_pc = pc_align;
    }
    to_align_pc->PaintUniformColor({1., 0.3, 0.3});
    auto min = get_parameter("crop_box.min").as_double_array();
    auto max = get_parameter("crop_box.max").as_double_array();

    open3d::geometry::AxisAlignedBoundingBox aabb = {
        { min[0], min[1], min[2] },
        { max[0], max[1], max[2] },
    };
    auto cropped = sample_pc->Crop(aabb);

    // open3d::visualization::DrawGeometries({cropped}, "cropped_sample_pc", 1920, 1080);
    // open3d::visualization::DrawGeometries({sample_pc, cropped, mesh_ori}, "cropped_origin_mesh", 1920, 1080);

    static Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
    if (!keep_last)
        trans = Eigen::Matrix4d::Identity();
    else
        cropped->Transform(trans);

    auto reg = [&](long max_iteration) {
        if (align_using_mesh)
            return open3d::pipelines::registration::RegistrationICP(
                *cropped, *to_align_pc,
                get_parameter("max_corr_dist").as_double(),
                Eigen::Matrix4d::Identity(),
                open3d::pipelines::registration::TransformationEstimationPointToPlane(),
                open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, max_iteration));
        else
            return open3d::pipelines::registration::RegistrationICP(
                *cropped, *to_align_pc,
                get_parameter("max_corr_dist").as_double(),
                Eigen::Matrix4d::Identity(),
                open3d::pipelines::registration::TransformationEstimationPointToPoint(),
                open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, max_iteration)); };
    if (get_parameter("vis_auto").as_bool())
    {
        auto vis = open3d::visualization::Visualizer();
        vis.CreateVisualizerWindow("Auto Aligner", 1920, 1080, 50, 50);
        vis.AddGeometry(cropped);
        vis.AddGeometry(to_align_pc);

        for (unsigned i = 0; i < get_parameter("max_iteration").as_int() && rclcpp::ok(); ++i) {
            auto reg_p2l = reg(1);
            vis.UpdateGeometry(cropped);
            vis.PollEvents();
            vis.UpdateRender();
            RCLCPP_INFO(get_logger(), "Align RMSE: %f, Fitness: %f", reg_p2l.inlier_rmse_, reg_p2l.fitness_);
            cropped->Transform(reg_p2l.transformation_);
            trans = reg_p2l.transformation_ * trans;
        }
    } else {
        auto reg_p2l = reg(get_parameter("max_iteration").as_int());
        RCLCPP_INFO(get_logger(), "Align RMSE: %f, Fitness: %f", reg_p2l.inlier_rmse_, reg_p2l.fitness_);
        cropped->Transform(reg_p2l.transformation_);
        trans = reg_p2l.transformation_ * trans;
    }

    // to_align_pc->PaintUniformColor({ 1, 1, 1 });
    // open3d::visualization::DrawGeometries({ cropped, cropped_mesh_pc }, "Aligned", 1920, 1080);

    // 发布 对齐过程 坐标系到最终的 world 坐标系的转换
    world_tf = std::make_shared<geometry_msgs::msg::TransformStamped>();
    world_tf->header.stamp = now();
    world_tf->header.frame_id = "middle";
    world_tf->child_frame_id = "world";
    world_tf->transform = tf2::eigenToTransform(Eigen::Affine3d(trans).inverse()).transform;

    tf_broadcaster->sendTransform(*world_tf);
    RCLCPP_INFO(get_logger(), "aligned tf prepared");
}
