#pragma once

#include <Eigen/Core>
#include <open3d/Open3D.h>
#include <open3d/t/geometry/RaycastingScene.h>

namespace pc_detector {

struct BoundingBox {
    Eigen::Vector3d min_bound;
    Eigen::Vector3d max_bound;

    BoundingBox() = default;
    // 用于 Open3D 向 PCL 过渡
    BoundingBox(const open3d::geometry::AxisAlignedBoundingBox& aabb)
    {
        min_bound = aabb.GetMinBound();
        max_bound = aabb.GetMaxBound();
    }
    open3d::geometry::AxisAlignedBoundingBox to_aabb() const { return { min_bound, max_bound }; }
};

struct Z_Map {
    using SharedPtr = std::shared_ptr<Z_Map>;

    open3d::geometry::AxisAlignedBoundingBox bbox;
    double voxel_size;
    Eigen::Vector3i grid_size;
    Eigen::MatrixXf z_map;

    Z_Map(Eigen::Vector3d grid_min, Eigen::Vector3d grid_max, double voxel_size, std::shared_ptr<const open3d::geometry::TriangleMesh> mesh)
        : bbox(grid_min, grid_max)
        , voxel_size(voxel_size)
        , grid_size(((grid_max - grid_min) / voxel_size + Eigen::Vector3d::Ones()).cast<int>())
        , z_map(Eigen::MatrixXf::Zero(grid_size[0], grid_size[1]))
    {
        double z_max = bbox.max_bound_(2);
        open3d::t::geometry::RaycastingScene scene_r;
        scene_r.AddTriangles(open3d::t::geometry::TriangleMesh::FromLegacy(*mesh));
        z_map = Eigen::MatrixXf::Zero(grid_size[0], grid_size[1]);
        Eigen::MatrixXf Q(grid_size[0] * grid_size[1], 6); // 查询光线
        for (int i = 0; i < grid_size[0]; ++i) {
            for (int j = 0; j < grid_size[1]; ++j) {
                Q.row(i * grid_size[1] + j)(Eigen::seq(0, 1)) = get_grid_center({ i, j, 0 })(Eigen::seq(0, 1)).cast<float>();
                Q.row(i * grid_size[1] + j)(2) = z_max;
                Q.row(i * grid_size[1] + j)(Eigen::seq(3, 5)) = Eigen::Vector3f { 0, 0, -1 };
            }
        }
        open3d::core::Tensor Q_tensor = open3d::core::eigen_converter::EigenMatrixToTensor(Q);
        std::unordered_map<std::string, open3d::core::Tensor> rslt_r = scene_r.CastRays(Q_tensor);
        open3d::core::Tensor rslt_hit = rslt_r["t_hit"];
        for (int i = 0; i < grid_size[0]; ++i)
            for (int j = 0; j < grid_size[1]; ++j) {
                z_map(i, j) = z_max - rslt_hit[i * grid_size[1] + j].Item<float>();
            }
    }
    Eigen::Vector3d project_ground(const Eigen::Vector2d& pt) const
    {
        /// @brief 将点投影到地面
        Eigen::Vector2d pt_grid = (pt - bbox.min_bound_(Eigen::seq(0, 1))) / voxel_size;
        Eigen::Vector2i pt_grid_int = pt_grid.cast<int>();
        if ((pt_grid_int.array() >= grid_size(Eigen::seq(0, 1)).array()).any() || (pt_grid_int.array() < 0).any())
            return Eigen::Vector3d(pt(0), pt(1), 0);
        else
            return Eigen::Vector3d(pt(0), pt(1), z_map(pt_grid_int(0), pt_grid_int(1)));
    }
    Eigen::Vector3d get_grid_center(const Eigen::Vector3i& grid_idx) const
    {
        /// @brief 获取体素中心点坐标
        return bbox.min_bound_ + (grid_idx.cast<double>() + Eigen::Vector3d::Ones() * 0.5) * voxel_size;
    }
};
}
