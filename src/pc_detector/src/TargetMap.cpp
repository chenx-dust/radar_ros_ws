#include "TargetMap.h"
#include "Clustering.h"
#include "VoxelGrid.h"

using namespace pc_detector;

using open3d::geometry::PointCloud;
// using open3d::geometry::TriangleMesh;

open3d::visualization::ColorMapJet cm;
Eigen::Matrix4d z_zip, z_zip_inv;

/// @brief 将向量转换为旋转矩阵
inline Eigen::Matrix3d vectorToOrthogonalMatrix(const Eigen::Vector3d& v)
{
    // 将向量规范化
    Eigen::Vector3d axis = v.normalized();
    // 计算旋转角度
    double angle = acos(axis.dot(Eigen::Vector3d::UnitZ()));
    // 计算旋转轴
    Eigen::Vector3d rot_axis = -axis.cross(Eigen::Vector3d::UnitZ()).normalized();
    // 创建旋转矩阵
    Eigen::AngleAxisd rotation(angle, rot_axis);
    return rotation.toRotationMatrix();
}

/// @brief DBSCAN 聚类
void TargetMap::cluster(const PointCloud &pc, double eps_, size_t min_points_, std::vector<int>& labels, std::vector<std::shared_ptr<PointCloud> > &pcs, PointCloud &pc_noise,
    std::vector<BoundingBox> &aabbs, std::vector<size_t> &pt_num, std::vector<Eigen::Vector3d> &grav)
{
    if (pc.points_.empty())
        return;
    PointCloud pc_tmp = pc;
    pc_tmp.Transform(z_zip);
    labels = NormalDBSCAN(pc_tmp, eps_, min_points_);
    int max_l = *std::max_element(labels.begin(), labels.end());
    for (int i = 0; i <= max_l; i++)
        pcs.emplace_back(std::make_shared<PointCloud>());
    for (size_t i = 0; i < pc.points_.size(); i++)
        if (labels[i] >= 0)
            pcs[labels[i]]->points_.push_back(pc.points_[i]);
        else
            pc_noise.points_.push_back(pc.points_[i]);
    for (int i = 0; i <= max_l; i++) {
        aabbs.emplace_back(pcs[i]->GetAxisAlignedBoundingBox());
        pt_num.emplace_back(pcs[i]->points_.size());
        grav.emplace_back(pcs[i]->GetCenter());
    }
}

/// @brief DBSCAN 聚类 (带距离权重)
void TargetMap::cluster_d(const PointCloud &pc, const Eigen::Vector3d &zero_pos, double eps_, size_t min_points_k,
    std::vector<int>& labels, std::vector<std::shared_ptr<PointCloud> > &pcs, PointCloud &pc_noise,
    std::vector<BoundingBox> &aabbs, std::vector<size_t> &pt_num, std::vector<Eigen::Vector3d> &grav)
{
    if (pc.points_.empty())
        return;
    PointCloud pc_tmp = pc;
    pc_tmp.Transform(z_zip);
    labels = DifferingDBSCAN(pc_tmp, zero_pos, eps_, min_points_k);
    int max_l = *std::max_element(labels.begin(), labels.end());
    for (int i = 0; i <= max_l; i++)
        pcs.emplace_back(std::make_shared<PointCloud>());
    for (size_t i = 0; i < pc.points_.size(); i++)
        if (labels[i] >= 0)
            pcs[labels[i]]->points_.push_back(pc.points_[i]);
        else
            pc_noise.points_.push_back(pc.points_[i]);
    for (int i = 0; i <= max_l; i++) {
        aabbs.emplace_back(pcs[i]->GetAxisAlignedBoundingBox());
        pt_num.emplace_back(pcs[i]->points_.size());
        grav.emplace_back(pcs[i]->GetCenter());
    }
}

void TargetMap::set_params(const TargetMapParams& p)
{
    params = p;

    double z_zip_c = params.z_zip;

    z_zip = Eigen::DiagonalMatrix<double, 4> { 1, 1, z_zip_c, 1 };
    z_zip_inv = Eigen::DiagonalMatrix<double, 4> { 1, 1, 1 / z_zip_c, 1 };

}

/// @brief 更新跟踪队列中的一个元素
void TargetMap::element_update(size_t id, const BoundingBox& aabb, size_t pt_num, Eigen::Vector3d grav)
{
    if (target_map.at(id).pt_num > 0) {
        target_map.at(id).aabb.max_bound = target_map.at(id).aabb.max_bound.cwiseMax(aabb.max_bound);
        target_map.at(id).aabb.min_bound = target_map.at(id).aabb.min_bound.cwiseMin(aabb.min_bound);
    }
    else {
        target_map.at(id).aabb.max_bound = aabb.max_bound;
        target_map.at(id).aabb.min_bound = aabb.min_bound;
    }
    // 更新重心, 对可能的多个聚类确定为同一目标的情况下, 进行平均
    target_map.at(id).grav = (pt_num * grav + target_map.at(id).pt_num * target_map.at(id).grav) / (pt_num + target_map.at(id).pt_num);
    target_map.at(id).pt_num += pt_num;
}

/// @brief 更新前阶段处理
void TargetMap::pre_update()
{
    for (auto& [id, target] : target_map) {
        target.pt_num = 0;
    }
}

/// @brief 更新后阶段处理, 包括更新卡尔曼滤波器, 更新可视化球体
void TargetMap::post_update()
{
    for (auto it = target_map.begin(); it != target_map.end();) {
        if (it->second.lost_time > params.last_frames) {
            // spdlog::info("TargetMap: Discard target {}.", id);
            RCLCPP_DEBUG(params.node->get_logger(), "TargetMap: Discard target %d.", it->first);
            it = target_map.erase(it);
            continue;
        }
        if (it->second.pt_num > 0) {
            it->second.kf.update(it->second.grav(Eigen::seq(0, 1)));
            it->second.lost_time = it->second.lost_time > 0 ? it->second.lost_time - 1 : 0;
        } else {
            it->second.kf.update();
            it->second.lost_time++;
        }
        ++it;
    }
}

/// @brief 将多个跟踪目标合并为一个新的跟踪目标
void TargetMap::combine(size_t new_id, std::vector<size_t> old_ids)
{
    for (size_t id : old_ids) {
        if (id == new_id || target_map.at(id).lost_time < params.combine_limit)
            continue;
        // spdlog::info("TargetMap: Combine {} to {}", id, new_id);
        RCLCPP_DEBUG(params.node->get_logger(), "TargetMap: Combine %lu to %lu.", id, new_id);
        // target_map[new_id].aabb.max_bound = (target_map[new_id].pt_num * target_map[new_id].aabb.max_bound + target_map.at(id).pt_num * target_map.at(id).aabb.max_bound) / (target_map[new_id].pt_num + target_map.at(id).pt_num);
        // target_map[new_id].aabb.min_bound = (target_map[new_id].pt_num * target_map[new_id].aabb.min_bound + target_map.at(id).pt_num * target_map.at(id).aabb.min_bound) / (target_map[new_id].pt_num + target_map.at(id).pt_num);
        // target_map[new_id].grav = (target_map[new_id].pt_num * target_map[new_id].grav + target_map.at(id).pt_num * target_map.at(id).grav) / (target_map[new_id].pt_num + target_map.at(id).pt_num);
        // target_map[new_id].pt_num += target_map.at(id).pt_num;
        target_map.erase(id);
    }
}

/// @brief 将多个跟踪目标合并为一个新的跟踪目标 (强制合并)
void TargetMap::combine_force(size_t new_id, std::vector<size_t> old_ids)
{
    for (size_t id : old_ids) {
        if (id == new_id)
            continue;
        // spdlog::info("TargetMap: Force Combine {} to {}", id, new_id);
        RCLCPP_DEBUG(params.node->get_logger(), "TargetMap: Force Combine %lu to %lu.", id, new_id);
        target_map.erase(id);
    }
}

/// @brief 尝试匹配已经存在的跟踪目标, 进行更新
size_t TargetMap::push(const BoundingBox& aabb, size_t pt_num, Eigen::Vector3d grav, const PointCloud& pc, bool no_strict)
{
    int64_t min_id = -1;
    double min_dis = -1;
    std::vector<size_t> combine_id_list, combine_force_id_list, seperate_id_list;
    for (auto& [id, target] : target_map) {
        // double dis = sqr_dis(grav, target_map[i].kf.pos());
        if (is_contain(aabb, target_map.at(id).pos()) && target_map.at(id).lost_time < params.separate_limit)
            seperate_id_list.push_back(id);
        double mah_dis = target_map.at(id).kf.mah_dis(grav(Eigen::seq(0, 1)));
        // if (dis > dis_thres * dis_thres && !is_coincide(aabb, *target_map[i].aabb))
        // 对于匹配目标距离过远, 应该不是同一个目标, 不应该进行合并
        if (mah_dis > params.dist_thres * params.dist_thres && !is_coincide(aabb, target_map.at(id).aabb))
            continue;
        if (mah_dis <= params.combine_dist * params.combine_dist)
            combine_id_list.push_back(id);
        if ((target_map.at(id).pos() - grav).squaredNorm() <= params.force_combine_dist * params.force_combine_dist)
            combine_force_id_list.push_back(id);
        if (mah_dis < min_dis || min_id == -1) {
                min_id = id;
                min_dis = mah_dis;
        }
        // spdlog::info("TargetMap: mah_dis: {}", mah_dis);
    }
    // 如果 aabb 与多个目标重合, 则告知出现了错误合并, 需要进行分离
    if (seperate_id_list.size() > 1 && !no_strict) {
        RCLCPP_DEBUG(params.node->get_logger(), "TargetMap: Seperate %lu targets.", seperate_id_list.size());
        seperate(aabb, pc);
        return TM_NEED_SEPERATE;
    }
    if (min_id != -1) {
        // if (combine_id_list.size() > 1)
        //     combine(min_id, combine_id_list);
        if (combine_force_id_list.size() > 1)
            combine_force(min_id, combine_force_id_list);
        element_update(min_id, aabb, pt_num, grav);
        return min_id;
    }
    /// 没有匹配到目标, 进行插入
    return new_target(aabb, pt_num, grav);
}

/// @brief 插入新的跟踪目标
size_t TargetMap::new_target(const BoundingBox& aabb, size_t pt_num, Eigen::Vector3d grav)
{
    target_map.emplace(inc_id, Target { 0, KalmanFilter(grav(Eigen::seq(0, 1))), 0, Eigen::Vector3d::Zero(), aabb });
    element_update(inc_id, aabb, pt_num, grav);
    // spdlog::info("TargetMap: Added new target {}.", inc_id);
    RCLCPP_DEBUG(params.node->get_logger(), "TargetMap: Added new target %lu.", inc_id);
    return inc_id++;
}

/// @brief 对于没有匹配到的目标, 进行松弛查询
void TargetMap::loose_query(const PointCloud &pc)
{
    std::vector<BoundingBox> aabbs;
    // 对于跟踪目标中的每一个目标, 如果没有在正常更新中被更新, 则将包围盒拿出来, 进行松弛查询
    for (auto& [id, target] : target_map) {
        if (target.pt_num == 0 && target.lost_time < params.combine_limit)
            aabbs.push_back(target.aabb);
    }
    for (const auto& aabb : aabbs) {
        BoundingBox aabb_expand = aabb;
        aabb_expand.max_bound.array() += params.loose_expand;
        aabb_expand.min_bound.array() -= params.loose_expand;
        PointCloud pc_crop = *pc.Crop(aabb_expand.to_aabb());
        if (pc_crop.points_.empty()) continue;
        std::vector<std::shared_ptr<PointCloud> > pcs;
        PointCloud pc_noise_x;
        std::vector<int> labels;
        std::vector<BoundingBox> aabbs_;
        std::vector<size_t> pt_num;
        std::vector<Eigen::Vector3d> grav;
        TargetMap::cluster(pc_crop, params.loose.eps, params.loose.min_points, labels, pcs, pc_noise_x, aabbs_, pt_num, grav);
        // spdlog::info("TargetMap: Loose query {} clusters.", pcs.size());
        for (size_t i = 0; i < aabbs_.size(); i++)
            // 对识别到的每个聚类交由 push 函数进行处理, 返回跟踪目标的 id
            push(aabbs_[i], pt_num[i], grav[i], pc);
    }
}

/// @brief 将一个跟踪目标分裂为多个新的跟踪目标
void TargetMap::seperate(const BoundingBox &aabb, const PointCloud &pc)
{
    PointCloud pc_cropped = *pc.Crop(aabb.to_aabb());

    std::vector<std::shared_ptr<PointCloud> > pcs;
    PointCloud pc_noise;
    std::vector<int> labels;
    std::vector<BoundingBox> aabbs;
    std::vector<size_t> pt_num;
    std::vector<Eigen::Vector3d> grav;
    TargetMap::cluster(pc_cropped, params.strict.eps, params.strict.min_points, labels, pcs, pc_noise, aabbs, pt_num, grav);

    if (pcs.size() == 1) {
        // spdlog::info("TargetMap: Seperate failed.");
        RCLCPP_DEBUG(params.node->get_logger(), "TargetMap: Seperate failed.");
        return;
    }

    for (size_t i = 0; i < aabbs.size(); i++) {
        // 对识别到的每个聚类交由 push 函数进行处理, 返回跟踪目标的 id
        push(aabbs[i], pt_num[i], grav[i], pc, true);
    }
}

/// @brief 批次更新跟踪目标, 返回跟踪目标的id
/// @param pc 点云
/// @param cluster_labels 聚类标签（原始标签）
/// @param tracking_ids 点对应的跟踪目标的id
void TargetMap::update(const PointCloud& pc, std::vector<int> &cluster_labels, std::vector<int> &tracking_ids)
{
    std::vector<BoundingBox> aabbs;
    std::vector<size_t> pt_nums;
    std::vector<Eigen::Vector3d> grav;
    std::vector<int> id_map;
    std::vector<std::shared_ptr<PointCloud>> pcs;
    PointCloud pc_noise;
    cluster(pc, params.normal.eps, params.normal.min_points, cluster_labels, pcs, pc_noise, aabbs, pt_nums, grav);
    pre_update();
    /// 贪心算法
    // spdlog::info("TargetMap: Update {} clusters.", pcs.size());
    for (size_t i = 0; i < pcs.size(); i++) {
        // 对识别到的每个聚类交由 push 函数进行处理, 返回跟踪目标的 id
        id_map.push_back(push(aabbs[i], pt_nums[i], grav[i], pc));
    }
    // 对于没有匹配到的目标, 进行松弛查询
    loose_query(pc);
    post_update();
    tracking_ids.reserve(cluster_labels.size());
    for (auto& l : cluster_labels)
        if (l >= 0)
            tracking_ids.push_back(id_map.at(l));
        else
            tracking_ids.push_back(TM_NOISE);
}
