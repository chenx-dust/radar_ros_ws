import sys
import numpy as np
import open3d as o3d
import rclpy
import rclpy.node
import rclpy.logging
import rclpy.qos
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_list

limit = 2_000_000
full_num = 0
pc1 = o3d.geometry.PointCloud()
pc2 = o3d.geometry.PointCloud()

def solve():
    threshold = 0.05
    icp_iteration = 40

    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

    global pc1, pc2
    pc1 = pc1.crop(o3d.geometry.AxisAlignedBoundingBox([3, -3, -5], [10, 3, 5]))
    pc1.estimate_normals()
    pc1.paint_uniform_color([1., 0., 0.])
    pc2 = pc2.crop(o3d.geometry.AxisAlignedBoundingBox([3, -3, -5], [10, 3, 5]))
    pc2.estimate_normals()
    pc2.paint_uniform_color([0., 1., 0.])
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pc1)
    vis.add_geometry(pc2)
    save_image = False
    trans = np.array([[0.90805441,  0.00851127,  0.41876575,  0.05435923],
                      [0.00681501, -0.9999614,  0.00554616, -0.01593622],
                      [0.41879679, -0.00218231, -0.90807736, -0.07701991],
                      [0.,  0.,  0.,  1.],])

    pc1.transform(trans)
    for i in range(icp_iteration):
        reg_p2l = o3d.pipelines.registration.registration_icp(
            pc1, pc2, threshold, np.identity(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1))
        pc1.transform(reg_p2l.transformation)
        trans = reg_p2l.transformation.dot(trans)
        vis.update_geometry(pc1)
        vis.poll_events()
        vis.update_renderer()
        if save_image:
            vis.capture_screen_image("temp_%04d.jpg" % i)
    threshold = 0.01
    for i in range(icp_iteration):
        reg_p2l = o3d.pipelines.registration.registration_icp(
            pc1, pc2, threshold, np.identity(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1))
        pc1.transform(reg_p2l.transformation)
        trans = reg_p2l.transformation.dot(trans)
        vis.update_geometry(pc1)
        vis.poll_events()
        vis.update_renderer()
        if save_image:
            vis.capture_screen_image("temp_%04d.jpg" % i)
    threshold = 0.005
    for i in range(icp_iteration):
        reg_p2l = o3d.pipelines.registration.registration_icp(
            pc1, pc2, threshold, np.identity(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1))
        pc1.transform(reg_p2l.transformation)
        trans = reg_p2l.transformation.dot(trans)
        vis.update_geometry(pc1)
        vis.poll_events()
        vis.update_renderer()
        if save_image:
            vis.capture_screen_image("temp_%04d.jpg" % i)

    rclpy.logging.get_logger("icp").info(f"result:\n{trans}")
    vis.destroy_window()

def pc_callback(msg: PointCloud2, o3d_pc):
    pts = np.asarray(read_points_list(msg, ['x', 'y', 'z']))
    o3d_pc.points.extend(pts)
    if np.asarray(o3d_pc.points).shape[0] >= limit:
        global full_num
        full_num += 1
        rclpy.logging.get_logger("receive").info(f"full {full_num}")

def main():
    topic1 = "/hap/pc_raw"
    topic2 = "/mid70/pc_raw"
    rclpy.init()
    node = rclpy.node.Node("icp")
    node.create_subscription(PointCloud2, topic1,
                             lambda m: pc_callback(m, pc1), rclpy.qos.qos_profile_system_default)
    node.create_subscription(PointCloud2, topic2,
                             lambda m: pc_callback(m, pc2), rclpy.qos.qos_profile_system_default)
    while full_num < 2:
        rclpy.spin_once(node)
    solve()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
