import sys
import numpy as np
import open3d as o3d
import rclpy
import rclpy.node
import rclpy.logging
import rclpy.qos
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_list

limit = 1_000_000
process_calc = 10000
process = 0
pc1 = o3d.geometry.PointCloud()
pc2 = o3d.geometry.PointCloud()

def pc_callback(msg: PointCloud2, o3d_pc):
    pts = np.asarray(read_points_list(msg, ['x', 'y', 'z']))
    o3d_pc.points.extend(pts)

def pick_points(pcd):
    print("")
    print(
        "1) Please pick at least three correspondences using [shift + left click]"
    )
    print("   Press [shift + right click] to undo point picking")
    print("2) After picking points, press 'Q' to close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    return vis.get_picked_points()


def register_via_correspondences(source, target, source_points, target_points):
    corr = np.zeros((len(source_points), 2))
    corr[:, 0] = source_points
    corr[:, 1] = target_points
    # estimate rough transformation using correspondences
    print("Compute a rough transform using the correspondences given by user")
    p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    trans_init = p2p.compute_transformation(source, target,
                                            o3d.utility.Vector2iVector(corr))
    # point-to-point ICP for refinement
    print("Perform point-to-point ICP refinement")
    threshold = 0.03  # 3cm distance threshold
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    source.transform(reg_p2p.transformation)

def main():
    global pc1, pc2
    rclpy.init()
    node = rclpy.node.Node("record")
    node.create_subscription(PointCloud2, "/hap/pc_raw",
                             lambda m: pc_callback(m, pc1), rclpy.qos.qos_profile_system_default)
    node.create_subscription(PointCloud2, "/mid70/pc_raw",
                             lambda m: pc_callback(m, pc2), rclpy.qos.qos_profile_system_default)
    while np.asarray(pc1.points).shape[0] < limit:
        rclpy.spin_once(node)
        global process
        if process_calc * process < np.asarray(pc1.points).shape[0]:
            process += 1
            node.get_logger().info(f"Process: {process * process_calc}/{limit}")
    node.get_logger().info("Finish.")
    pc1 = pc1.crop(o3d.geometry.AxisAlignedBoundingBox(
        (0, -5, -5), (10, 5, 5)))
    pc2 = pc2.crop(o3d.geometry.AxisAlignedBoundingBox(
        (0, -5, -5), (10, 5, 5)))
    pc2.transform(np.array([[0.90833842,  0.00773934,  0.41816435,  0.04892528],
                            [0.0060087, -0.99996707,  0.00545515, -0.01346022],
                            [0.41819279, -0.00244249, -0.90835501, -0.07771256],
                            [0.,  0.,  0.,  1.],]))
    pc1.points.extend(pc2.points)
    # o3d.visualization.draw(pc1)
    selected = pick_points(pc1)
    ground = o3d.geometry.PointCloud()
    ground.points.extend(np.array([[7.5, 5.0, 0.],[10.,5.0,0.],[7.5,7.5,0.]]))
    register_via_correspondences(pc1, ground, selected, [0, 1, 2])
    o3d.visualization.draw_geometries([pc1])
    o3d.io.write_point_cloud("output.pcd", pc1)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
