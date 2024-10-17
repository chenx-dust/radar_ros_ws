from launch import LaunchDescription, actions
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from tf2_geometry_msgs.tf2_geometry_msgs import _get_quat_from_mat, _build_affine, _decompose_affine

import numpy as np
import os

pnp_trans_ = {"rmat":[[-0.15693326863896367,-0.9873704625195574,-0.021714486828070766],
                     [-0.4212989940342729,0.08681516962926414,-0.9027570459143216],
                     [0.8932407888246041,-0.13252432254555271,-0.4296023709360107]],
             "tvec_mm":[9463.615330043687,2221.851249709676,3767.0116980334747]}
pnp_trans = np.eye(4, dtype=np.float64)
pnp_trans[:3, :3] = pnp_trans_["rmat"]
pnp_trans[:3, 3] = np.asarray(pnp_trans_["tvec_mm"]).T
# pnp_trans = np.linalg.inv(pnp_trans)
pnp_quat = _get_quat_from_mat(np.asarray(pnp_trans[:3, :3]))

t_lidar_cam = [
    -0.027836976599217165,
    -0.10324384287728583,
    -0.01244172499789492,
    -0.49023953382411173,
    0.5751660581671042,
    -0.4800265388404169,
    0.4454477825933392
]

aff = _build_affine([-t_lidar_cam[6], t_lidar_cam[3], t_lidar_cam[4], t_lidar_cam[5]],
                    [t_lidar_cam[0], t_lidar_cam[1], t_lidar_cam[2]])
rot, tsl = _decompose_affine(np.linalg.inv(aff))


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('radar_bringup'),
        'config',
        'config.23record.yaml'
    )

    return LaunchDescription([
        # Node(
        #     package='pc_aligner',
        #     executable='pc_aligner',
        #     namespace='radar',
        #     parameters=[config],
        # ),
        Node(
            package='pc_detector',
            executable='pc_detector_node',
            namespace='radar',
            parameters=[config],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace='radar',
            name='world2cam',
            arguments=['--x', str(pnp_trans[0, 3] / 1000),
                       '--y', str(pnp_trans[1, 3] / 1000),
                       '--z', str(pnp_trans[2, 3] / 1000),
                       '--qw', str(pnp_quat[0]),
                       '--qx', str(pnp_quat[1]),
                       '--qy', str(pnp_quat[2]),
                       '--qz', str(pnp_quat[3]),
                       '--frame-id', 'cam_frame',
                       '--child-frame-id', 'world'],
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     namespace='radar',
        #     name='lidar2cam',
        #     arguments=['--x', str(t_lidar_cam[0]),
        #                '--y', str(t_lidar_cam[1]),
        #                '--z', str(t_lidar_cam[2]),
        #                '--qx', str(t_lidar_cam[3]),
        #                '--qy', str(t_lidar_cam[4]),
        #                '--qz', str(t_lidar_cam[5]),
        #                '--qw', str(-t_lidar_cam[6]),
        #                '--frame-id', 'cam_frame',
        #                '--child-frame-id', 'livox_frame'],
        # ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace='radar',
            name='cam2lidar',
            arguments=['--x', str(tsl[0]),
                       '--y', str(tsl[1]),
                       '--z', str(tsl[2]),
                       '--qw', str(rot[0]),
                       '--qx', str(rot[1]),
                       '--qy', str(rot[2]),
                       '--qz', str(rot[3]),
                       '--frame-id', 'livox_frame',
                       '--child-frame-id', 'cam_frame'],
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
        ),
        Node(
            package='target_visualizer',
            executable='target_visualizer',
            namespace='radar',
        ),
        Node(
            package='result_visualizer',
            executable='result_visualizer',
            namespace='radar',
        ),
        # Node(
        #     package='nn_detector',
        #     executable='nn_detector_node',
        #     namespace='radar',
        #     parameters=[config],
        # ),
        Node(
            package='img_recognizer',
            executable='img_recognizer_node',
            namespace='radar/cam',
            parameters=[config],
        ),
        Node(
            package='target_matcher',
            executable='target_matcher',
            namespace='radar',
            parameters=[config],
        ),
        Node(
            package='judge_bridge',
            executable='judge_bridge',
            namespace='radar',
            parameters=[config],
        ),
        Node(
            package='target_multiplexer',
            executable='target_multiplexer',
            namespace='radar',
            parameters=[config],
        ),
        Node(
            package='dv_trigger',
            executable='dv_trigger',
            namespace='radar',
            parameters=[config],
        ),
        Node(
            package='radar_supervisor',
            executable='radar_supervisor',
            namespace='radar',
        ),
        Node(
            package='radar_utils',
            executable='marker_pub',
            namespace='radar',
            parameters=[{"mesh": "23_bg2align.stl"}]
        ),
        actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '/home/chenx/Documents/radar_bag',
                 '--remap', '/radar/camera/compressed:=/radar/cam/image/compressed',
                 '/radar/camera/camera_info:=/radar/cam/camera_info',
                 '/radar/lidar/pc_raw:=/radar/livox/pc_raw',],
            output='screen'
        )
    ])
