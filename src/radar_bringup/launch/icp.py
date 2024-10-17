from launch import LaunchDescription, actions
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from tf2_geometry_msgs.tf2_geometry_msgs import _get_quat_from_mat, _build_affine, _decompose_affine


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='livox_v2_lidar',
            executable='livox_v2_lidar_node',
            namespace='hap',
        ),
        Node(
            package='livox_v1_lidar',
            executable='livox_v1_lidar_node',
            namespace='mid70',
        ),
        Node(
            package='radar_utils',
            executable='icp',
        ),
    ])
