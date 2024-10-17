from launch import LaunchDescription, actions
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from tf2_geometry_msgs.tf2_geometry_msgs import _get_quat_from_mat, _build_affine, _decompose_affine

from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch.actions import TimerAction, Shutdown
from launch import LaunchDescription

import numpy as np
import os

debug = True

node_params = os.path.join(
    get_package_share_directory('radar_bringup'),
    'config',
    'config.24.national.yaml'
)


def get_xyzw_tf_broadcaster(cali: list, fr: str, child_fr: str):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace='radar',
        name=fr+'_to_'+child_fr,
        parameters=[{'use_sim_time': True}],
        arguments=['--x', str(cali[0]),
                   '--y', str(cali[1]),
                   '--z', str(cali[2]),
                   '--qx', str(cali[3]),
                   '--qy', str(cali[4]),
                   '--qz', str(cali[5]),
                   '--qw', str(cali[6]),
                   '--frame-id', fr,
                   '--child-frame-id', child_fr],)


def get_matrix_tf_broadcaster(cali: np.array, fr: str, child_fr: str):
    quat, trans = _decompose_affine(cali)
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace='radar',
        name=fr+'_to_'+child_fr,
        parameters=[{'use_sim_time': True}],
        arguments=['--x', str(trans[0]),
                   '--y', str(trans[1]),
                   '--z', str(trans[2]),
                   '--qw', str(quat[0]),
                   '--qx', str(quat[1]),
                   '--qy', str(quat[2]),
                   '--qz', str(quat[3]),
                   '--frame-id', fr,
                   '--child-frame-id', child_fr],)


def get_vision_container(cam_name: str):
    if not debug:
        return (ComposableNodeContainer(
            name=cam_name + '_vision_container',
            namespace='radar',
            package='rclcpp_components',
            executable='component_container_isolated',
            arguments=['--use_multi_threaded_executor'],
            composable_node_descriptions=[
                # ComposableNode(
                #     package='hik_camera',
                #     plugin='hik_camera::HikCameraNode',
                #     name='hik_camera',
                #     namespace='radar/' + cam_name,
                #     parameters=[node_params],
                #     extra_arguments=[{'use_intra_process_comms': True}]
                # ),
                ComposableNode(
                    package='img_recognizer',
                    plugin='img_recognizer::RecognizerNode',
                    name='img_recognizer',
                    namespace='radar/' + cam_name,
                    parameters=[
                        node_params,
                        {'use_sim_time': True,
                         'img_compressed': True},
                        ],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
            ],
            output='both',
            emulate_tty=True,
            on_exit=Shutdown(),
        ),)
    else:
        return (
            # Node(
            #     package='hik_camera',
            #     executable='info_pub',
            #     namespace='radar/' + cam_name,
            #     parameters=[
            #         {'use_sim_time': True},
            #         node_params,
            #     ],
            # ),
            Node(
                package='img_recognizer',
                executable='img_recognizer_node',
                namespace='radar/' + cam_name,
                parameters=[
                    node_params,
                    {'use_sim_time': True,
                     'img_compressed': True},
                ],
            ),
        )


def get_pc_container():
    if not debug:
        return (ComposableNodeContainer(
            name='pc_container',
            namespace='radar',
            package='rclcpp_components',
            executable='component_container_isolated',
            arguments=['--use_multi_threaded_executor'],
            composable_node_descriptions=[
                # ComposableNode(
                #     package='livox_v1_lidar',
                #     plugin='livox_v1_lidar::LidarPublisher',
                #     name='livox_v1_lidar',
                #     namespace='radar/' + 'lidar_mid70',
                #     parameters=[node_params],
                #     extra_arguments=[{'use_intra_process_comms': True}]
                # ),
                # ComposableNode(
                #     package='livox_v2_lidar',
                #     plugin='livox_v2_lidar::LidarPublisher',
                #     name='livox_v2_lidar',
                #     namespace='radar/' + 'lidar_hap',
                #     parameters=[node_params],
                #     extra_arguments=[{'use_intra_process_comms': True}]
                # ),
                ComposableNode(
                    package='pc_detector',
                    plugin='pc_detector::DetectorNode',
                    name='pc_detector',
                    namespace='radar',
                    parameters=[node_params,
                                {'use_sim_time': True}],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
            ],
            output='both',
            emulate_tty=True,
            on_exit=Shutdown(),
        ),)
    else:
        return (
            # Node(
            #     package='livox_v1_lidar',
            #     executable='livox_v1_lidar_node',
            #     name='livox_v1_lidar',
            #     namespace='radar/' + 'lidar_mid70',
            #     parameters=[node_params],
            # ),
            # Node(
            #     package='livox_v2_lidar',
            #     executable='livox_v2_lidar_node',
            #     name='livox_v2_lidar',
            #     namespace='radar/' + 'lidar_hap',
            #     parameters=[node_params],
            # ),
            Node(
                package='pc_detector',
                executable='pc_detector_node',
                name='pc_detector',
                namespace='radar',
                parameters=[node_params,
                            {'use_sim_time': True}],
            ),
        )


def generate_launch_description():
    return LaunchDescription([
        *get_vision_container('hik_4mm'),
        get_xyzw_tf_broadcaster(
            [
                0.05190184339880943,
                -0.11320271342992783,
                -0.11610990762710571,
                0.38116112374482103,
                0.3843852681403423,
                0.5903648363415522,
                0.5986931779740841
            ], 'lidar_hap_frame', 'hik_4mm_frame'
        ),
        *get_vision_container('hik_8mm'),
        get_xyzw_tf_broadcaster(
            [
                0.059292495250701904,
                -0.10663162916898727,
                0.004948855843394995,
                0.49931320973839743,
                0.49561537544531076,
                0.4977604777199716,
                0.5072338976832756
            ], 'lidar_hap_frame', 'hik_8mm_frame'
        ),
        *get_vision_container('hik_12mm'),
        get_xyzw_tf_broadcaster(
            [
                0.05408414080739021,
                -0.09837870299816132,
                0.05553804710507393,
                0.4978566713677437,
                0.49243206474914125,
                0.5044147516238935,
                0.5051882073140497
            ], 'lidar_hap_frame', 'hik_12mm_frame'
        ),
        *get_pc_container(),
        get_matrix_tf_broadcaster(
            np.array([[0.90805441,  0.00851127,  0.41876575,  0.05435923],
                      [0.00681501, -0.9999614,  0.00554616, -0.01593622],
                      [0.41879679, -0.00218231, -0.90807736, -0.07701991],
                      [0.,  0.,  0.,  1.],]), 'lidar_hap_frame', 'lidar_mid70_frame'),
        Node(
            package='radar_utils',
            executable='marker_pub',
            namespace='radar',
            parameters=[{"mesh": "24_bg2align_fix1.stl"}],
            output='both',
        ),
        Node(
            package='pc_aligner',
            executable='pc_aligner',
            namespace='radar',
            parameters=[node_params,
                        {'use_sim_time': True}],
            output='both',
        ),
        Node(
            package='target_matcher',
            executable='target_matcher',
            namespace='radar',
            parameters=[node_params,
                        {'use_sim_time': True}],
            output='both',
        ),
        # Node(
        #     package='target_multiplexer',
        #     executable='target_multiplexer',
        #     namespace='radar',
        #     parameters=[node_params],
        #     output='both',
        # ),
        Node(
            package='dv_trigger',
            executable='dv_trigger',
            namespace='radar',
            parameters=[node_params,
                        {'use_sim_time': True}],
            output='both',
        ),
        Node(
            package='radar_supervisor',
            executable='radar_supervisor',
            namespace='radar',
            parameters=[node_params,
                        {'use_sim_time': True}],
            output='both',
        ),
        Node(
            package='judge_bridge',
            executable='judge_bridge',
            name='judge_bridge',
            namespace='radar',
            parameters=[node_params,
                        {'use_sim_time': True}],
            output='both',
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
        ),
        Node(
            package='target_visualizer',
            executable='target_visualizer',
            namespace='radar',
            output='both',
        ),
        Node(
            package='result_visualizer',
            executable='result_visualizer',
            namespace='radar',
            output='both',
        ),
    ])
