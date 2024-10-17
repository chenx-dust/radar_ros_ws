from math import sqrt
import rclpy
import rclpy.node
import os
from ament_index_python import get_package_share_directory
from rclpy.qos import qos_profile_system_default, DurabilityPolicy
import rclpy.qos
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
from visualization_msgs.msg import MarkerArray, Marker

def main():
    rclpy.init()
    node = rclpy.node.Node("marker_pub")
    node.declare_parameter("mesh", "bg2align.stl")
    res_dir = os.path.join(get_package_share_directory("radar_bringup"), "resource")
    marker_array = MarkerArray()
    marker = Marker(header=Header(frame_id="world", stamp=node.get_clock().now().to_msg()),
                    ns="mesh", id=0, type=Marker.MESH_RESOURCE, action=Marker.ADD,
                    scale=Vector3(x=0.001, y=0.001, z=0.001),
                    pose=Pose(position=Point(x=0., y=0., z=0.),
                              orientation=Quaternion(x=-sqrt(0.5), y=0., z=0., w=sqrt(0.5))),
                    color=ColorRGBA(r=1., g=1., b=1., a=1.),
                    mesh_resource="package://radar_bringup/resource/" + node.get_parameter("mesh").value)
    marker_array.markers.append(marker)
    red_marker = Marker(header=Header(frame_id="world", stamp=node.get_clock().now().to_msg()),
                    ns="side", id=1, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD,
                    scale=Vector3(x=0., y=0., z=1.),
                    pose=Pose(position=Point(x=-1., y=7.5, z=2.),
                              orientation=Quaternion(x=0., y=0., z=0., w=1.)),
                    color=ColorRGBA(r=1., g=0., b=0., a=1.),
                    text="RED")
    blue_marker = Marker(header=Header(frame_id="world", stamp=node.get_clock().now().to_msg()),
                    ns="side", id=2, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD,
                    scale=Vector3(x=0., y=0., z=1.),
                    pose=Pose(position=Point(x=29., y=7.5, z=2.),
                              orientation=Quaternion(x=0., y=0., z=0., w=1.)),
                    color=ColorRGBA(r=0., g=0., b=1., a=1.),
                    text="BLUE")
    marker_array.markers.append(red_marker)
    marker_array.markers.append(blue_marker)
    qos = qos_profile_system_default
    qos.durability = DurabilityPolicy(DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(MarkerArray, "markers", qos)
    pub.publish(marker_array)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
