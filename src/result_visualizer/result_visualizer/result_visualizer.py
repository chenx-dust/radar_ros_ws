import os
import rclpy
import rclpy.qos
import numpy as np
from ament_index_python import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from radar_interface.msg import MatchResult, MatchedTarget, RadarMarkData

import cv2

configs = {
    "real_width": 28.0,
    "real_height": 15.0,
}

# Target:
# uint64 id
# float64[2] position
# float64[4] pos_covariance
# float64[2] velocity
# float64[4] vel_covariance
# float64 calc_z
# float64[3] observed_pos
# uint32 uncertainty

class ResultVisualizer(Node):
    team_color = True    # False: Blue, True: Red
    mark = [0., 0., 0., 0., 0.,]

    def __init__(self):
        super().__init__('result_visualizer')
        self.declare_parameter('im_show', True)
        self.get_logger().info('Initializing result_visualizer...')
        self.ori_img = cv2.imread(os.path.join(get_package_share_directory('radar_bringup'), 'resource', 'map.png'))
        self.img_pub = self.create_publisher(Image, 'result_image', 10)
        self.target_sub = self.create_subscription(
            MatchResult, 'matcher/match_result', self.target_callback, 10)
        self.color_sub = self.create_subscription(
            Bool, 'judge/color', self.team_color_callback, 10)
        self.mark_data_sub = self.create_subscription(
            RadarMarkData, 'judge/radar_mark_data', self.mark_data_callback, 10)
        self.get_logger().info('Initialized result_visualizer.')

    def team_color_callback(self, msg: Bool):
        self.team_color = msg.data

    def mark_data_callback(self, msg: RadarMarkData):
        self.mark = msg.mark_progress

    def target_callback(self, msg: MatchResult):
        now_img = self.ori_img.copy()

        if self.team_color:
            cv2.putText(now_img, 'RED', (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 4)
        else:
            cv2.putText(now_img, 'BLUE', (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 0), 4)

        def draw(is_red: bool, num: int, target: MatchedTarget):
            if target.id == -1:
                return
            try:
                im_x = int(
                    target.position[0] / configs['real_width'] * self.ori_img.shape[1])
                im_y = int(
                    (1 - target.position[1] / configs['real_height']) * self.ori_img.shape[0])
                color = (0, 0, 255) if is_red else (255, 0, 0)
                dimmed_color = (color[0] // 2, color[1] // 2, color[2] // 2)
                cv2.circle(now_img, (im_x, im_y), 20, color, -1)
                cv2.circle(now_img, (im_x, im_y), 20, dimmed_color, 4)
                cv2.putText(now_img, str(num), (im_x -10, im_y +10),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                if is_red and not self.team_color:
                    cv2.putText(now_img, f"{self.mark[i]}/120", (im_x - 20, im_y + 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                if not is_red and self.team_color:
                    cv2.putText(now_img, f"{self.mark[i]}/120", (im_x - 20, im_y + 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            except IndexError as e:
                self.get_logger().error(f"Error in drawing: {e}")

        for i, target in enumerate(msg.red):
            draw(True, i, target)
        for i, target in enumerate(msg.blue):
            draw(False, i, target)

        img = Image()
        img.height = now_img.shape[0]
        img.width = now_img.shape[1]
        img.encoding = 'bgr8'
        img.is_bigendian = False
        img.step = now_img.shape[1] * 3
        img.data = now_img.tobytes()
        self.img_pub.publish(img)
        if self.get_parameter('im_show').value:
            cv2.imshow('result', now_img)
            cv2.waitKey(1)


def main():
    rclpy.init()
    result_visualizer = ResultVisualizer()
    rclpy.spin(result_visualizer)
    result_visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
