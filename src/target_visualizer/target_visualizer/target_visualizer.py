import os
import rclpy
import rclpy.qos
import numpy as np
from ament_index_python import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Image
from radar_interface.msg import TargetArray, Target

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

class TargetVisualizer(Node):
    def __init__(self):
        super().__init__('target_visualizer')
        self.get_logger().info('Initializing target_visualizer...')
        self.ori_img = cv2.imread(os.path.join(get_package_share_directory('radar_bringup'), 'resource', 'map.png'))
        self.img_pub = self.create_publisher(Image, 'target_image', 10)
        self.target_sub = self.create_subscription(TargetArray, 'pc_detector/targets', self.target_callback, 10)
        self.get_logger().info('Initialized target_visualizer.')

    def target_callback(self, msg):
        now_img = self.ori_img.copy()
        for target in msg.targets:
            im_x = int(target.position[0] / configs['real_width'] * self.ori_img.shape[1])
            im_y = int((1 - target.position[1] / configs['real_height']) * self.ori_img.shape[0])
            cv2.circle(now_img, (im_x, im_y), 5, (0, 0, 255), -1)
            cv2.putText(now_img, str(target.id), (im_x, im_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            # 等马氏距离椭圆
            cov = np.asarray(target.pos_covariance).reshape((2, 2))
            w, v = np.linalg.eig(cov)
            angle = np.arccos(np.dot(v[:, 0], np.array([1, 0])))
            angle = -angle if cov[0, 1] > 0 else angle
            mah_dis = 6. / configs['real_width'] * self.ori_img.shape[1]
            width = mah_dis * np.sqrt(w[0])
            height = mah_dis * np.sqrt(w[1])
            cv2.ellipse(now_img, (im_x, im_y), (int(width), int(height)), angle / np.pi * 180, 0, 360, (0, 255, 0), 2)
            # mah_dis = 20 / configs['real_width'] * self.ori_img.shape[1]
            # width = mah_dis * np.sqrt(w[0])
            # height = mah_dis * np.sqrt(w[1])
            # cv2.ellipse(now_img, (im_x, im_y), (int(width), int(height)), angle / np.pi * 180, 0, 360, (255, 0, 255), 2)
        
        img = Image()
        img.height = now_img.shape[0]
        img.width = now_img.shape[1]
        img.encoding = 'bgr8'
        img.is_bigendian = False
        img.step = now_img.shape[1] * 3
        img.data = now_img.tobytes()
        self.img_pub.publish(img)


def main():
    rclpy.init()
    target_visualizer = TargetVisualizer()
    rclpy.spin(target_visualizer)
    target_visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
