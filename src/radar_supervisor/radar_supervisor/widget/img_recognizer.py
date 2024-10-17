import asyncio
from typing import Callable

import rclpy.node
from .status_button import StatusButton, QVBoxLayout

import rclpy
import rclpy.qos
from rclpy.node import Node
from foxglove_msgs.msg import ImageMarkerArray, ImageAnnotations
from radar_interface.msg import DetectedTargetArray


class ImgRecognizer:
    def __init__(self, node: Node) -> None:
        self.node = node
        self.button_ = StatusButton("img_recognizer", lambda: asyncio.ensure_future(self.create_sub()))


    async def create_sub(self) -> None:
        self.node.get_logger().info(f'create img_recognizer subscribers')
        self.button_.set_button_disable()
        self.button_.set_text(f"请稍后，检测正在进行中......")
        self.status = True

        self.enter_marker = False
        self.markers_sub = self.node.create_subscription(
            topic='img_recognizer/markers',
            msg_type=ImageMarkerArray,
            callback=self.marker_callback,
            qos_profile=rclpy.qos.qos_profile_system_default)
        
        self.enter_annotation = False
        self.annotations_sub = self.node.create_subscription(
            topic='img_recognizer/annotations',
            msg_type=ImageAnnotations,
            callback=self.annotations_callback,
            qos_profile=rclpy.qos.qos_profile_system_default)
        
        self.enter_detection = False
        self.detected_targets_sub = self.node.create_subscription(
            topic='img_recognizer/detected_targets',
            msg_type=DetectedTargetArray,
            callback=self.detection_callback,
            qos_profile=rclpy.qos.qos_profile_system_default)
        
        await self.wait_for_result()
        self.button_.set_button_enable()

    def marker_callback(self, msg) -> None:
        self.enter_marker = True
        if msg:
            return
        else:
            self.status = False
            self.button_.set_text(f'Failed: img_recognizer/markers 没有消息')

    def annotations_callback(self, msg) -> None:
        self.enter_annotation = True
        if msg:
            return
        else:
            self.status = False
            self.button_.set_text(f'Failed: img_recognizer/annotations 没有消息')

    def detection_callback(self, msg) -> None:
        self.enter_detection = True
        if msg:
            return
        else:
            self.status = False
            self.button_.set_text(f'Failed: img_recognizer/detected_targets 没有消息')

    async def wait_for_result(self) -> None:
        await asyncio.sleep(10)
        if not (self.enter_marker and self.enter_annotation and self.enter_detection):
            self.button_.set_text(f"回调函数没有被触发，请检查")
            self.status = False
        else:
            self.button_.set_text(f"Success: img_recognizer 功能完好")
        self.set_status()

    def set_status(self) -> None:
        self.button_.set_status(self.status)

    def get_layout(self) -> QVBoxLayout:
        return self.button_.get_layout()
    
