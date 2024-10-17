import asyncio
from typing import Callable
from .status_button import StatusButton, QVBoxLayout

import rclpy
import rclpy.qos
from rclpy.node import Node
from radar_interface.msg import MatchResult
from sensor_msgs.msg import Image


class TargetMatcher:
    def __init__(self, node: Node) -> None:
        self.node = node
        self.button_ = StatusButton("target_matcher", lambda: asyncio.ensure_future(self.create_sub()))


    async def create_sub(self) -> None:
        self.node.get_logger().info(f'create target_matcher subscribers')
        self.button_.set_button_disable()
        self.button_.set_text(f"请稍后，检测正在进行中......")
        self.status = True

        self.enter_match_result = False
        self.match_result_sub = self.node.create_subscription(
            topic='matcher/match_result',
            msg_type=MatchResult,
            callback=self.marker_callback,
            qos_profile=rclpy.qos.qos_profile_system_default)
        
        self.enter_image = False
        self.image_sub = self.node.create_subscription(
            topic='matcher/visualization',
            msg_type=Image,
            callback=self.image_callback,
            qos_profile=rclpy.qos.qos_profile_system_default)
        
        await self.wait_for_result()
        self.button_.set_button_enable()

    def marker_callback(self, msg) -> None:
        self.enter_match_result = True
        if msg:
            return
        else:
            self.status = False
            self.button_.set_text(f'Failed: matcher/target_matcher 没有消息')
            
    def image_callback(self, msg) -> None:
        self.enter_image = True
        if msg:
            return
        else:
            self.status = False
            self.button_.set_text(f'Failed: matcher/visualization 没有消息')

    async def wait_for_result(self) -> None:
        await asyncio.sleep(10)
        if not (self.enter_match_result and self.enter_image):
            self.button_.set_text(f"回调函数没有被触发，请检查")
            self.status = False
        else:
            self.button_.set_text(f"Success: target_matcher 功能完好")
        self.set_status()

    def set_status(self) -> None:
        self.button_.set_status(self.status)

    def get_layout(self) -> QVBoxLayout:
        return self.button_.get_layout()
    
