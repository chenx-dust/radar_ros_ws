import asyncio
from typing import Callable
from .status_button import StatusButton, QVBoxLayout

import rclpy
import rclpy.qos
from rclpy.node import Node
from radar_interface.srv import Detect


class NnDetector:
    def __init__(self, node: Node) -> None:
        self.node = node
        self.button_ = StatusButton("nn_detector", lambda: asyncio.ensure_future(self.create_cli()))


    async def create_cli(self) -> None:
        self.node.get_logger().info(f'create detector client')
        self.button_.set_button_disable()
        self.button_.set_text(f"请稍后，检测正在进行中......")     
        self.status = True

        self.enter_detector = False
        self.detector_client = self.node.create_client(
            srv_name='detect_armor',
            srv_type=Detect,
            qos_profile=rclpy.qos.qos_profile_system_default)
        
        await asyncio.sleep(5)
        await self.detector_callback()

        if not self.enter_detector:
            self.status = False
            self.button_.set_text(f"回调函数没有被触发，请检查")
        else:
            self.button_.set_text(f"Success: nn_detector 功能完好")
        self.set_status()
        self.button_.set_button_enable()

    async def detector_callback(self) -> None:
        if not self.detector_client.wait_for_service(timeout_sec=10):
            self.enter_detector = False
            self.node.get_logger().error(f'detector recive no messages')
            self.button_.set_text(f'Failed: detect_armor 没有消息')
        else:
            self.enter_detector = True
            self.node.get_logger().info(f'nn_detector is ready')

    def set_status(self) -> None:
        self.button_.set_status(self.status)

    def get_layout(self) -> QVBoxLayout:
        return self.button_.get_layout()
    
