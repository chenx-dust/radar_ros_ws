import asyncio
from typing import Callable
from .status_button import StatusButton, QVBoxLayout

import rclpy
import rclpy.qos
from rclpy.node import Node
from radar_interface.msg import RadarMarkData, RadarInfo, RadarCmd


class JudgeBridge:
    def __init__(self, node: Node) -> None:
        self.node = node
        self.button_ = StatusButton("judge_bridge", lambda: asyncio.ensure_future(self.create_sub()))


    async def create_sub(self) -> None:
        self.node.get_logger().info(f'create judge_bridge subscribers')
        self.button_.set_button_disable()
        self.button_.set_text(f"请稍后，检测正在进行中......")
        self.status = True

        self.enter_mark_data = False
        self.match_data_sub = self.node.create_subscription(
            topic='judge/radar_mark_data',
            msg_type=RadarMarkData,
            callback=self.marker_callback,
            qos_profile=rclpy.qos.qos_profile_system_default)
        
        self.enter_radar_info = False
        self.radar_info_sub = self.node.create_subscription(
            topic='judge/radar_info',
            msg_type=RadarInfo,
            callback=self.info_callback,
            qos_profile=rclpy.qos.qos_profile_system_default)
        
        self.enter_radar_cmd = False
        self.image_sub = self.node.create_subscription(
            topic='judge/radar_cmd',
            msg_type=RadarCmd,
            callback=self.cmd_callback,
            qos_profile=rclpy.qos.qos_profile_system_default)
        
        await self.wait_for_result()
        self.button_.set_button_enable()

    def marker_callback(self, msg) -> None:
        self.enter_mark_data = True
        if msg:
            return
        else:
            self.status = False
            self.button_.set_text(f'Failed: judge/radar_mark_data 没有消息')
            
    def info_callback(self, msg) -> None:
        self.enter_radar_info = True
        if msg:
            return
        else:
            self.status = False
            self.button_.set_text(f'Failed: judge/radar_info 没有消息')

    def cmd_callback(self, msg) -> None:
        self.enter_radar_cmd = True
        if msg:
            return
        else:
            self.status = False
            self.button_.set_text(f'Failed: judge/radar_cmd 没有消息')

    async def wait_for_result(self) -> None:
        await asyncio.sleep(10)
        if not (self.enter_mark_data and self.enter_radar_info and self.enter_radar_cmd):
            self.button_.set_text(f"回调函数没有被触发，请检查")
            self.status = False
        else:
            self.button_.set_text(f"Success: judge_bridge 功能完好")
        self.set_status()

    def set_status(self) -> None:
        self.button_.set_status(self.status)

    def get_layout(self) -> QVBoxLayout:
        return self.button_.get_layout()
    
