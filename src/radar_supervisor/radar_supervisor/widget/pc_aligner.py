import asyncio
from typing import Callable
from .status_button import StatusButton, QVBoxLayout

from PyQt5.QtWidgets import QPushButton, QHBoxLayout, QLabel, QVBoxLayout, QLineEdit
from PyQt5.QtWidgets import QSpacerItem, QSizePolicy, QCheckBox
from PyQt5.QtCore import QRect, Qt

import rclpy
import rclpy.qos
from rclpy.node import Node
from radar_interface.srv import Detect
import std_srvs.srv._empty
import radar_interface.srv._auto_align

class pcAligner:
    def __init__(self, node: Node) -> None:
        self.node = node
        self.button_manual_align = QPushButton("手动对齐")
        self.button_align = QPushButton("自动对齐")

        self.label_corr = QLabel("匹配精度: ")
        self.label_corr.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.edit_corr = QLineEdit("5.")
        layout_corr = QHBoxLayout()
        layout_corr.addWidget(self.label_corr)
        layout_corr.addWidget(self.edit_corr)

        self.label_iter = QLabel("迭代次数: ")
        self.label_iter.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.edit_iter = QLineEdit("30")
        layout_iter = QHBoxLayout()
        layout_iter.addWidget(self.label_iter)
        layout_iter.addWidget(self.edit_iter)

        self.checkbox_inherit = QCheckBox("继承上次结果")
        layout_inherit = QHBoxLayout()
        layout_inherit.addWidget(self.checkbox_inherit)

        sizePolicy = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.button_manual_align.sizePolicy().hasHeightForWidth())
        self.button_manual_align.setSizePolicy(sizePolicy)
        sizePolicy.setHeightForWidth(self.button_align.sizePolicy().hasHeightForWidth())
        self.button_align.setSizePolicy(sizePolicy)

        layout_ = QHBoxLayout()
        layout_.addWidget(self.button_manual_align)
        layout_.addWidget(self.button_align)

        self.layout = QVBoxLayout()
        self.layout.addLayout(layout_)
        self.layout.addLayout(layout_corr)
        self.layout.addLayout(layout_iter)
        self.layout.addLayout(layout_inherit)

        self.layout.setContentsMargins(20, 15, 20, 15)

        self.create_cli()

        self.button_manual_align.clicked.connect(lambda: self.manual_align_client_request())
        self.button_align.clicked.connect(lambda: self.align_client_request())

    def get_auto_align_request(self) -> None:
        corr = float(self.edit_corr.text())
        iter = int(self.edit_iter.text())
        self.auto_align_req = radar_interface.srv.AutoAlign.Request()
        self.auto_align_req.max_corr_dist = corr
        self.auto_align_req.max_iteration = iter
        self.auto_align_req.inherit_last = self.checkbox_inherit.isChecked()
    
    def create_cli(self) -> None:
        self.node.get_logger().info(f'create client')   
        self.request = std_srvs.srv.Empty.Request()
        self.manual_align_client = self.node.create_client(
            srv_name='manual_align',
            srv_type=std_srvs.srv.Empty,
            qos_profile=rclpy.qos.qos_profile_system_default)
        self.auto_align_client = self.node.create_client(
            srv_name='auto_align',
            srv_type=radar_interface.srv.AutoAlign,
            qos_profile=rclpy.qos.qos_profile_system_default)
    
    def manual_align_client_request(self) -> None:
        future = self.manual_align_client.call_async(self.request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=0)


    
    def align_client_request(self) -> None:
        self.get_auto_align_request()

        future = self.auto_align_client.call_async(self.auto_align_req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=0)



    def get_layout(self):
        return self.layout
