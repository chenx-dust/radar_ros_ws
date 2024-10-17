import rclpy
import asyncio
import rclpy.publisher
import rclpy.qos
import std_msgs.msg._bool
from rclpy.node import Node
from ..widget.status_button import StatusButton
from PyQt5.QtWidgets import QPushButton, QHBoxLayout, QLabel, QVBoxLayout
from PyQt5.QtWidgets import QSpacerItem, QSizePolicy
from PyQt5.QtCore import QRect, Qt

def change_status(button: StatusButton):
    button.set_status(True)
    
class Publish_color:
    def __init__(self, node: Node) -> None:
        self.node = node
        self.button_red = QPushButton("红方")
        self.button_blue = QPushButton("蓝方")
        self.label = QLabel(f"手动发送我方颜色： ")
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        sizePolicy = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.button_red.sizePolicy().hasHeightForWidth())
        self.button_red.setSizePolicy(sizePolicy)
        sizePolicy.setHeightForWidth(self.button_blue.sizePolicy().hasHeightForWidth())
        self.button_blue.setSizePolicy(sizePolicy)

        layout_ = QHBoxLayout()
        layout_.addWidget(self.button_red)
        layout_.addWidget(self.button_blue)

        self.layout = QVBoxLayout()
        self.layout.addWidget(self.label)
        self.layout.addLayout(layout_)

        self.layout.setContentsMargins(20, 15, 20, 15)

        self.button_red.clicked.connect(lambda: self.publish_color(True))
        self.button_blue.clicked.connect(lambda: self.publish_color(False))

        self.publisher = self.node.create_publisher(
            msg_type=std_msgs.msg.Bool,
            topic='judge/color',
            qos_profile=rclpy.qos.qos_profile_system_default
        )

    def publish_color(self, color: bool):
        self.node.get_logger().info("-------------Publish color---------------")
        self.button_red.setCheckable(False)
        self.button_red.setEnabled(False)
        self.button_blue.setCheckable(False)
        self.button_blue.setEnabled(False)

        msg_color = std_msgs.msg.Bool()
        msg_color.data = color
        self.publisher.publish(msg_color)
        if color:
            self.label.setText(f"color: red")
        else:
            self.label.setText(f"color: blue")

        self.button_red.setCheckable(True)
        self.button_red.setEnabled(True)
        self.button_blue.setCheckable(True)
        self.button_blue.setEnabled(True)

    def get_layout(self):
        return self.layout
