from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import QVBoxLayout, QMainWindow, QWidget, QAction
from PyQt5.QtCore import Qt


from .widget.status_button import StatusButton
from .widget.img_recognizer import ImgRecognizer
from .widget.nn_detector import NnDetector
from .widget.target_matcher import TargetMatcher
from .widget.judge_bridge import JudgeBridge
from .widget.pc_aligner import pcAligner
from .utils.click_callback import change_status
from .utils.click_callback import Publish_color

from rclpy.node import Node

class MainWindow(QMainWindow):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.setWindowTitle("My App")

        # self.start_button = StatusButton(name="雷达启动", clicked_callback=change_status, is_self_connect=True)
        color_button = Publish_color(node=self.node)
        info_button = ImgRecognizer(node=self.node)
        detector_button = NnDetector(node=self.node)
        matcher_button = TargetMatcher(node=self.node)
        judge_button = JudgeBridge(node=self.node)
        pc_aligner = pcAligner(node=self.node)

        layout = QVBoxLayout()
        layout.addLayout(color_button.get_layout())
        layout.addLayout(info_button.get_layout())
        layout.addLayout(detector_button.get_layout())
        layout.addLayout(matcher_button.get_layout())
        layout.addLayout(judge_button.get_layout())
        layout.addLayout(pc_aligner.get_layout())

        widget = QWidget()
        widget.setLayout(layout)
    


        # Set the central widget of the Window.
        self.setCentralWidget(widget)


