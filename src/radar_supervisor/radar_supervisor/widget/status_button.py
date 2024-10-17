from PyQt5.QtWidgets import QPushButton, QLabel, QVBoxLayout, QSpacerItem, QSizePolicy
from PyQt5.QtCore import QRect, Qt

from typing import Callable

class StatusButton:
    def __init__(self, name: str, clicked_callback: Callable | None = None, is_self_connect: bool = False) -> None:
        self.layout_ = QVBoxLayout()
        self.layout_.setContentsMargins(20, 15, 20, 15)

        self.button = QPushButton(f"{name}")
        self.button.setCheckable(False)
        self.button.setDisabled(True)
        if clicked_callback:
            self.button.setCheckable(True)
            self.button.setEnabled(True)
            if is_self_connect:
                self.button.clicked.connect(lambda: clicked_callback(self))
            else:
                self.button.clicked.connect(clicked_callback)
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.button.sizePolicy().hasHeightForWidth())
        self.button.setSizePolicy(sizePolicy)

        self.status_list_ = ["❌", "✅"]
        self.status_ = False

        self.label_ = QLabel(f"当前工作状态：{self.status_list_[self.status_]}")
        self.label_.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.text_label_ = QLabel(f"请点击按钮进行相关信息检测")
        self.text_label_.setAlignment(Qt.AlignmentFlag.AlignCenter)

        spacer_item_up = QSpacerItem(10, 10, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Maximum)
        spacer_item_down = QSpacerItem(10, 10, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Maximum)

        # self.layout_.addItem(spacer_item_up)
        self.layout_.addWidget(self.button)
        self.layout_.addWidget(self.label_)
        self.layout_.addWidget(self.text_label_)
        # self.layout_.addItem(spacer_item_down)

    def set_status(self, status: bool) -> None:
        self.status_ = status
        self.label_.setText(f"当前工作状态：{self.status_list_[self.status_]}")

    def get_layout(self) -> QVBoxLayout:
        return self.layout_
    
    def set_text(self, text: str) -> None:
        self.text_label_.setText(text)

    def set_button_disable(self) -> None:
        self.button.setCheckable(False)
        self.button.setDisabled(True)

    def set_button_enable(self) -> None:
        self.button.setCheckable(True)
        self.button.setEnabled(True)
