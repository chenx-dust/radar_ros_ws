from PyQt5.QtWidgets import QApplication
import rclpy.logging
from rclpy.node import Node
from qasync import QEventLoop
from PyQt5.QtCore import QThread

import rclpy
import asyncio
import signal

from .main_window import MainWindow


class Supervisor(Node):
    def __init__(self):
        super().__init__('radar_supervisor')

    def ui_init(self):
        self.get_logger().info('Initializing radar_supervisor...')

        self.app = QApplication([])

        loop = QEventLoop(self.app)
        asyncio.set_event_loop(loop)
        

        with loop:
            self.window = MainWindow(node=self)
            self.window.setWindowTitle("雷达面板")
            self.window.setMinimumSize(500, 900)
            self.window.show()
            # 连接 SIGINT 信号到 mainwindow_shutdown
            loop.add_signal_handler(signal.SIGINT, self.mainwindow_shutdown)

            loop.run_forever()
            
        
        self.get_logger().info('Closed radar_supervisor.')

    def mainwindow_shutdown(self):
        self.window.close()
        self.get_logger().info('Shutting down...')
        self.app.quit()
        rclpy.shutdown()
        
class SpinThread(QThread):
    def __init__(self, node):
        self.node = node
        super().__init__()

    def run(self):
        rclpy.spin(self.node)
        self.node.destroy_node()
        rclpy.shutdown()



def main():
    rclpy.init()
    radar_supervisor = Supervisor()
    thread = SpinThread(radar_supervisor)
    thread.start()
    radar_supervisor.ui_init()
    thread.wait()


if __name__ == '__main__':
    main()