import sys
import os
import sqlite3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from PyQt5 import uic
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QTableWidgetItem
from ament_index_python.packages import get_package_share_directory
from threading import Thread
from datetime import datetime

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from library.Constants import ResponseCode, DispenseCommand
from message.srv import RobotService
from baris_UI.RobotServiceClient import RobotServiceClient
from baris_UI.dispense_service_client import DispenseServiceClient
from baris_UI.robot_status_subscription_node import ROSNodeSignals, RobotStatusSubscription


# ui 폴더 경로 설정
ui_folder = os.path.dirname(os.path.abspath(__file__))

# UI 파일 경로 설정
baris_ui_file = os.path.join(get_package_share_directory("baris_UI"), "ui", "baris_drip_academy.ui")
baris_ui = uic.loadUiType(baris_ui_file)[0]


class MainWindow(QMainWindow, baris_ui, Node):
    def __init__(self, robot_control_node, dispense_control_node):
        super().__init__()
        self.setupUi(self)
        self.robot_control_node = robot_control_node
        self.dispense_control_node = dispense_control_node

        self.signals = ROSNodeSignals()
        self.signals.robot_status_received.connect(self.update_robot_status)

        self.setWindowTitle("SQLite3 Data Viewer")

        self.showCommandBtn.clicked.connect(self.load_data)
        self.sendServiceRequestBtn.clicked.connect(self.start_service_requests)

        self.current_stage = 0
        self.timer = QTimer()
        self.timer.timeout.connect(self.send_service_request)

    def load_data(self):
        # SQLite3 데이터베이스 연결
        connection = sqlite3.connect("/home/joe/xyz/baris_project/src/baris_db/baris_db.db")
        cursor = connection.cursor()

        # 데이터베이스에서 모든 데이터 가져오기
        cursor.execute("SELECT * FROM BarisCommand")
        rows = cursor.fetchall()
        self.rows = rows.copy()

        # 열 제목 가져오기
        cursor.execute("PRAGMA table_info(BarisCommand)")
        columns_info = cursor.fetchall()
        column_names = [column_info[1] for column_info in columns_info]

        # 테이블에 데이터 설정
        self.commandTable.setRowCount(len(rows))
        self.commandTable.setColumnCount(len(column_names))
        self.commandTable.setHorizontalHeaderLabels(column_names)

        for row_idx, row_data in enumerate(rows):
            for col_idx, col_data in enumerate(row_data):
                self.commandTable.setItem(row_idx, col_idx, QTableWidgetItem(str(col_data)))

        # 데이터베이스 연결 종료
        connection.close()

    def start_service_requests(self):
        self.current_stage = 0
        self.timer.start(500)  # Call the service request method every 500 ms

    def send_service_request(self):
        if self.current_stage < len(self.rows):
            if self.rows[self.current_stage][1] == DispenseCommand.COFFEE_ON or self.rows[self.current_stage][1] == DispenseCommand.WATER_TOGGLE:
                response = self.dispense_control_node.send_request(
                    "",
                    "",
                    self.rows[self.current_stage][1],
                )
            else:
                response = self.robot_control_node.send_request(
                    str(self.rows[self.current_stage][0]),
                    str(self.rows[self.current_stage][1]),
                    str(self.rows[self.current_stage][2]),
                    str(self.rows[self.current_stage][3]),
                    str(self.rows[self.current_stage][4]),
                    str(self.rows[self.current_stage][5]),
                    str(self.rows[self.current_stage][6])
                )
            if response.response_cd == ResponseCode.SUCCESS:
                self.current_stage += 1
            else:
                print(f"Error occurred: {response.result}")
        else:
            self.timer.stop()
            print("Coffee DONE")

    def update_robot_status(self, seq_no, node_status, component):
        dt = datetime.strptime(seq_no, "%Y-%m-%d %H:%M:%S.%f")
    
    # Format the datetime object back into a string, including only the date and time up to seconds
        formatted_str = dt.strftime("%Y-%m-%d %H:%M:%S")
        self.currentTimeLabel.setText(formatted_str)
        current_step = node_status + "(" + component[0].status + ")"
        self.currentStepLabel.setText(current_step)
        # print(f"seq_no : {seq_no}")
        # print(f"node_status : {node_status}")
        # print(f"component : {component}")


def main():
    rclpy.init(args=None)

    robot_control_node = RobotServiceClient()  # Initialize the RegisterService node
    dispense_control_node = DispenseServiceClient()

    app = QApplication(sys.argv)
    main_window = MainWindow(robot_control_node, dispense_control_node)
    main_window.show()
    signals = main_window.signals

    robot_status_subscriber = RobotStatusSubscription(signals)

    # Spin the ROS node in a separate thread
    executor = MultiThreadedExecutor()
    executor.add_node(robot_control_node)
    executor.add_node(dispense_control_node)
    executor.add_node(robot_status_subscriber)

    def spin_ros():
        while rclpy.ok():
            rclpy.spin_once(robot_control_node, timeout_sec=0.1)
            rclpy.spin_once(dispense_control_node, timeout_sec=0.1)
            rclpy.spin_once(robot_status_subscriber, timeout_sec=0.1)

    ros_thread = Thread(target=spin_ros)
    ros_thread.start()

    app.exec_()
    rclpy.shutdown()
    ros_thread.join()


if __name__ == "__main__":
    main()
