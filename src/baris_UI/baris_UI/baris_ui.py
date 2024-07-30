import sys, os
import sqlite3
import rclpy
import time
from PyQt5 import uic
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton, QTableWidget, QTableWidgetItem
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from message.srv import RobotService
from baris_UI.RobotServiceClient import RobotServiceClient
from threading import Thread



# ui 폴더 경로 설정
ui_folder = os.path.dirname(os.path.abspath(__file__))

# UI 파일 경로 설정
# baris_ui_path = os.path.join(ui_folder, "ui", "baris.ui")
baris_ui_file = os.path.join(get_package_share_directory("baris_UI"), "ui", "baris_drip_academy.ui")
# print(baris_ui_file)
baris_ui = uic.loadUiType(baris_ui_file)[0]

class MainWindow(QMainWindow, baris_ui):
    def __init__(self, control_node):
        super().__init__()
        self.setupUi(self)
        self.control_node = control_node

        self.setWindowTitle("SQLite3 Data Viewer")

        self.showCommandBtn.clicked.connect(self.load_data)
        self.sendServiceRequestBtn.clicked.connect(self.send_service_request)

    def load_data(self):
        # SQLite3 데이터베이스 연결
        connection = sqlite3.connect("/home/joe/xyz/baris_project/src/baris_db/baris_db.db")
        cursor = connection.cursor()

        # 데이터베이스에서 모든 데이터 가져오기
        cursor.execute("SELECT * FROM BarisCommand")
        rows = cursor.fetchall()
        self.rows = rows.copy()
        # print(rows)

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
        
    def send_service_request(self):
        current_stage = 0
        while current_stage < len(self.rows):
            response = self.control_node.send_request(
                str(self.rows[current_stage][0]),
                str(self.rows[current_stage][1]),
                str(self.rows[current_stage][2]),
                str(self.rows[current_stage][3]),
                str(self.rows[current_stage][4]),
                str(self.rows[current_stage][5]),
                str(self.rows[current_stage][6])
            )
            print(response)
            current_stage += 1
            time.sleep(1)  # Optional: sleep between requests
        
        
def main():
    rclpy.init(args=None)
    ros_node = RobotServiceClient()  # Initialize the RegisterService node
    
    app = QApplication(sys.argv)
    main_window = MainWindow(ros_node)
    main_window.show()

    # Spin the ROS node in a separate thread
    ros_thread = Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    app.exec_()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
