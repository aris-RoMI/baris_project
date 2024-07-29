import sys, os
import sqlite3
from PyQt5 import uic
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton, QTableWidget, QTableWidgetItem
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

# ui 폴더 경로 설정
ui_folder = os.path.dirname(os.path.abspath(__file__))

# UI 파일 경로 설정
baris_ui_path = os.path.join(ui_folder, "ui", "baris.ui")


admin_ui = uic.loadUiType(baris_ui_path)[0]
class MainWindow(QMainWindow, admin_ui):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.setWindowTitle("SQLite3 Data Viewer")
        self.setGeometry(100, 100, 800, 600)

        self.showCommandBtn.clicked.connect(self.load_data)

    def load_data(self):
        # SQLite3 데이터베이스 연결
        connection = sqlite3.connect("/home/joe/xyz/baris_project/src/baris_db/baris_db.db")
        cursor = connection.cursor()

        # 데이터베이스에서 모든 데이터 가져오기
        cursor.execute("SELECT * FROM BarisCommand")
        rows = cursor.fetchall()
        print(rows)

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

if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())
