from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QGroupBox, QCheckBox, QLabel
from PyQt5.QtCore import Qt
from ros_interface import launch_drivers, move_to_start_pose

class ROSGui(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Minimal ROS Control")
        self.setGeometry(200, 200, 520, 320)
        self.workspace_name = "catkin_ws_recker"  # wie in deinen Dateien

        root = QVBoxLayout()

        # Auswahl: Roboter
        rob_group = QGroupBox("Robots")
        rob_layout = QVBoxLayout()
        self.robots = {
            "mur620a": QCheckBox("mur620a"),
            "mur620b": QCheckBox("mur620b"),
            "mur620c": QCheckBox("mur620c"),
            "mur620d": QCheckBox("mur620d"),
        }
        # default: a/b an
        self.robots["mur620a"].setChecked(True)
        self.robots["mur620b"].setChecked(True)
        for cb in self.robots.values():
            rob_layout.addWidget(cb)
        rob_group.setLayout(rob_layout)

        # Auswahl: URs
        ur_group = QGroupBox("UR arms")
        ur_layout = QVBoxLayout()
        self.ur10_l = QCheckBox("UR10_l")
        self.ur10_r = QCheckBox("UR10_r")
        self.ur10_l.setChecked(True)
        self.ur10_r.setChecked(True)
        ur_layout.addWidget(self.ur10_l)
        ur_layout.addWidget(self.ur10_r)
        ur_group.setLayout(ur_layout)

        sel_row = QHBoxLayout()
        sel_row.addWidget(rob_group)
        sel_row.addWidget(ur_group)
        root.addLayout(sel_row)

        # Buttons
        btn_row = QHBoxLayout()
        btn_launch = QPushButton("Launch Drivers")
        btn_launch.clicked.connect(lambda: launch_drivers(self))
        btn_move = QPushButton("Move to Start Pose")
        btn_move.clicked.connect(lambda: move_to_start_pose(self))
        btn_row.addWidget(btn_launch)
        btn_row.addWidget(btn_move)
        root.addLayout(btn_row)

        # Status-Hinweis
        note = QLabel("Startet Treiber via SSH und ruft Move-Launches je Robot/UR auf.")
        note.setAlignment(Qt.AlignCenter)
        root.addWidget(note)

        self.setLayout(root)

    def get_selected_robots(self):
        return [name for name, cb in self.robots.items() if cb.isChecked()]

    def get_selected_urs(self):
        out = []
        if self.ur10_l.isChecked():
            out.append("UR10_l")
        if self.ur10_r.isChecked():
            out.append("UR10_r")
        return out
