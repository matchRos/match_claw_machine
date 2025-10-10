from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QGroupBox, QCheckBox, QLabel
from PyQt5.QtCore import Qt
from ros_interface import (
    launch_drivers,
    move_to_initial_pose,
    switch_to_twist_controller,
    switch_to_arm_controller,
    enable_selected_urs,
    start_move_to_true_start,
)

class ROSGui(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Minimal ROS Control")
        self.setGeometry(200, 200, 600, 340)
        self.workspace_name = "catkin_ws_recker"

        root = QVBoxLayout()

        # Robots
        rob_group = QGroupBox("Robots")
        rob_layout = QVBoxLayout()
        self.robots = {
            "mur620a": QCheckBox("mur620a"),
            "mur620b": QCheckBox("mur620b"),
            "mur620c": QCheckBox("mur620c"),
            "mur620d": QCheckBox("mur620d"),
        }
        #self.robots["mur620a"].setChecked(True)
        self.robots["mur620b"].setChecked(True)
        for cb in self.robots.values():
            rob_layout.addWidget(cb)
        rob_group.setLayout(rob_layout)

        # UR arms
        ur_group = QGroupBox("UR arms")
        ur_layout = QVBoxLayout()
        self.ur10_l = QCheckBox("UR10_l")
        self.ur10_r = QCheckBox("UR10_r")
        self.ur10_l.setChecked(True)
        #self.ur10_r.setChecked(True)
        ur_layout.addWidget(self.ur10_l)
        ur_layout.addWidget(self.ur10_r)
        ur_group.setLayout(ur_layout)

        sel_row = QHBoxLayout()
        sel_row.addWidget(rob_group)
        sel_row.addWidget(ur_group)
        root.addLayout(sel_row)

        # Buttons
        row1 = QHBoxLayout()
        btn_launch = QPushButton("Launch Drivers")
        btn_launch.clicked.connect(lambda: launch_drivers(self))
        btn_init = QPushButton("Move to Initial Pose")  # <--- umbenannt
        btn_init.clicked.connect(lambda: move_to_initial_pose(self))
        row1.addWidget(btn_launch)
        row1.addWidget(btn_init)
        root.addLayout(row1)

        row2 = QHBoxLayout()
        btn_twist = QPushButton("Switch to Twist Ctrl")
        btn_twist.clicked.connect(lambda: switch_to_twist_controller(self))
        btn_arm = QPushButton("Switch to Arm Ctrl")
        btn_arm.clicked.connect(lambda: switch_to_arm_controller(self))
        row2.addWidget(btn_twist)
        row2.addWidget(btn_arm)
        root.addLayout(row2)

        row3 = QHBoxLayout()
        btn_enable = QPushButton("Enable UR(s)")
        btn_enable.clicked.connect(lambda: enable_selected_urs(self))
        row3.addWidget(btn_enable)
        root.addLayout(row3)

        row4 = QHBoxLayout()
        btn_true_start = QPushButton("Move to True Start")
        btn_true_start.clicked.connect(lambda: start_move_to_true_start(self))
        row4.addWidget(btn_true_start)
        root.addLayout(row4)


        note = JLabelCentered("Startet Treiber, Initialpose und Controller-Wechsel je Robot/UR.")
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

class JLabelCentered(QLabel):
    def __init__(self, text):
        super().__init__(text)
        self.setAlignment(Qt.AlignCenter)
