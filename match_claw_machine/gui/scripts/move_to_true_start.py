#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from math import sqrt
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
import math
import subprocess
import os
import signal
from datetime import datetime
import tf.transformations as tft


class RobotMover:
    def __init__(self):
        rospy.init_node('robot_mover', anonymous=True)

        # Zielposition (Startposition des Roboters)
        self.target_position = rospy.get_param('~start_pos', [0.33263377919966164, 0.0, 0.6763138405651459])  # z.B. [x, y, z]
        self.target_orientation = rospy.get_param('~start_orientation', [1.0, 0.0, 0.0, 0.0])  # Quaternion [x, y, z, w]

        # Publisher für Twist
        self.twist_pub = rospy.Publisher('/mur620b/UR10_l/twist_controller/command_collision_free', Twist, queue_size=10)

        # Subscriber für Pose
        rospy.Subscriber('/mur620b/UR10_l/ur_calibrated_pose', PoseStamped, self.pose_callback)

        self.current_pose = None
        self.Kp, self.Ki, self.Kd = 0.8, 0.08, 0.1
        
        self.integral = 0
        self.prev_error = 0
        self.rate = rospy.Rate(100)  # 10 Hz
        self.dt = 1/100.0  # Zeitintervall basierend auf der Rate

    def pose_callback(self, msg):
        self.current_pose = msg.pose

    def distance_to_target(self):
        if self.current_pose is None:
            return float('inf')

        dx = self.target_position[0] - self.current_pose.position.x
        dy = self.target_position[1] - self.current_pose.position.y
        dz = self.target_position[2] - self.current_pose.position.z
        return sqrt(dx**2 + dy**2 + dz**2)


    def move_to_start_fast(self, tolerance=0.005):

        self.integral = 0
        self.prev_error = 0

        while not rospy.is_shutdown():
            if self.current_pose is None:
                rospy.loginfo("Warte auf aktuelle Pose...")
                self.rate.sleep()
                continue

            distance = self.distance_to_target()

            if distance < tolerance:
                rospy.loginfo("Roboter ist in der Startposition.")
                self.publish_zero_twist()
                break

            # Steuerung per Proportionalregelung
            twist = Twist()
            #twist.linear.x = 0.5 * (self.target_position[0] - self.current_pose.position.x)
            twist.linear.x = self.update(self.target_position[0] - self.current_pose.position.x)
            twist.linear.y = 0.5 * (self.target_position[1] - self.current_pose.position.y)
            twist.linear.z = 0.5 * (self.target_position[2] - self.current_pose.position.z) 

            # Begrenzung der Geschwindigkeit
            max_vel = 0.02  # m/s
            twist.linear.x = max(-max_vel, min(twist.linear.x, max_vel))
            twist.linear.y = max(-max_vel, min(twist.linear.y, max_vel))
            twist.linear.z = max(-max_vel, min(twist.linear.z, max_vel))

            self.twist_pub.publish(twist)
            self.rate.sleep()

    def update(self, error):
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        return self.Kp*error + self.Ki*self.integral + self.Kd*derivative

    def publish_zero_twist(self):
        stop_twist = Twist()
        self.twist_pub.publish(stop_twist)

  
   

if __name__ == '__main__':
    try:
        mover = RobotMover()
        mover.move_to_start_fast()

    except rospy.ROSInterruptException:
        pass
