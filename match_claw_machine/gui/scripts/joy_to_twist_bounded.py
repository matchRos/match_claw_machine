#!/usr/bin/env python3
import rospy, math
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Header

class JoyToTwistBounded:
    def __init__(self):
        # Topics
        self.joy_topic  = rospy.get_param("~joy_topic", "/joy")
        self.pose_topic = rospy.get_param("~pose_topic", "/mur620b/UR10_r/ur_calibrated_pose")
        self.cmd_topic  = rospy.get_param("~cmd_topic", "/mur620b/UR10_r/twist_controller/command_collision_free")

        # Axes/Buttons
        self.axis_x     = rospy.get_param("~axis_x", 0)   # z.B. Left stick horizontal
        self.axis_y     = rospy.get_param("~axis_y", 1)   # z.B. Left stick vertical
        self.enable_btn = rospy.get_param("~enable_button", -1)  # -1 = immer aktiv; sonst Button-ID

        # Scaling / Limits
        self.deadzone   = rospy.get_param("~deadzone", 0.08)
        self.v_max      = rospy.get_param("~v_max", 0.15)        # m/s
        self.a_max      = rospy.get_param("~a_max", 0.40)        # m/s^2 (pro Achse)
        self.rate_hz    = rospy.get_param("~rate_hz", 100.0)

        # Workspace bounds (in Roboterbasis / frame des PoseTopics)
        self.x_min = rospy.get_param("~x_min", 0.30)
        self.x_max = rospy.get_param("~x_max", 0.90)
        self.y_min = rospy.get_param("~y_min", -0.40)
        self.y_max = rospy.get_param("~y_max",  0.40)

        # Internals
        self.last_pose = None
        self.last_joy  = None
        self.v_prev    = [0.0, 0.0]  # x,y

        # ROS I/O
        self.pub  = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        self.subj = rospy.Subscriber(self.joy_topic, Joy, self.cb_joy, queue_size=20)
        self.subp = rospy.Subscriber(self.pose_topic, PoseStamped, self.cb_pose, queue_size=20)

    def cb_pose(self, msg: PoseStamped):
        self.last_pose = msg.pose

    def cb_joy(self, msg: Joy):
        self.last_joy = msg

    def _apply_deadzone(self, val):
        return 0.0 if abs(val) < self.deadzone else val

    def _accel_limit(self, v_des, dt):
        # pro Achse begrenzen
        out = [0.0, 0.0]
        for i in (0,1):
            dv = v_des[i] - self.v_prev[i]
            max_step = self.a_max * dt
            if abs(dv) > max_step:
                dv = max_step if dv > 0 else -max_step
            out[i] = self.v_prev[i] + dv
        self.v_prev = out
        return out

    def _enforce_bounds(self, v_xy):
        """Wenn am Rand: verbiete Geschwindigkeit in Richtung 'weiter raus'."""
        if self.last_pose is None:
            return [0.0, 0.0]  # ohne Pose sicherheits-halber stehen

        x = self.last_pose.position.x
        y = self.last_pose.position.y
        vx, vy = v_xy

        # Hart stoppen an den Grenzen
        if x >= self.x_max and vx > 0: vx = 0.0
        if x <= self.x_min and vx < 0: vx = 0.0
        if y >= self.y_max and vy > 0: vy = 0.0
        if y <= self.y_min and vy < 0: vy = 0.0

        return [vx, vy]

    def step(self, dt):
        # Keine Joy-Daten → nichts senden
        if self.last_joy is None:
            self._publish_twist(0.0, 0.0)
            return
        
        # Enable-Button (optional)
        if self.enable_btn >= 0:
            pressed = (self.enable_btn < len(self.last_joy.buttons) and self.last_joy.buttons[self.enable_btn] == 1)
            if not pressed:
                self.v_prev = [0.0, 0.0]
                self._publish_twist(0.0, 0.0)
                return

        # Axes lesen
        ax = self.last_joy.axes[self.axis_x] if self.axis_x < len(self.last_joy.axes) else 0.0
        ay = self.last_joy.axes[self.axis_y] if self.axis_y < len(self.last_joy.axes) else 0.0

        ax = self._apply_deadzone(ax)
        ay = self._apply_deadzone(ay)

        # Mapping Joystick -> gewünschte v (XY)
        v_des = [ax * self.v_max, ay * self.v_max]

        # Beschleunigungsbegrenzung
        v_cmd = self._accel_limit(v_des, dt)

        # Grenzraum erzwingen
        v_cmd = self._enforce_bounds(v_cmd)
        
        print(f"Joy: ({ax:.2f}, {ay:.2f}) -> v_cmd: ({v_cmd[0]:.3f}, {v_cmd[1]:.3f})")

        self._publish_twist(v_cmd[0], v_cmd[1])

    def _publish_twist(self, vx, vy):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = 0.0
        msg.angular.x = msg.angular.y = msg.angular.z = 0.0
        self.pub.publish(msg)

def main():
    rospy.init_node("joystick_to_twist_bounded", anonymous=True)
    node = JoyToTwistBounded()
    rate = rospy.Rate(node.rate_hz)
    last = rospy.Time.now()
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        dt = max(1e-3, (now - last).to_sec())
        last = now
        node.step(dt)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
