#!/usr/bin/env python3
import rospy, math
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger, TriggerRequest

class JoyToTwistBounded:
    def __init__(self):
        # Topics
        self.joy_topic  = rospy.get_param("~joy_topic", "/joy")
        self.pose_topic = rospy.get_param("~pose_topic", "/mur620b/UR10_r/ur_calibrated_pose")
        self.cmd_topic  = rospy.get_param("~cmd_topic",  "/mur620b/UR10_r/twist_controller/command_collision_free")

        # Axes/Buttons
        self.axis_x     = rospy.get_param("~axis_x", 0)
        self.axis_y     = rospy.get_param("~axis_y", 1)
        self.enable_btn = rospy.get_param("~enable_button", -1)    # -1: immer aktiv
        self.enter_btn  = rospy.get_param("~enter_button", 5)      # ENTER-Button-Index vom keyboard_to_joy

        # XY Dynamik / Grenzen
        self.deadzone   = rospy.get_param("~deadzone", 0.08)
        self.v_max      = rospy.get_param("~v_max", 0.15)          # m/s (XY)
        self.a_max      = rospy.get_param("~a_max", 0.40)          # m/s² (XY)
        self.rate_hz    = rospy.get_param("~rate_hz", 100.0)
        self.border_soft_zone = rospy.get_param("~border_soft_zone", 0.03)

        self.x_min = rospy.get_param("~x_min", 0.30)
        self.x_max = rospy.get_param("~x_max", 0.90)
        self.y_min = rospy.get_param("~y_min", -0.40)
        self.y_max = rospy.get_param("~y_max",  0.40)

        # Z Sequenz-Parameter
        self.z_down_m   = rospy.get_param("~z_down_m", 0.10)       # 10 cm runter
        self.vz_max     = rospy.get_param("~vz_max", 0.10)         # m/s
        self.az_max     = rospy.get_param("~az_max", 0.30)         # m/s²
        self.z_tol      = rospy.get_param("~z_tol", 0.003)         # m
        self.dwell_s    = rospy.get_param("~dwell_after_close_s", 2.0)
        self.close_srv_name = rospy.get_param("~close_gripper_service", "/close_gripper")

        # Internals
        self.last_pose = None
        self.last_joy  = None
        self.v_prev_xy = [0.0, 0.0]
        self.v_prev_z  = 0.0
        self.enter_prev = 0

        self.mode = "xy"            # "xy" | "descend" | "dwell" | "ascend"
        self.z_start = None
        self.z_target = None
        self.dwell_until = rospy.Time(0)
        self.close_called = False

        # ROS I/O
        self.pub  = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        self.subj = rospy.Subscriber(self.joy_topic, Joy, self.cb_joy, queue_size=20)
        self.subp = rospy.Subscriber(self.pose_topic, PoseStamped, self.cb_pose, queue_size=20)

        # Service-Proxy (optional, wird bei Bedarf aufgerufen)
        self.close_srv = rospy.ServiceProxy(self.close_srv_name, Trigger)

    def cb_pose(self, msg: PoseStamped):
        self.last_pose = msg.pose

    def cb_joy(self, msg: Joy):
        self.last_joy = msg

    @staticmethod
    def _apply_deadzone(val, dz):
        return 0.0 if abs(val) < dz else val

    def _accel_limit_xy(self, v_des, dt):
        out = [0.0, 0.0]
        for i in (0,1):
            dv = v_des[i] - self.v_prev_xy[i]
            max_step = self.a_max * dt
            if abs(dv) > max_step:
                dv = math.copysign(max_step, dv)
            out[i] = self.v_prev_xy[i] + dv
        return out

    def _accel_limit_z(self, vz_des, dt):
        dv = vz_des - self.v_prev_z
        max_step = self.az_max * dt
        if abs(dv) > max_step:
            dv = math.copysign(max_step, dv)
        return self.v_prev_z + dv

    def _enforce_bounds(self, v_xy):
        if self.last_pose is None:
            return [0.0, 0.0]
        x = self.last_pose.position.x
        y = self.last_pose.position.y
        vx, vy = v_xy

        def scale_towards(limit_dist, v):
            if limit_dist <= 0.0: return 0.0
            if limit_dist < self.border_soft_zone:
                return v * (limit_dist / self.border_soft_zone)
            return v

        if vx > 0: vx = scale_towards(self.x_max - x, vx)
        elif vx < 0: vx = scale_towards(x - self.x_min, vx)
        if vy > 0: vy = scale_towards(self.y_max - y, vy)
        elif vy < 0: vy = scale_towards(y - self.y_min, vy)
        return [vx, vy]

    def _publish(self, vx, vy, vz):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = vz
        self.pub.publish(msg)

    def _enter_pressed(self):
        if self.last_joy is None: return False
        now_val = 0
        if 0 <= self.enter_btn < len(self.last_joy.buttons):
            now_val = 1 if self.last_joy.buttons[self.enter_btn] == 1 else 0
        rising = (self.enter_prev == 0 and now_val == 1)
        self.enter_prev = now_val
        return rising

    def _start_sequence(self):
        if self.last_pose is None:
            rospy.logwarn("No pose yet; cannot start Z sequence.")
            return
        self.mode = "descend"
        self.z_start = self.last_pose.position.z
        self.z_target = self.z_start - self.z_down_m
        self.v_prev_z = 0.0
        self.v_prev_xy = [0.0, 0.0]
        self.close_called = False
        rospy.loginfo("Starting Z-sequence: descend to %.3f m", self.z_target)

    def step(self, dt):
        # ENTER-Start
        if self.mode == "xy" and self._enter_pressed():
            self._start_sequence()

        # Enable prüfen (nur für XY relevant)
        enable_ok = True
        if self.enable_btn >= 0 and self.last_joy is not None:
            enable_ok = (self.enable_btn < len(self.last_joy.buttons) and self.last_joy.buttons[self.enable_btn] == 1)

        # Betriebsarten
        if self.mode == "xy":
            if self.last_joy is None or not enable_ok:
                self.v_prev_xy = [0.0, 0.0]
                self._publish(0.0, 0.0, 0.0)
                return

            ax = self.last_joy.axes[self.axis_x] if self.axis_x < len(self.last_joy.axes) else 0.0
            ay = self.last_joy.axes[self.axis_y] if self.axis_y < len(self.last_joy.axes) else 0.0

            # (dein aktuelles Snapping; Deadzone bei Bedarf aktivieren)
            ax_adj = 1.0 if ax > 0.05 else (-1.0 if ax < -0.05 else 0.0)
            ay_adj = 1.0 if ay > 0.05 else (-1.0 if ay < -0.05 else 0.0)

            v_des = [ax_adj * self.v_max, ay_adj * self.v_max]
            v_des = self._enforce_bounds(v_des)
            v_cmd = self._accel_limit_xy(v_des, dt)
            self.v_prev_xy = v_cmd[:]
            self._publish(v_cmd[0], v_cmd[1], 0.0)
            return

        # Z-abhängige Modi benötigen Pose
        if self.last_pose is None:
            self.v_prev_z = 0.0
            self._publish(0.0, 0.0, 0.0)
            return

        z = self.last_pose.position.z

        if self.mode == "descend":
            dz = self.z_target - z  # negativ Richtung unten typ. -> Vorzeichen in vz_des
            if abs(dz) <= self.z_tol:
                self.v_prev_z = 0.0
                self._publish(0.0, 0.0, 0.0)
                # Greifer schließen (einmalig)
                if not self.close_called:
                    try:
                        rospy.wait_for_service(self.close_srv_name, timeout=2.0)
                        self.close_srv(TriggerRequest())
                        rospy.loginfo("Close gripper service called.")
                    except Exception as e:
                        rospy.logwarn("Close gripper call failed: %s", e)
                    self.close_called = True
                self.mode = "dwell"
                self.dwell_until = rospy.Time.now() + rospy.Duration.from_sec(self.dwell_s)
                return
            # Soll-vz Richtung Ziel, begrenzt
            dir_sign = 1.0 if dz > 0 else -1.0
            vz_des = dir_sign * self.vz_max
            vz_cmd = self._accel_limit_z(vz_des, dt)
            self.v_prev_z = vz_cmd
            self._publish(0.0, 0.0, vz_cmd)
            return

        if self.mode == "dwell":
            self.v_prev_z = 0.0
            self._publish(0.0, 0.0, 0.0)
            if rospy.Time.now() >= self.dwell_until:
                self.mode = "ascend"
                self.z_target = self.z_start
            return

        if self.mode == "ascend":
            dz = self.z_target - z  # Richtung oben typ. positiv
            if abs(dz) <= self.z_tol:
                self.v_prev_z = 0.0
                self._publish(0.0, 0.0, 0.0)
                self.mode = "xy"
                rospy.loginfo("Z-sequence finished; back to XY control.")
                return
            dir_sign = 1.0 if dz > 0 else -1.0
            vz_des = dir_sign * self.vz_max
            vz_cmd = self._accel_limit_z(vz_des, dt)
            self.v_prev_z = vz_cmd
            self._publish(0.0, 0.0, vz_cmd)
            return

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
