#!/usr/bin/env python3
import rospy, math
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Joy
from ur_msgs.srv import SetIO, SetIORequest
from ur_msgs.msg import IOStates

class JoyToTwistBounded:
    def __init__(self):
        # Topics (Robot spezifisch anpassen)
        self.joy_topic   = rospy.get_param("~joy_topic", "/joy")
        self.pose_topic  = rospy.get_param("~pose_topic", "/mur620b/UR10_l/ur_calibrated_pose")
        self.cmd_topic   = rospy.get_param("~cmd_topic",  "/mur620b/UR10_l/twist_controller/command_collision_free")
        self.io_srv_name = rospy.get_param("~set_io_service", "/mur620b/UR10_l/ur_hardware_interface/set_io")
        self.io_states_topic = rospy.get_param("~io_states_topic", "/mur620b/UR10_l/ur_hardware_interface/io_states")

        # Axes/Buttons
        self.axis_x     = rospy.get_param("~axis_x", 1)
        self.axis_y     = rospy.get_param("~axis_y", 0)
        self.enable_btn = rospy.get_param("~enable_button", -1)    # -1: immer aktiv
        self.enter_btn  = rospy.get_param("~enter_button", 3)      # ENTER-Index (keyboard_to_joy)

        # XY Dynamik / Grenzen (nur im manuellen XY-Modus)
        self.v_max      = rospy.get_param("~v_max", 0.15)          # m/s (XY)
        self.a_max      = rospy.get_param("~a_max", 0.40)          # m/s² (XY)
        self.rate_hz    = rospy.get_param("~rate_hz", 200.0)
        self.border_soft_zone = rospy.get_param("~border_soft_zone", 0.03)
        self.x_min = rospy.get_param("~x_min", 0.30)
        self.x_max = rospy.get_param("~x_max", 0.90)
        self.y_min = rospy.get_param("~y_min", -0.40)
        self.y_max = rospy.get_param("~y_max",  0.40)

        # Z & Auto-Fahrt Parameter
        self.z_down_m   = rospy.get_param("~z_down_m", 0.33)       # 36 cm runter
        self.vz_max     = rospy.get_param("~vz_max", 0.08)         # m/s
        self.az_max     = rospy.get_param("~az_max", 0.40)         # m/s²
        self.j_max      = 10.0     # xy [m/s^3], Beispiel
        self.jz_max     = 5.0      # z  [m/s^3]
        self.pos_tol    = rospy.get_param("~pos_tol", 0.003)       # m (3D)

        # Sequenz: Ziele / Wartezeiten
        self.drop_pos   = rospy.get_param("~drop_pos", [0.70, 0.00, 0.40])  # [x,y,z]
        self.dwell_after_close_s = rospy.get_param("~dwell_after_close_s", 2.0)
        self.dwell_after_open_s  = rospy.get_param("~dwell_after_open_s", 1.0)

        # UR I/O Pins
        self.enable_pin    = rospy.get_param("~enable_pin", 1)      # muss 1 sein zum Bewegen
        self.gripper_pin   = rospy.get_param("~gripper_pin", 0)     # 0=open, 1=close
        self.sensor_di_pin = rospy.get_param("~sensor_di_pin", 0)   # digital input 0

        # Internals
        self.last_pose = None
        self.last_joy  = None
        self.v_prev_xy = [0.0, 0.0]
        self.v_prev_z  = 0.0
        self.v_prev_xyz = [0.0, 0.0, 0.0]
        self.a_prev_xyz = [0.0, 0.0, 0.0]
        self.enter_prev = 0
        self.mode = "xy"            # "xy" | "descend" | "dwell" | "ascend" | "to_drop" | "detect_dwell" | "to_home"
        self.start_pose = None
        self.z_target = None
        self.dwell_until = rospy.Time(0)
        self.Kp, self.Ki, self.Kd = 0.6, 0.02, 0.1
        self.integral = 0
        self.prev_error = 0
        self.dt = 1.0 / self.rate_hz

        # DI 0 Überwachung
        self.di0_state = 0
        self.di0_prev  = 0
        self.detect_window_until = rospy.Time(0)
        self.detect_seen = False

        # ROS I/O
        self.pub  = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        self.subj = rospy.Subscriber(self.joy_topic, Joy, self.cb_joy, queue_size=20)
        self.subp = rospy.Subscriber(self.pose_topic, PoseStamped, self.cb_pose, queue_size=20)
        self.subio= rospy.Subscriber(self.io_states_topic, IOStates, self.cb_io, queue_size=50)

        # Service-Proxy
        rospy.wait_for_service(self.io_srv_name)
        self.set_io = rospy.ServiceProxy(self.io_srv_name, SetIO)

    # ---------- Callbacks ----------
    def cb_pose(self, msg: PoseStamped):
        self.last_pose = msg.pose

    def cb_joy(self, msg: Joy):
        self.last_joy = msg

    def cb_io(self, msg: IOStates):
        # suche digitalen Eingang mit gewünschtem Pin
        val = 0
        for din in msg.digital_in_states:
            if din.pin == self.sensor_di_pin:
                val = 1 if din.state else 0
                break
        # rising edge während Erkennungsfenster?
        if self.di0_prev == 0 and val == 1 and rospy.Time.now() <= self.detect_window_until:
            self.detect_seen = True
        self.di0_prev = val
        self.di0_state = val

    # ---------- Helpers ----------
    def _accel_limit_xy(self, v_des, dt):
        out = [0.0, 0.0]
        for i in (0,1):
            dv = v_des[i] - self.v_prev_xy[i]
            max_step = self.a_max * dt
            if abs(dv) > max_step: dv = math.copysign(max_step, dv)
            out[i] = self.v_prev_xy[i] + dv
        return out
    
    def _accel_jerk_limit_xyz(self, v_des, dt):
        """Jerk- und Beschl.-begrenzter Geschwindigkeitsfilter (C2-stetig)."""
        if dt <= 0.0:
            return list(self.v_prev_xyz)

        out = [0.0, 0.0, 0.0]
        a_limits = [self.a_max, self.a_max, self.az_max]
        j_limits = [self.j_max, self.j_max, self.jz_max]

        for i in (0, 1, 2):
            # gewünschte Beschleunigung, um v_des in einem Schritt zu erreichen
            a_req = (v_des[i] - self.v_prev_xyz[i]) / dt

            # Schritt der Beschleunigung per Ruck limitieren
            da = a_req - self.a_prev_xyz[i]
            max_da = j_limits[i] * dt
            if abs(da) > max_da:
                da = math.copysign(max_da, da)
            a_new = self.a_prev_xyz[i] + da

            # Beschleunigung klemmen
            a_max = a_limits[i]
            if a_new >  a_max: a_new =  a_max
            if a_new < -a_max: a_new = -a_max

            # Integration auf Geschwindigkeit
            v_new = self.v_prev_xyz[i] + a_new * dt

            out[i] = v_new
            self.a_prev_xyz[i] = a_new

        self.v_prev_xyz = out
        return out


    def _accel_limit_z(self, vz_des, dt):
        dv = vz_des - self.v_prev_z
        max_step = self.az_max * dt
        if abs(dv) > max_step: dv = math.copysign(max_step, dv)
        return self.v_prev_z + dv

    def _accel_limit_xyz_auto(self, v_des, dt):
        out = [0.0, 0.0, 0.0]
        a_limits = [self.a_max, self.a_max, self.az_max]
        for i in (0,1,2):
            dv = v_des[i] - self.v_prev_xyz[i]
            max_step = a_limits[i] * dt
            if abs(dv) > max_step: dv = math.copysign(max_step, dv)
            out[i] = self.v_prev_xyz[i] + dv
        return out

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
        msg.linear.x, msg.linear.y, msg.linear.z = vx, vy, vz
        self.pub.publish(msg)

    def _enter_pressed(self):
        if self.last_joy is None: return False
        now_val = 0
        if 0 <= self.enter_btn < len(self.last_joy.buttons):
            now_val = 1 if self.last_joy.buttons[self.enter_btn] == 1 else 0
        rising = (self.enter_prev == 0 and now_val == 1)
        self.enter_prev = now_val
        return rising

    # ----- UR I/O -----
    def _set_do(self, pin, state):
        req = SetIORequest()
        req.fun = SetIORequest.FUN_SET_DIGITAL_OUT
        req.pin = pin
        req.state = float(1.0 if state else 0.0)
        return self.set_io(req)

    def _enable_gripper_io(self):
        try:
            self._set_do(self.enable_pin, 1)
        except Exception as e:
            rospy.logwarn("Enable pin failed: %s", e)

    def _gripper_close(self):
        self._enable_gripper_io()
        try:
            self._set_do(self.gripper_pin, 1)  # 1 = geschlossen
        except Exception as e:
            rospy.logwarn("Close gripper DO failed: %s", e)

    def _gripper_open(self):
        self._enable_gripper_io()
        try:
            self._set_do(self.gripper_pin, 0)  # 0 = offen
        except Exception as e:
            rospy.logwarn("Open gripper DO failed: %s", e)

    # ----- Sequence helpers -----
    def _start_sequence(self):
        if self.last_pose is None:
            rospy.logwarn("No pose yet; cannot start sequence.")
            return
        p = self.last_pose.position
        self.start_pose = [p.x, p.y, p.z]
        self.mode = "descend"
        self.z_target = p.z - self.z_down_m
        self.v_prev_xy = [0.0, 0.0]
        self.v_prev_z  = 0.0
        self.v_prev_xyz = [0.0, 0.0, 0.0]
        rospy.loginfo("Sequence start: descend to z=%.3f", self.z_target)

    # ---------- Main ----------
    def step(self, dt):
        # Start per ENTER
        if self.mode == "xy" and self._enter_pressed():
            self._start_sequence()

        # Enable nur für XY-Modus relevant
        enable_ok = True
        if self.enable_btn >= 0 and self.last_joy is not None:
            enable_ok = (self.enable_btn < len(self.last_joy.buttons) and self.last_joy.buttons[self.enable_btn] == 1)

        # XY-Modus (manuell)
        if self.mode == "xy":
            if self.last_joy is None or not enable_ok:
                self.v_prev_xy = [0.0, 0.0]
                self._publish(0.0, 0.0, 0.0)
                return

            ax = -self.last_joy.axes[self.axis_x] if self.axis_x < len(self.last_joy.axes) else 0.0
            ay = -self.last_joy.axes[self.axis_y] if self.axis_y < len(self.last_joy.axes) else 0.0
            ax_adj = 1.0 if ax > 0.05 else (-1.0 if ax < -0.05 else 0.0)
            ay_adj = 1.0 if ay > 0.05 else (-1.0 if ay < -0.05 else 0.0)

            v_des = [ax_adj * self.v_max, ay_adj * self.v_max]
            v_des = self._enforce_bounds(v_des)
            v_des = [v_des[0], v_des[1], 0.0]
            v_cmd = self._accel_jerk_limit_xyz(v_des, dt)
            self.v_prev_xy = v_cmd[:]
            self._publish(v_cmd[0], v_cmd[1], 0.0)
            return

        # Für Automatik-Modi Pose nötig
        if self.last_pose is None:
            self.v_prev_z = 0.0
            self.v_prev_xyz = [0.0, 0.0, 0.0]
            self._publish(0.0, 0.0, 0.0)
            return

        px, py, pz = self.last_pose.position.x, self.last_pose.position.y, self.last_pose.position.z

        # --- DESCEND: 20 cm runter ---
        if self.mode == "descend":
            dz = self.z_target - pz
            if abs(dz) <= self.pos_tol:
                self.v_prev_z = 0.0
                self._publish(0.0, 0.0, 0.0)
                rospy.sleep(0.1)  # kurz warten
                rospy.loginfo("Descend done -> closing gripper.")
                # Greifer schließen
                self._gripper_close()
                # Wartezeit nach Close
                self.mode = "dwell"
                self.dwell_until = rospy.Time.now() + rospy.Duration.from_sec(self.dwell_after_close_s)
                return
            vz_des = self.update(dz) 
            vz_cmd = self._accel_limit_z(vz_des, dt)
            self.v_prev_z = vz_cmd
            self._publish(0.0, 0.0, vz_cmd)
            return

        # --- DWELL nach Close ---
        if self.mode == "dwell":
            self.v_prev_z = 0.0
            self._publish(0.0, 0.0, 0.0)
            if rospy.Time.now() >= self.dwell_until:
                self.mode = "ascend"
                self.z_target = self.start_pose[2]
            return

        # --- ASCEND zurück nach oben ---
        if self.mode == "ascend":
            dz = self.z_target - pz
            if abs(dz) <= self.pos_tol:
                self.v_prev_z = 0.0
                self._publish(0.0, 0.0, 0.0)
                # weiter zur Ablageposition
                self.mode = "to_drop"
                rospy.loginfo("Ascend done -> moving to drop position.")
                return
            vz_des = (self.vz_max if dz > 0 else -self.vz_max)
            vz_cmd = self._accel_limit_z(vz_des, dt)
            self.v_prev_z = vz_cmd
            self._publish(0.0, 0.0, vz_cmd)
            return

        # --- Fahrt zur Ablageposition (3D) ---
        if self.mode == "to_drop":
            tgt = self.drop_pos
            e = [tgt[0]-px, tgt[1]-py, tgt[2]-pz]
            dist = math.sqrt(e[0]**2 + e[1]**2 + e[2]**2)
            if dist <= self.pos_tol:
                self.v_prev_xyz = [0.0, 0.0, 0.0]
                self._publish(0.0, 0.0, 0.0)
                # Greifer öffnen + Erkennungsfenster starten
                self._gripper_open()
                self.detect_seen = False
                self.detect_window_until = rospy.Time.now() + rospy.Duration.from_sec(2.0)  # 2s Fenster
                # Dwell (mind. dwell_after_open_s, überlappt mit Detect)
                base = rospy.Time.now() + rospy.Duration.from_sec(self.dwell_after_open_s)
                self.dwell_until = max(base, self.detect_window_until)
                self.mode = "detect_dwell"
                return
            v_des = [
                self.v_max * (e[0]/dist) if dist > 1e-6 else 0.0,
                self.v_max * (e[1]/dist) if dist > 1e-6 else 0.0,
                self.vz_max * (e[2]/dist) if dist > 1e-6 else 0.0,
            ]
            v_cmd = self._accel_limit_xyz_auto(v_des, dt)
            self.v_prev_xyz = v_cmd[:]
            self._publish(v_cmd[0], v_cmd[1], v_cmd[2])
            return

        # --- Detektions-Dwell: DI0 überwachen ---
        if self.mode == "detect_dwell":
            self.v_prev_xyz = [0.0, 0.0, 0.0]
            self._publish(0.0, 0.0, 0.0)
            if rospy.Time.now() >= self.dwell_until:
                if self.detect_seen:
                    rospy.loginfo("Objekt erkannt (DI%d ging HIGH).", self.sensor_di_pin)
                else:
                    rospy.loginfo("Kein Objekt erkannt (DI%d blieb LOW).", self.sensor_di_pin)
                self.mode = "to_home"
            return

        # --- Zurück zur Ausgangsposition ---
        if self.mode == "to_home":
            tgt = self.start_pose
            e = [tgt[0]-px, tgt[1]-py, tgt[2]-pz]
            dist = math.sqrt(e[0]**2 + e[1]**2 + e[2]**2)
            if dist <= self.pos_tol:
                self.v_prev_xyz = [0.0, 0.0, 0.0]
                self._publish(0.0, 0.0, 0.0)
                self.mode = "xy"
                rospy.loginfo("Sequence finished -> back to XY control.")
                return
            v_des = [
                self.v_max * (e[0]/dist) if dist > 1e-6 else 0.0,
                self.v_max * (e[1]/dist) if dist > 1e-6 else 0.0,
                self.vz_max * (e[2]/dist) if dist > 1e-6 else 0.0,
            ]
            v_cmd = self._accel_limit_xyz_auto(v_des, dt)
            self.v_prev_xyz = v_cmd[:]
            self._publish(v_cmd[0], v_cmd[1], v_cmd[2])
            return
        
    def update(self, error):
        print("Error:", error)
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        return self.Kp*error + self.Ki*self.integral + self.Kd*derivative

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
