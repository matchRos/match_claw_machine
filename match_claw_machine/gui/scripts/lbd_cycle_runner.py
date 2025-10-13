#!/usr/bin/env python3
import rospy, math, random, subprocess
from geometry_msgs.msg import PoseStamped, Twist
from ur_msgs.srv import SetIO, SetIORequest
from ur_msgs.msg import IOStates
from std_msgs.msg import Float32

class LbDCycleRunner:
    def __init__(self):
        # ---------- Topics/Services ----------
        self.pose_topic  = rospy.get_param("~pose_topic", "/mur620b/UR10_l/ur_calibrated_pose")
        self.cmd_topic   = rospy.get_param("~cmd_topic",  "/mur620b/UR10_l/twist_controller/command_collision_free")
        self.io_srv_name = rospy.get_param("~set_io_service", "/mur620b/UR10_l/ur_hardware_interface/set_io")
        self.io_states_topic = rospy.get_param("~io_states_topic", "/mur620b/UR10_l/ur_hardware_interface/io_states")

        # ---------- Workspace / Ziele ----------
        self.image_capture_pos = [0.50, 0.30, 0.70]
        self.drop_pos          = [0.70, 0.00, 0.40]
        self.z_pick            = rospy.get_param("~z_pick", 0.40)   # m im Frame der Pose
        self.x_min = rospy.get_param("~x_min", 0.40)
        self.x_max = rospy.get_param("~x_max", 0.50)
        self.y_min = rospy.get_param("~y_min", -0.20)
        self.y_max = rospy.get_param("~y_max",  0.40)

        # ---------- Dynamik ----------
        self.rate_hz = rospy.get_param("~rate_hz", 100.0)
        self.v_max   = rospy.get_param("~v_max", 0.15)      # XY m/s
        self.vz_max  = rospy.get_param("~vz_max", 0.10)     # Z  m/s
        self.a_max   = rospy.get_param("~a_max", 0.40)      # XY m/s^2
        self.az_max  = rospy.get_param("~az_max", 0.30)     # Z  m/s^2
        self.pos_tol = rospy.get_param("~pos_tol", 0.01)   # 1 mm

        # ---------- Greifer / Erkennung ----------
        self.enable_pin    = rospy.get_param("~enable_pin", 1)    # DO enable
        self.gripper_pin   = rospy.get_param("~gripper_pin", 0)   # DO 0=open, 1=close
        # RG2-Öffnungsweite: bevorzugt Topic (mm), sonst Analog-In (Volt->mm)
        self.rg2_width_topic = rospy.get_param("~rg2_width_topic", "")  # Float32 mm
        self.rg2_ai_pin       = rospy.get_param("~rg2_ai_pin", 0)
        self.rg2_ai_mm_per_volt = rospy.get_param("~rg2_ai_mm_per_volt", 40.0)
        self.rg2_ai_offset_mm    = rospy.get_param("~rg2_ai_offset_mm", 0.0)
        self.empty_thresh_mm = rospy.get_param("~grip_empty_threshold_mm", 2.0)
        self.dwell_after_open_s  = rospy.get_param("~dwell_after_open_s", 1.0)

        # ---------- Capture ----------
        self.capture_script = "~/imitationCV/capture.py"

        # --- PID (Z-Regler) ---
        self.Kp = rospy.get_param("~pid_kp", 0.6)
        self.Ki = rospy.get_param("~pid_ki", 0.06)
        self.Kd = rospy.get_param("~pid_kd", 0.10)
        self.pid_int = [0.0, 0.0, 0.0]
        self.pid_prev = [0.0, 0.0, 0.0]
        self.dt_fixed = 1.0 / self.rate_hz  # nominales dt für den PID (wie in deinem Beispiel)


        # ---------- State ----------
        self.last_pose = None
        self.last_io   = None
        self.v_prev = [0.0, 0.0, 0.0]
        self.mode = "to_image"
        self.rand_xy = [0.0, 0.0]
        self.width_mm = None

        # ---------- ROS I/O ----------
        self.pub  = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        rospy.Subscriber(self.pose_topic, PoseStamped, self.cb_pose, queue_size=20)
        rospy.Subscriber(self.io_states_topic, IOStates, self.cb_io, queue_size=50)
        if self.rg2_width_topic:
            rospy.Subscriber(self.rg2_width_topic, Float32, self.cb_width, queue_size=10)
        rospy.wait_for_service(self.io_srv_name)
        self.set_io = rospy.ServiceProxy(self.io_srv_name, SetIO)

        random.seed()

    # -------- Callbacks --------
    def cb_pose(self, msg): self.last_pose = msg.pose
    def cb_io(self, msg):   self.last_io = msg
    def cb_width(self, m):  self.width_mm = float(m.data)

    # -------- Helpers --------
    def _publish(self, vx, vy, vz):
        t = Twist(); t.linear.x, t.linear.y, t.linear.z = vx, vy, vz; self.pub.publish(t)

    def _accel_limit(self, v_des, dt):
        out = [0.0, 0.0, 0.0]
        a_lim = [self.a_max, self.a_max, self.az_max]
        for i in (0,1,2):
            dv = v_des[i] - self.v_prev[i]
            step = a_lim[i] * dt
            if abs(dv) > step: dv = math.copysign(step, dv)
            out[i] = self.v_prev[i] + dv
        return out

    def _set_do(self, pin, state):
        req = SetIORequest()
        req.fun = SetIORequest.FUN_SET_DIGITAL_OUT
        req.pin = pin
        req.state = 1.0 if state else 0.0
        self.set_io(req)

    def _gripper_enable(self): self._set_do(self.enable_pin, 1)
    def _gripper_open(self):   self._gripper_enable(); self._set_do(self.gripper_pin, 0)
    def _gripper_close(self):  self._gripper_enable(); self._set_do(self.gripper_pin, 1)

    def _read_rg2_width_mm(self):
        if self.rg2_width_topic and self.width_mm is not None:
            return self.width_mm
        if self.last_io:
            for ain in self.last_io.analog_in_states:
                if ain.pin == self.rg2_ai_pin:
                    return self.rg2_ai_mm_per_volt * float(ain.state) + self.rg2_ai_offset_mm
        return None
    
    def update(self, e):
        v = [0.0, 0.0, 0.0]
        limits = [self.v_max, self.v_max, self.vz_max]

        for i in (0, 1, 2):
            self.pid_int[i] += e[i] * self.dt_fixed
            d = (e[i] - self.pid_prev[i]) / self.dt_fixed
            self.pid_prev[i] = e[i]
            v[i] = self.Kp * e[i] + self.Ki * self.pid_int[i] + self.Kd * d
            # Output-Clamp je Achse
            if abs(v[i]) > limits[i]:
                v[i] = math.copysign(limits[i], v[i])
        return v



    # -------- Main Step --------
    def step(self, dt):
        if self.last_pose is None:
            self._publish(0,0,0); return

        px, py, pz = self.last_pose.position.x, self.last_pose.position.y, self.last_pose.position.z

        # 1) Zur ImageCapture-Position
        if self.mode == "to_image":
            tgt = self.image_capture_pos

            e = [tgt[0]-px, tgt[1]-py, tgt[2]-pz]
            print("Error:", e)
            print("Pose:", px, py, pz)

            dist = math.sqrt(e[0]**2+e[1]**2+e[2]**2)
            if dist <= self.pos_tol:
                self.pid_integral = 0.0
                self.pid_prev_error = 0.0
                self.v_prev = [0,0,0]; self._publish(0,0,0)
                try:
                    subprocess.run(["bash","-lc", "~/imitationCV/.venv/bin/python3 "+self.capture_script], check=True)
                except Exception as e:
                    rospy.logwarn("Capture script failed: %s", e)
                # Nächsten XY-Punkt wählen
                self.rand_xy = [random.uniform(self.x_min, self.x_max),
                                random.uniform(self.y_min, self.y_max)]
                self.mode = "to_rand_xy"
                return
            # v_des = [ self.v_max*(e[0]/max(dist,1e-6)),
            #           self.v_max*(e[1]/max(dist,1e-6)),
            #           self.vz_max*(e[2]/max(dist,1e-6)) ]
            # v_cmd = self._accel_limit(v_des, dt); self.v_prev = v_cmd[:]
            v_cmd = self.update(e)                      # <- PID-Ausgabe [m/s]     
            print("Cmd:", v_cmd)   
            self.v_prev = v_cmd[:]

            self._publish(*v_cmd); 
            return


        # 2) Nur XY zur Zufallsposition (Z beibehalten)
        if self.mode == "to_rand_xy":
            tgt = [self.rand_xy[0], self.rand_xy[1], pz]
            e = [tgt[0]-px, tgt[1]-py, 0.0]; dist = math.hypot(e[0], e[1])
            if dist <= self.pos_tol:
                self.v_prev = [0,0,0]; self._publish(0,0,0)
                self.mode = "descend"; return
            v_des = [ self.v_max*(e[0]/max(dist,1e-6)),
                      self.v_max*(e[1]/max(dist,1e-6)),
                      0.0 ]
            v_cmd = self._accel_limit(v_des, dt); self.v_prev = v_cmd[:]
            self._publish(v_cmd[0], v_cmd[1], 0.0); return

        # 3) Gerade runter auf z_pick
        if self.mode == "descend":
            dz = self.z_pick - pz
            if abs(dz) <= self.pos_tol:
                self.v_prev = [0,0,0]; self._publish(0,0,0)
                # Greifer schließen und Erfolg prüfen
                try: self._gripper_close()
                except Exception as e: rospy.logwarn("Close DO failed: %s", e)
                rospy.sleep(0.2)  # kurzes Einklemmen
                width = self._read_rg2_width_mm()
                if width is not None and width < self.empty_thresh_mm:
                    rospy.loginfo("Leer gegriffen (RG2 %.2f mm < %.2f).", width, self.empty_thresh_mm)
                    self.mode = "ascend"   # direkt hoch, kein Drop
                else:
                    self.mode = "ascend_to_drop"
                return
            vz = self.vz_max if dz > 0 else -self.vz_max
            v_cmd = self._accel_limit([0.0,0.0,vz], dt); self.v_prev = v_cmd[:]
            self._publish(0.0, 0.0, v_cmd[2]); return

        # 4) Hoch zurück auf Start-Z (ImageCapture Z)
        if self.mode in ("ascend","ascend_to_drop"):
            z_target = self.image_capture_pos[2]
            dz = z_target - pz
            if abs(dz) <= self.pos_tol:
                self.v_prev = [0,0,0]; self._publish(0,0,0)
                if self.mode == "ascend":
                    # leer: neuer Zyklus
                    self.mode = "to_image"
                else:
                    self.mode = "to_drop"
                return
            vz = self.vz_max if dz > 0 else -self.vz_max
            v_cmd = self._accel_limit([0.0,0.0,vz], dt); self.v_prev = v_cmd[:]
            self._publish(0.0,0.0,v_cmd[2]); return

        # 5) Zur Ablageposition (3D), öffnen, kurz warten, neuer Zyklus
        if self.mode == "to_drop":
            tgt = self.drop_pos
            e = [tgt[0]-px, tgt[1]-py, tgt[2]-pz]; dist = math.sqrt(e[0]**2+e[1]**2+e[2]**2)
            if dist <= self.pos_tol:
                self.v_prev = [0,0,0]; self._publish(0,0,0)
                try: self._gripper_open()
                except Exception as e: rospy.logwarn("Open DO failed: %s", e)
                rospy.sleep(self.dwell_after_open_s)
                self.mode = "to_image"
                return
            v_des = [ self.v_max*(e[0]/max(dist,1e-6)),
                      self.v_max*(e[1]/max(dist,1e-6)),
                      self.vz_max*(e[2]/max(dist,1e-6)) ]
            v_cmd = self._accel_limit(v_des, dt); self.v_prev = v_cmd[:]
            self._publish(*v_cmd); return

def main():
    rospy.init_node("lbd_cycle_runner", anonymous=True)
    node = LbDCycleRunner()
    rate = rospy.Rate(node.rate_hz)
    last = rospy.Time.now()
    while not rospy.is_shutdown():
        now = rospy.Time.now(); dt = max(1e-3, (now - last).to_sec()); last = now
        node.step(dt); rate.sleep()

if __name__ == "__main__":
    try: main()
    except rospy.ROSInterruptException: pass
