#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Header

class StartPoseRegulator:
    def __init__(self):
        # --- Parameter ---
        self.pose_topic   = rospy.get_param("~pose_topic", "/mur620x/UR10_x/ur_calibrated_pose")
        self.cmd_topic    = rospy.get_param("~cmd_topic",  "/mur620x/UR10_x/twist_controller/command")
        self.target_xyz   = rospy.get_param("~start_pos",  [0.500, 0.000, 0.300])  # [m]
        self.pos_tol      = rospy.get_param("~pos_tol",    0.003)   # [m] Stopptoleranz
        self.v_max        = rospy.get_param("~v_max",      0.15)    # [m/s] Max. Kart.-Geschwindigkeit
        self.a_max        = rospy.get_param("~a_max",      0.40)    # [m/s^2] Kart.-Beschl.begrenzung (pro Achse via Skalar)
        self.k_tanh       = rospy.get_param("~k_tanh",     3.0)     # Formfaktor für weiches Anfahren/Abbremsen
        self.rate_hz      = rospy.get_param("~rate_hz",    100.0)   # [Hz]
        self.timeout_s    = rospy.get_param("~pose_timeout_s", 0.5) # [s] Pose-Timeout
        self.frame_id     = rospy.get_param("~frame_id",   "base_link")  # für TwistStamped Header

        # --- Runtime ---
        self.last_pose = None
        self.last_pose_stamp = None
        self.v_prev = [0.0, 0.0, 0.0]  # für Beschleunigungsbegrenzung

        # --- ROS I/O ---
        self.pub = rospy.Publisher(self.cmd_topic, TwistStamped, queue_size=10)
        self.sub = rospy.Subscriber(self.pose_topic, PoseStamped, self.cb_pose, queue_size=10)

    def cb_pose(self, msg: PoseStamped):
        self.last_pose = msg.pose
        self.last_pose_stamp = msg.header.stamp

    @staticmethod
    def _norm3(v):
        return math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])

    def _soft_speed(self, dist):
        """
        Weiche Geschwindigkeitsrampe:
        v_cmd = v_max * tanh(k * dist)
        - Für kleine Distanzen ~ linear
        - Sättigt weich gegen v_max
        """
        return self.v_max * math.tanh(self.k_tanh * dist)

    def _accel_limit(self, v_des, dt):
        """
        Skalarer Beschleunigungsbegrenzer auf die Kart-Geschwindigkeit.
        Begrenze delta_v (als Vektornorm) auf a_max * dt.
        """
        dv = [v_des[0]-self.v_prev[0], v_des[1]-self.v_prev[1], v_des[2]-self.v_prev[2]]
        dv_norm = self._norm3(dv)
        max_step = max(1e-9, self.a_max * dt)
        if dv_norm > max_step:
            scale = max_step / dv_norm
            dv = [dv[0]*scale, dv[1]*scale, dv[2]*scale]
        v_out = [self.v_prev[0]+dv[0], self.v_prev[1]+dv[1], self.v_prev[2]+dv[2]]
        self.v_prev = v_out
        return v_out

    def step(self, dt):
        # Pose verfügbar?
        if self.last_pose is None:
            return False, "waiting_for_pose"

        # Timeout prüfen
        if (rospy.Time.now() - self.last_pose_stamp).to_sec() > self.timeout_s:
            # Keine frische Pose -> auf Null gehen
            self.v_prev = [0.0, 0.0, 0.0]
            self._publish_twist([0.0, 0.0, 0.0])
            return False, "pose_timeout"

        # Positionsfehler
        e = [
            self.target_xyz[0] - self.last_pose.position.x,
            self.target_xyz[1] - self.last_pose.position.y,
            self.target_xyz[2] - self.last_pose.position.z,
        ]
        dist = self._norm3(e)

        # Ziel erreicht?
        if dist <= self.pos_tol:
            self.v_prev = [0.0, 0.0, 0.0]
            self._publish_twist([0.0, 0.0, 0.0])
            return True, "reached"

        # Soll-Geschwindigkeit (weiche Rampe)
        v_scalar = self._soft_speed(dist)
        # Richtungseinheit
        dir_vec = [e[0]/dist, e[1]/dist, e[2]/dist]
        v_des = [dir_vec[0]*v_scalar, dir_vec[1]*v_scalar, dir_vec[2]*v_scalar]

        # Beschleunigungsbegrenzung
        v_cmd = self._accel_limit(v_des, dt)

        # Publizieren (nur linear x,y,z; keine Orientierung)
        self._publish_twist(v_cmd)
        return False, "moving"

    def _publish_twist(self, v):
        msg = TwistStamped()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = v[0]
        msg.twist.linear.y = v[1]
        msg.twist.linear.z = v[2]
        # Winkelgeschwindigkeiten = 0
        self.pub.publish(msg)

def main():
    rospy.init_node("move_to_true_start", anonymous=True)
    node = StartPoseRegulator()
    rate = rospy.Rate(node.rate_hz)
    last = rospy.Time.now()
    reached_counter = 0  # kleine Hysterese für sauberes Stoppen

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        dt = max(1e-3, (now - last).to_sec())
        last = now

        reached, state = node.step(dt)

        if reached:
            reached_counter += 1
            if reached_counter >= int(0.3 * node.rate_hz):  # 0.3s innerhalb Toleranz
                rospy.loginfo("Start position reached.")
                # Optional: hier könnte man Node beenden:
                # break
        else:
            reached_counter = 0

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
