#!/usr/bin/env python3
import sys, time, tty, termios, fcntl, os
import rospy
from sensor_msgs.msg import Joy

class KeyboardJoy:
    def __init__(self):
        # Topics/Mapping
        self.pub_topic   = rospy.get_param("~joy_topic", "/joy")
        self.axis_x_idx  = rospy.get_param("~axis_x_index", 0)
        self.axis_y_idx  = rospy.get_param("~axis_y_index", 1)
        self.enable_btn  = rospy.get_param("~enable_button_index", -1)  # -1 = keiner
        self.axes_count  = rospy.get_param("~axes_count", 8)
        self.buttons_cnt = rospy.get_param("~buttons_count", 12)

        # Dynamik
        self.rate_hz     = rospy.get_param("~rate_hz", 50.0)
        self.step        = rospy.get_param("~step", 1.0)     # Wert bei Tastendruck (-1/0/1)
        self.decay       = rospy.get_param("~decay", 1.25)   # pro Sek. Richtung -> 0 (gleitend)

        # State
        self.ax = 0.0
        self.ay = 0.0
        self.enable_pressed = (self.enable_btn < 0)  # wenn kein Button, dann immer „an“

        self.pub = rospy.Publisher(self.pub_topic, Joy, queue_size=10)

        # Terminal in nonblocking raw mode
        self.fd = sys.stdin.fileno()
        self.old_term = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        self.old_fl = fcntl.fcntl(self.fd, fcntl.F_GETFL)
        fcntl.fcntl(self.fd, fcntl.F_SETFL, self.old_fl | os.O_NONBLOCK)

    def restore_terminal(self):
        try:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_term)
            fcntl.fcntl(self.fd, fcntl.F_SETFL, self.old_fl)
        except Exception:
            pass

    def read_keys(self):
        """Liest alle verfügbaren Zeichen, setzt ax/ay; Auto-Repeat des Terminals reicht aus."""
        try:
            while True:
                ch = sys.stdin.read(1)
                if not ch:
                    break
                c = ch.lower()
                if c == 'q':
                    rospy.signal_shutdown("quit")
                    return
                elif c == ' ' or c == '0':
                    self.ax, self.ay = 0.0, 0.0
                elif c == 'w':
                    self.ax = -self.step
                elif c == 's':
                    self.ax = +self.step
                elif c == 'a':
                    self.ay = -self.step
                elif c == 'd':
                    self.ay = +self.step
                elif c == 'e':
                    if self.enable_btn >= 0:
                        self.enable_pressed = not self.enable_pressed
                # Pfeiltasten (falls Terminal sendet): \x1b[A etc.
                elif ch == '\x1b':
                    seq = (sys.stdin.read(1) or '') + (sys.stdin.read(1) or '')
                    if seq == '[A':      # Up
                        self.ay = +self.step
                    elif seq == '[B':    # Down
                        self.ay = -self.step
                    elif seq == '[C':    # Right
                        self.ax = +self.step
                    elif seq == '[D':    # Left
                        self.ax = -self.step
        except IOError:
            pass

    def decay_axes(self, dt):
        """Gleitend Richtung 0 (falls keine Taste gehalten/kein Repeat)."""
        def approach_zero(v, rate):
            if v > 0:
                v = max(0.0, v - rate)
            elif v < 0:
                v = min(0.0, v + rate)
            return v
        rate = self.decay * dt
        self.ax = approach_zero(self.ax, rate)
        self.ay = approach_zero(self.ay, rate)

    def publish(self):
        msg = Joy()
        msg.axes = [0.0] * self.axes_count
        msg.buttons = [0] * self.buttons_cnt
        if 0 <= self.axis_x_idx < self.axes_count: msg.axes[self.axis_x_idx] = self.ax
        if 0 <= self.axis_y_idx < self.axes_count: msg.axes[self.axis_y_idx] = self.ay
        if self.enable_btn >= 0 and self.enable_btn < self.buttons_cnt:
            msg.buttons[self.enable_btn] = 1 if self.enable_pressed else 0
        self.pub.publish(msg)

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        last = rospy.Time.now()
        print("\n[keyboard_to_joy] Steuerung: W/A/S/D, SPACE stop, E toggle enable, Q quit\n")
        try:
            while not rospy.is_shutdown():
                now = rospy.Time.now()
                dt = max(1e-3, (now - last).to_sec())
                last = now
                self.read_keys()
                self.decay_axes(dt)
                self.publish()
                rate.sleep()
        finally:
            self.restore_terminal()

def main():
    rospy.init_node("keyboard_to_joy", anonymous=True)
    node = KeyboardJoy()
    node.run()

if __name__ == "__main__":
    main()
