"""A gamepad made of keys, for running the car's rules in the simulator.

Publishes sensor_msgs/Joy exactly as joy_node does for the 8BitDo pad, so the supervisor
cannot tell the difference. Keys held in this terminal:

    SPACE   RB, the autonomous dead-man: the stack drives while it is held
    M       LB, the manual dead-man: arrows drive while it is held
    arrows  up/down speed, left/right steering (manual)
    X       the stop button
    Q       quit

A key counts as held while the terminal keeps repeating it, so letting go stops the car
within about a third of a second, like the real button. Run by `feb up --deadman`.
"""
import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

HOLD = 0.35                       # s, how long after the last repeat a key still counts as held
FIRST_HOLD = 0.8                  # s, the grace after a fresh press: the OS waits ~0.5 s before it starts repeating
LB, RB, STOP = 4, 5, 6            # the supervisor's defaults
SPEED_AXIS, STEER_AXIS = 1, 3


class KeyJoy(Node):
    def __init__(self):
        super().__init__("key_joy")
        self.pub = self.create_publisher(Joy, "/joy", 10)
        self.create_subscription(String, "/feb/state", lambda m: setattr(self, "state", m.data), 10)
        self.state = "?"
        self.last = {}
        self.grace = {}                # key -> how long after that it still counts as held
        self.quit = False
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        self.create_timer(0.05, self.tick)
        print("key_joy: hold SPACE = autonomous, M = manual (arrows drive), X = stop, Q = quit", flush=True)

    def read_keys(self):
        while select.select([self.fd], [], [], 0)[0]:
            ch = os.read(self.fd, 8).decode(errors="ignore")
            now = time.monotonic()
            if ch == "q" or ch == "Q":
                self.quit = True
            for key, names in ((" ", "space"), ("m", "m"), ("M", "m"), ("x", "x"), ("X", "x"),
                               ("\x1b[A", "up"), ("\x1b[B", "down"), ("\x1b[C", "right"), ("\x1b[D", "left")):
                if key in ch:
                    fresh = not self.held(names)            # a press after a pause, not a repeat
                    self.last[names] = now
                    self.grace[names] = FIRST_HOLD if fresh else HOLD

    def held(self, name):
        return time.monotonic() - self.last.get(name, -1e9) < self.grace.get(name, HOLD)

    def tick(self):
        self.read_keys()
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = [0.0] * 8
        msg.buttons = [0] * 11
        msg.buttons[RB] = int(self.held("space"))
        msg.buttons[LB] = int(self.held("m"))
        msg.buttons[STOP] = int(self.held("x"))
        msg.axes[SPEED_AXIS] = float(self.held("up")) - float(self.held("down"))
        msg.axes[STEER_AXIS] = float(self.held("left")) - float(self.held("right"))
        self.pub.publish(msg)
        sys.stdout.write("\r\033[K[key_joy] RB %d  LB %d  speed %+.0f  steer %+.0f  |  %s" % (
            msg.buttons[RB], msg.buttons[LB], msg.axes[SPEED_AXIS], msg.axes[STEER_AXIS], self.state))
        sys.stdout.flush()
        if self.quit:
            raise KeyboardInterrupt

    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
        print()


def main():
    if not sys.stdin.isatty():
        sys.exit("key_joy needs a terminal (feb up --deadman gives it one)")
    rclpy.init()
    node = KeyJoy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.restore()
        node.destroy_node()


if __name__ == "__main__":
    main()
