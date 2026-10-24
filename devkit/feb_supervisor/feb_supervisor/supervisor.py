"""The vehicle supervisor: the one node between every driver and the wheels.

States, published on /feb/state:
    IDLE        nothing drives (boot, or the dead-man released)
    MANUAL      the gamepad drives, its manual button held
    AUTONOMOUS  the stack's /drive passes through, the autonomous button held (or no dead-man
                required, as in the simulator), under the speed cap
    STOP        a fault: the lidar went silent, a collision is imminent, or someone pressed
                stop. Brakes, and leaves for IDLE only once every button is released and the
                sensors have been healthy for a second.

In: /drive (the stack, AckermannDriveStamped), /joy, /scan, /odom.  Out: /ackermann_cmd at 50 Hz,
consumed by the sim adapter or the VESC driver. Same code in the simulator and on the car; the
simulator runs it with require_deadman false, the car with true.

Gamepad defaults are the 8BitDo Ultimate 2C in XInput mode under joy_node: left stick up/down
(axis 1) is speed, right stick left/right (axis 3) is steering, LB (button 4) is the manual
dead-man, RB (button 5) the autonomous one, and both bumpers plus the view button (6) is stop.
"""
import math
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import String

IDLE, MANUAL, AUTONOMOUS, STOP = "IDLE", "MANUAL", "AUTONOMOUS", "STOP"


class Supervisor(Node):
    def __init__(self):
        super().__init__("supervisor")
        defaults = dict(require_deadman=True, max_speed=1.0, max_steer=0.5236, scan_timeout=0.5, drive_timeout=0.5,
                        ttc_stop=0.0, recover_after=1.0, rate=50.0,
                        manual_button=4, auto_button=5, stop_button=6, speed_axis=1, steer_axis=3,
                        manual_speed=1.0, manual_steer=0.4)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {k: self.get_parameter(k).value for k in defaults}

        self.state = IDLE
        self.reason = "boot"
        self.joy = None
        self.joy_t = None
        self.drive = None
        self.drive_t = None
        self.scan_t = None
        self.ttc = math.inf
        self.speed = 0.0
        self.healthy_since = None

        self.pub_cmd = self.create_publisher(AckermannDriveStamped, "/ackermann_cmd", 10)
        self.pub_state = self.create_publisher(String, "/feb/state", 10)
        self.create_subscription(AckermannDriveStamped, "/drive", self.on_drive, 10)
        self.create_subscription(Joy, "/joy", self.on_joy, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_timer(1.0 / self.p["rate"], self.tick)
        self.create_timer(0.5, self.announce)
        self.get_logger().info("supervisor: %s, speed cap %.1f m/s" %
                               ("dead-man required" if self.p["require_deadman"] else "no dead-man (simulator)", self.p["max_speed"]))

    # ---------------------------------------------------------------- inputs

    def now(self):
        # a monotonic clock for the watchdogs: wall-clock jumps (a Windows box whose WSL clock was
        # reset by 7 hours mid-run) must never read as a dead lidar or a released button
        return time.monotonic()

    def on_drive(self, msg):
        self.drive, self.drive_t = msg, self.now()

    def on_joy(self, msg):
        self.joy, self.joy_t = msg, self.now()

    def on_odom(self, msg):
        self.speed = msg.twist.twist.linear.x

    def on_scan(self, msg):
        self.scan_t = self.now()
        if self.p["ttc_stop"] <= 0.0 or self.speed <= 0.05:
            self.ttc = math.inf
            return
        # time to collision along the direction of travel: range / closing speed, over the scan
        best = math.inf
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                closing = self.speed * math.cos(angle)
                if closing > 0.05:
                    best = min(best, r / closing)
            angle += msg.angle_increment
        self.ttc = best

    def button(self, name):
        i = self.p[name]
        return bool(self.joy and self.joy_t and self.now() - self.joy_t < 0.5 and i < len(self.joy.buttons) and self.joy.buttons[i])

    def axis(self, name):
        i = self.p[name]
        return float(self.joy.axes[i]) if self.joy and i < len(self.joy.axes) else 0.0

    # ---------------------------------------------------------------- the machine

    def fault(self):
        """The reason to be in STOP, or None."""
        t = self.now()
        if self.state in (MANUAL, AUTONOMOUS) or self.state == STOP:
            if self.scan_t is None or t - self.scan_t > self.p["scan_timeout"]:
                return "no lidar"
            if self.ttc < self.p["ttc_stop"]:
                return "collision imminent (ttc %.2f s)" % self.ttc
        if self.button("stop_button"):
            return "stop button"
        return None

    def tick(self):
        t = self.now()
        manual, auto = self.button("manual_button"), self.button("auto_button")
        fault = self.fault()
        if fault:
            self.go(STOP, fault)
        elif self.state == STOP:
            if not (manual or auto) and (self.scan_t and t - self.scan_t < self.p["scan_timeout"]):
                self.healthy_since = self.healthy_since or t
                if t - self.healthy_since > self.p["recover_after"]:
                    self.go(IDLE, "recovered")
            else:
                self.healthy_since = None
        elif manual:
            self.go(MANUAL, "manual button")
        elif auto or (not self.p["require_deadman"] and self.drive_t and t - self.drive_t < self.p["drive_timeout"]):
            self.go(AUTONOMOUS, "autonomous button" if auto else "stack is driving")
        else:
            self.go(IDLE, "released" if self.state in (MANUAL, AUTONOMOUS) else self.reason)

        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        if self.state == MANUAL:
            cmd.drive.speed = self.axis("speed_axis") * self.p["manual_speed"]
            cmd.drive.steering_angle = self.axis("steer_axis") * self.p["manual_steer"]
        elif self.state == AUTONOMOUS and self.drive_t and t - self.drive_t < self.p["drive_timeout"]:
            cmd.drive = self.drive.drive
        cap = self.p["max_speed"]
        cmd.drive.speed = max(-cap, min(cap, cmd.drive.speed))
        cmd.drive.steering_angle = max(-self.p["max_steer"], min(self.p["max_steer"], cmd.drive.steering_angle))
        self.pub_cmd.publish(cmd)

    def go(self, state, reason):
        if state != self.state:
            self.get_logger().info("%s -> %s (%s)" % (self.state, state, reason))
            self.state, self.reason = state, reason
            self.healthy_since = None
            self.announce()

    def announce(self):
        self.pub_state.publish(String(data="%s: %s" % (self.state, self.reason)))


def main():
    rclpy.init()
    node = Supervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = AckermannDriveStamped()
        node.pub_cmd.publish(stop)
        node.destroy_node()


if __name__ == "__main__":
    main()
