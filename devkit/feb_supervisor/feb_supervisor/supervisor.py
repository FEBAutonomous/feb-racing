"""The vehicle supervisor: the one node between every driver and the wheels.

States, published on /feb/state:
    IDLE        nothing drives (boot, or the dead-man released)
    MANUAL      the gamepad drives, its manual button held
    AUTONOMOUS  the stack's /drive passes through, the autonomous button held (or no dead-man
                required, as in the simulator), under the speed cap
    STOP        a fault: the lidar went silent, a collision is imminent, or someone pressed
                stop. Brakes, and leaves for IDLE only once every button is released and the
                sensors have been healthy for a second.

In: /drive (the stack, AckermannDriveStamped), /joy, /scan, /odom; on the car also /feb/battery
(the gauge) and /sensors/core (the VESC).  Out: /ackermann_cmd at 50 Hz, consumed by the sim
adapter or the VESC driver. Same code in the simulator and on the car; the simulator runs it with
require_deadman false, the car with true.

The car's three guards, all off unless car.yaml sets them (the simulator has no battery or VESC):
    battery_floor   V resting: below it the car will not leave IDLE ("charge it" on /feb/state)
    battery_cut     V measured under load: below it while driving, STOP (the pack is collapsing)
    stall_current   A motor current with the wheels not turning for stall_time: throttle cut for
                    stall_release seconds (the car is pushing against something; pack, gears and
                    bumper all prefer it stops)
    speed_ceiling   m/s, the car's own cap on top of the driver card's: the lower wins

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
from sensor_msgs.msg import BatteryState, Joy, LaserScan
from std_msgs.msg import String

try:
    from vesc_msgs.msg import VescStateStamped
except ImportError:                      # an image without the VESC packages: no stall guard
    VescStateStamped = None

IDLE, MANUAL, AUTONOMOUS, STOP = "IDLE", "MANUAL", "AUTONOMOUS", "STOP"


class Supervisor(Node):
    def __init__(self):
        super().__init__("supervisor")
        defaults = dict(require_deadman=True, max_speed=1.0, max_steer=0.5236, scan_timeout=0.5, drive_timeout=0.5,
                        ttc_stop=0.0, recover_after=1.0, rate=50.0,
                        manual_button=4, auto_button=5, stop_button=6, speed_axis=1, steer_axis=3,
                        manual_speed=1.0, manual_steer=0.4,
                        battery_floor=0.0, battery_cut=0.0, speed_ceiling=0.0,
                        stall_current=0.0, stall_time=0.5, stall_erpm=400.0, stall_release=1.0)
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
        self.batt_rest = None          # resting-voltage estimate from the gauge (sum of its cell estimates)
        self.batt_raw = None           # measured pack voltage, sags under load
        self.batt_t = None
        self.motor_i = 0.0             # VESC motor current, A
        self.motor_rpm = 0.0           # VESC electrical rpm
        self.vesc_t = None
        self.stall_since = None
        self.stalled_until = 0.0
        self.floor_said = None

        self.pub_cmd = self.create_publisher(AckermannDriveStamped, "/ackermann_cmd", 10)
        self.pub_state = self.create_publisher(String, "/feb/state", 10)
        self.create_subscription(AckermannDriveStamped, "/drive", self.on_drive, 10)
        self.create_subscription(Joy, "/joy", self.on_joy, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(BatteryState, "/feb/battery", self.on_battery, 10)
        if VescStateStamped is not None:
            self.create_subscription(VescStateStamped, "/sensors/core", self.on_vesc, 10)
        self.create_timer(1.0 / self.p["rate"], self.tick)
        self.create_timer(0.5, self.announce)
        self.cap = self.p["max_speed"]
        if self.p["speed_ceiling"] > 0:
            self.cap = min(self.cap, self.p["speed_ceiling"])
        guards = [g for g, on in (("battery floor %.1f V" % self.p["battery_floor"], self.p["battery_floor"] > 0),
                                  ("sag cut %.1f V" % self.p["battery_cut"], self.p["battery_cut"] > 0),
                                  ("stall cut %.0f A" % self.p["stall_current"], self.p["stall_current"] > 0 and VescStateStamped is not None)) if on]
        self.get_logger().info("supervisor: %s, speed cap %.1f m/s%s%s" %
                               ("dead-man required" if self.p["require_deadman"] else "no dead-man (simulator)", self.cap,
                                " (car ceiling %.1f)" % self.p["speed_ceiling"] if 0 < self.p["speed_ceiling"] < self.p["max_speed"] else "",
                                ", " + ", ".join(guards) if guards else ""))

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

    def on_battery(self, msg):
        self.batt_raw = float(msg.voltage)
        cells = [c for c in msg.cell_voltage if c == c]      # the gauge's resting estimate per cell (nan-free)
        self.batt_rest = sum(cells) if cells else self.batt_raw
        self.batt_t = self.now()

    def on_vesc(self, msg):
        self.motor_i = abs(float(msg.state.current_motor))
        self.motor_rpm = abs(float(msg.state.speed))
        self.vesc_t = self.now()

    def battery_fresh(self):
        return self.batt_t is not None and self.now() - self.batt_t < 3.0

    def under_floor(self):
        """The resting voltage the gauge reports is under the floor: the car must not start a drive."""
        return self.p["battery_floor"] > 0 and self.battery_fresh() and self.batt_rest < self.p["battery_floor"]

    def stalled(self, asked):
        """Throttle is asked for, the motor pulls stall_current, the wheels do not turn: for
        stall_time that is a stall, and the throttle is cut for stall_release seconds."""
        t = self.now()
        if self.p["stall_current"] <= 0 or self.vesc_t is None or t - self.vesc_t > 0.5:
            return False
        if t < self.stalled_until:
            return True
        pushing = asked and self.motor_i > self.p["stall_current"] and self.motor_rpm < self.p["stall_erpm"]
        if pushing:
            self.stall_since = self.stall_since or t
            if t - self.stall_since > self.p["stall_time"]:
                self.stalled_until = t + self.p["stall_release"]
                self.stall_since = None
                self.get_logger().warn("stall: %.0f A with the wheels stopped, throttle cut for %.1f s" % (self.motor_i, self.p["stall_release"]))
                return True
        else:
            self.stall_since = None
        return False

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
        if self.state in (MANUAL, AUTONOMOUS) and self.p["battery_cut"] > 0 and self.battery_fresh() and self.batt_raw < self.p["battery_cut"]:
            return "battery sagging to %.1f V under load: the pack is empty, charge it" % self.batt_raw
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
        elif (manual or auto) and self.state == IDLE and self.under_floor():
            why = "battery %.1f V resting, under the %.1f V floor: charge it" % (self.batt_rest, self.p["battery_floor"])
            if self.floor_said is None or t - self.floor_said > 5.0:
                self.get_logger().warn("not starting: " + why)
                self.floor_said = t
            self.reason = why                  # stays IDLE; the reason says why on /feb/state
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
        if self.stalled(abs(cmd.drive.speed) > 0.05):
            cmd.drive.speed = 0.0
        cap = self.cap
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
