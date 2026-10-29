"""Pad watchdog for the car: the 8BitDo sleeps after a few idle minutes and drops off its dongle;
the next button press reconnects it and Linux creates the device again as a new file. joy_node
still reads the old, dead one and /joy goes silent for good, so the car ignores the pad until
the run is restarted. This node reads the same device alongside joy_node (the js interface allows
that) and, when the pad is producing events but /joy has been silent for `silent_after` seconds,
ends joy_node; the launch file respawns it and it opens the new device. One button press, the
car answers again. The same covers a pad switched on after the launch: joy_node found nothing
at start and never looks again, so the first few seconds of pad events get it restarted."""
import os
import select
import subprocess
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class PadWatch(Node):
    def __init__(self):
        super().__init__("pad_watch")
        self.declare_parameter("device", "/dev/input/js0")
        self.declare_parameter("silent_after", 3.0)
        self.declare_parameter("joy_process", "joy_node")
        self.dev = self.get_parameter("device").value
        self.silent_after = float(self.get_parameter("silent_after").value)
        self.joy_process = self.get_parameter("joy_process").value
        self.fd = None
        self.last_joy = None
        self.last_event = None
        self.active_since = None       # when the current streak of pad events began
        self.last_kick = 0.0
        self.create_subscription(Joy, "/joy", self.on_joy, 10)
        self.create_timer(0.2, self.tick)

    def on_joy(self, _):
        self.last_joy = time.monotonic()

    def open_dev(self):
        if self.fd is not None:
            return True
        try:
            self.fd = os.open(self.dev, os.O_RDONLY | os.O_NONBLOCK)
            return True
        except OSError:
            return False

    def tick(self):
        t = time.monotonic()
        if not self.open_dev():
            return
        try:
            ready, _, _ = select.select([self.fd], [], [], 0)
            if ready:
                data = os.read(self.fd, 4096)
                if data:
                    if self.last_event is None or t - self.last_event > 1.0:
                        self.active_since = t
                    self.last_event = t
                else:
                    raise OSError("eof")
        except OSError:                    # the device went away: close, reopen on the next tick
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = None
            return
        if self.last_event is None or t - self.last_event > 1.0:
            return                          # the pad is quiet: nothing to judge
        if self.last_joy is not None and t - self.last_joy < self.silent_after:
            return                          # joy_node is relaying: fine
        if t - self.active_since < self.silent_after:
            return                          # the pad only just started talking: give joy_node a moment
        if t - self.last_kick < 5.0:
            return
        self.last_kick = t
        self.get_logger().warn("the pad is sending but /joy is silent: restarting %s so it reopens %s" % (self.joy_process, self.dev))
        subprocess.run(["pkill", "-x", self.joy_process], check=False)


def main():
    rclpy.init()
    node = PadWatch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
