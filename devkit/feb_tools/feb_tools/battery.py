"""Battery gauge for the real car, from the VESC's own telemetry (no extra hardware).

Reads /sensors/core (vesc_msgs/VescStateStamped: input voltage and current) and publishes
  /feb/battery       sensor_msgs/BatteryState  voltage, current, percentage (0..1), cell voltage
  /feb/battery_text  std_msgs/String           "11.2 V  48%  ~22 min to 20%, ~35 min to stop"
once a second, and logs a warning at 20% and an error at 10%.

Percentage comes from a LiPo open-circuit-voltage table after undoing the load sag
(v_rest = v_in + i * pack_resistance). Minutes come from the remaining charge divided by a
one-minute average of the current the pack really supplies: what the VESC sees plus
`baseline_current`, the Jetson/lidar/servo draw that goes through the power board and past
the VESC. Two estimates: time to 20% (recharge point) and time to the stop voltage.
"""
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

try:
    from vesc_msgs.msg import VescStateStamped
except ImportError:                      # the simulator image has no VESC packages; the node is car-only
    VescStateStamped = None

# LiPo resting voltage per cell -> fraction of capacity left (typical 3S hobby pack)
OCV = [(4.20, 1.00), (4.10, 0.90), (4.03, 0.80), (3.97, 0.70), (3.92, 0.60), (3.87, 0.50),
       (3.83, 0.40), (3.79, 0.30), (3.75, 0.20), (3.70, 0.15), (3.65, 0.10), (3.55, 0.05), (3.30, 0.00)]


def soc_from_cell(v):
    if v >= OCV[0][0]:
        return 1.0
    for (v1, s1), (v0, s0) in zip(OCV, OCV[1:]):
        if v >= v0:
            return s0 + (s1 - s0) * (v - v0) / (v1 - v0)
    return 0.0


class Battery(Node):
    def __init__(self):
        super().__init__("battery")
        self.declare_parameter("cells", 3)
        self.declare_parameter("capacity_ah", 6.0)
        self.declare_parameter("pack_resistance", 0.05)      # ohm, whole pack incl. leads
        self.declare_parameter("baseline_current", 1.5)      # A drawn past the VESC (Jetson, lidar, servo)
        self.declare_parameter("recharge_soc", 0.20)
        self.declare_parameter("stop_cell_voltage", 3.50)    # 10.5 V on 3S
        self.declare_parameter("source_topic", "/sensors/core")
        g = self.get_parameter
        self.cells = int(g("cells").value)
        self.cap = float(g("capacity_ah").value)
        self.r = float(g("pack_resistance").value)
        self.base_i = float(g("baseline_current").value)
        self.recharge = float(g("recharge_soc").value)
        self.stop_v = float(g("stop_cell_voltage").value)

        self.v = None
        self.i = 0.0
        self.last_msg = None
        self.soc_f = None        # filtered SOC
        self.i_avg = None        # ~1 min average of pack current
        self.level = 0           # 0 ok, 1 warned 20%, 2 warned 10%

        self.pub = self.create_publisher(BatteryState, "/feb/battery", 10)
        self.pub_text = self.create_publisher(String, "/feb/battery_text", 10)
        if VescStateStamped is None:
            self.get_logger().error("vesc_msgs not installed: battery gauge idle")
        else:
            self.create_subscription(VescStateStamped, g("source_topic").value, self.on_vesc, 10)
        self.create_timer(1.0, self.tick)

    def on_vesc(self, msg):
        self.v = float(msg.state.voltage_input)
        self.i = max(0.0, float(msg.state.current_input))
        self.last_msg = time.monotonic()

    def tick(self):
        if self.v is None or self.last_msg is None or time.monotonic() - self.last_msg > 3.0:
            self.pub_text.publish(String(data="battery: no VESC telemetry"))
            return
        pack_i = self.i + self.base_i
        self.i_avg = pack_i if self.i_avg is None else self.i_avg + (pack_i - self.i_avg) / 60.0
        v_rest = self.v + self.i * self.r
        cell = v_rest / self.cells
        soc = soc_from_cell(cell)
        self.soc_f = soc if self.soc_f is None else self.soc_f + (soc - self.soc_f) * 0.2

        remaining_ah = self.soc_f * self.cap
        to_recharge = max(0.0, (self.soc_f - self.recharge) * self.cap) / max(self.i_avg, 0.1) * 60.0
        to_stop = max(0.0, (self.soc_f - soc_from_cell(self.stop_v)) * self.cap) / max(self.i_avg, 0.1) * 60.0

        bs = BatteryState()
        bs.header.stamp = self.get_clock().now().to_msg()
        bs.voltage = self.v
        bs.current = -pack_i
        bs.charge = remaining_ah
        bs.capacity = self.cap
        bs.design_capacity = self.cap
        bs.percentage = self.soc_f
        bs.present = True
        bs.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        bs.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        bs.cell_voltage = [cell] * self.cells
        bs.location = "under the deck"
        self.pub.publish(bs)

        pct = int(round(self.soc_f * 100))
        text = "%.1f V  %d%%  ~%d min to %d%%, ~%d min to stop  (%.1f A)" % (
            self.v, pct, to_recharge, int(self.recharge * 100), to_stop, pack_i)
        self.pub_text.publish(String(data=text))

        if cell <= self.stop_v:
            self.get_logger().error("BATTERY EMPTY %.1f V: stop and charge now" % self.v, throttle_duration_sec=10.0)
        elif self.soc_f <= 0.10 and self.level < 2:
            self.level = 2
            self.get_logger().error("battery 10%%: %.1f V, head back and charge" % self.v)
        elif self.soc_f <= self.recharge and self.level < 1:
            self.level = 1
            self.get_logger().warn("battery %d%%: %.1f V, ~%d min of driving left" % (pct, self.v, to_stop))


def main():
    rclpy.init()
    node = Battery()
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
