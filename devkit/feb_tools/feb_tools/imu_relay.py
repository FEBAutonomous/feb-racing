"""/imu on the car: the VESC 6's own IMU, which vesc_driver publishes as /sensors/imu/raw, republished
on the contract's /imu (sensor_msgs/Imu, orientation + rates) so a driver sees the same topic on the
car as in the simulator. The RealSense D435 on the car has no IMU (the D435i would).

Units: vesc_driver copies the VESC's numbers as they come, degrees per second and g (measured on
the car 2026-11-08: gyro slope 48 against the kinematic yaw rate, accel z = 1.05 at rest). The
contract is SI, so this converts: rad/s and m/s^2. `convert_units:=false` passes them through.

Standalone too, for a box whose image predates this node:
    python3 devkit/feb_tools/feb_tools/imu_relay.py
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuRelay(Node):
    def __init__(self):
        super().__init__("imu_relay")
        self.declare_parameter("source", "/sensors/imu/raw")
        self.declare_parameter("convert_units", True)       # VESC: deg/s and g -> rad/s and m/s^2
        src = self.get_parameter("source").value
        self.convert = bool(self.get_parameter("convert_units").value)
        self.pub = self.create_publisher(Imu, "/imu", 10)
        self.create_subscription(Imu, src, self.on_imu, qos_profile_sensor_data)
        self.get_logger().info("imu relay: %s -> /imu%s" % (src, " (deg/s, g -> rad/s, m/s^2)" if self.convert else ""))

    def on_imu(self, msg):
        if not msg.header.frame_id:
            msg.header.frame_id = "base_link"
        if self.convert:
            k = math.pi / 180.0
            w, a = msg.angular_velocity, msg.linear_acceleration
            w.x, w.y, w.z = w.x * k, w.y * k, w.z * k
            a.x, a.y, a.z = a.x * 9.80665, a.y * 9.80665, a.z * 9.80665
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args if args is not None else sys.argv)
    node = ImuRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
