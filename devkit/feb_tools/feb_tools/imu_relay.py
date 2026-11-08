"""/imu on the car: the VESC 6's own IMU, which vesc_driver publishes as /sensors/imu/raw, republished
on the contract's /imu (sensor_msgs/Imu, orientation + rates) so a driver sees the same topic on the
car as in the simulator. The RealSense D435 on the car has no IMU (the D435i would).

Standalone too, for a box whose image predates this node:
    python3 devkit/feb_tools/feb_tools/imu_relay.py
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuRelay(Node):
    def __init__(self):
        super().__init__("imu_relay")
        self.declare_parameter("source", "/sensors/imu/raw")
        src = self.get_parameter("source").value
        self.pub = self.create_publisher(Imu, "/imu", 10)
        self.create_subscription(Imu, src, self.on_imu, qos_profile_sensor_data)
        self.get_logger().info("imu relay: %s -> /imu" % src)

    def on_imu(self, msg):
        if not msg.header.frame_id:
            msg.header.frame_id = "base_link"
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
