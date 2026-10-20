"""The lidar's mounting yaw, taken out of the scan.

The RPLidar is bolted at whatever angle its cable allows; this node re-labels each scan so that
/scan's zero is the car's nose, and every driver can keep assuming it is. Subscribes scan_raw
(the lidar node, remapped), publishes scan. One parameter, lidar_yaw, radians, counter-clockwise:
a ray the lidar calls angle a is at angle a + lidar_yaw in the car's frame. Assumes a full
360-degree scan, as the S2 gives with angle_compensate on.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

QOS = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=10)


class ScanRotate(Node):
    def __init__(self):
        super().__init__("scan_rotate")
        self.declare_parameter("lidar_yaw", 0.0)
        self.pub = self.create_publisher(LaserScan, "scan", QOS)
        self.create_subscription(LaserScan, "scan_raw", self.on_scan, qos_profile_sensor_data)
        self.get_logger().info("scan_rotate: lidar_yaw %.3f rad" % self.get_parameter("lidar_yaw").value)

    def on_scan(self, m):
        yaw = float(self.get_parameter("lidar_yaw").value)
        n = len(m.ranges)
        if n and m.angle_increment:
            k = int(round(yaw / m.angle_increment)) % n          # out[i] = in[i - k]: car angle = lidar angle + yaw
            if k:
                m.ranges = np.roll(np.asarray(m.ranges, dtype=np.float32), k).tolist()
                if len(m.intensities) == n:
                    m.intensities = np.roll(np.asarray(m.intensities, dtype=np.float32), k).tolist()
        self.pub.publish(m)


def main():
    rclpy.init()
    node = ScanRotate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
