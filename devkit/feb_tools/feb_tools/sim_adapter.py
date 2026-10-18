"""The simulator as a RoboRacer car: the topic contract every stack is written against.

    reads  /autodrive/roboracer_1/{lidar, imu, front_camera, left_encoder, right_encoder}
    gives  /scan (LaserScan, frame laser), /odom (Odometry, odom -> base_link, dead-reckoned from
           the wheel encoders and the gyro like the car's VESC odometry), /imu, /camera/color/image_raw
    takes  /ackermann_cmd (AckermannDriveStamped: speed in m/s, steering_angle in rad) and turns
           it into the simulator's throttle and steering, with the throttle law that used to live
           in every driver: feed-forward from the steady-state gain plus a bounded PI trim.

On the car the VESC driver and the lidar driver provide the same topics; nothing above this
node knows which it is running on. A command older than cmd_timeout stops the car.
"""
import math

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, Imu, JointState, LaserScan
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster

NS = "/autodrive/roboracer_1/"
QOS = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=10)
WHEEL_RADIUS = 0.059
MAX_STEER = 0.5236


# speed_per_throttle: the simulator sets the WHEEL speed at 25.11 m/s per unit throttle (measured,
# starter_kit/feb_cone_racer/feb_cone_racer/vehicle.py). A /drive speed is therefore a wheel-speed
# request, exactly what the VESC's ERPM is on the car; the PI trim only covers lag and drag, and
# the slew (4 per second) only strips jitter: a racer that brakes by asking for a lower wheel
# speed must get it within a scan, not be eased into it.


class SimAdapter(Node):
    def __init__(self):
        super().__init__("sim_adapter")
        defaults = dict(speed_per_throttle=25.11, throttle_kp=0.02, throttle_ki=0.03, throttle_slew=4.0,
                        reverse_throttle=0.12, speed_window=0.1, cmd_timeout=0.5, laser_x=0.0, laser_z=0.0)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {k: self.get_parameter(k).value for k in defaults}

        self.encoders = {}          # frame_id -> [(t, angle), ...] over the last speed_window
        self.speed = 0.0
        self.yaw_rate = 0.0
        self.x = self.y = self.yaw = 0.0
        self.last_odom_t = None
        self.throttle = 0.0
        self.integral = 0.0
        self.last_cmd_t = None
        self.last_cmd = None

        self.pub_scan = self.create_publisher(LaserScan, "/scan", QOS)
        self.pub_odom = self.create_publisher(Odometry, "/odom", QOS)
        self.pub_imu = self.create_publisher(Imu, "/imu", QOS)
        self.pub_image = self.create_publisher(Image, "/camera/color/image_raw", QOS)
        self.pub_throttle = self.create_publisher(Float32, NS + "throttle_command", QOS)
        self.pub_steering = self.create_publisher(Float32, NS + "steering_command", QOS)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(LaserScan, NS + "lidar", self.on_scan, QOS)
        self.create_subscription(Imu, NS + "imu", self.on_imu, QOS)
        self.create_subscription(Image, NS + "front_camera", self.on_image, QOS)
        for side in ("left", "right"):
            self.create_subscription(JointState, NS + side + "_encoder", self.on_encoder, QOS)
        self.create_subscription(AckermannDriveStamped, "/ackermann_cmd", self.on_cmd, QOS)
        self.create_timer(0.1, self.watchdog)
        self.get_logger().info("sim adapter: /scan /odom /imu /camera/color/image_raw <- simulator, /ackermann_cmd -> throttle, steering")

    # ---------------------------------------------------------------- sensors

    def on_scan(self, msg):
        msg.header.frame_id = "laser"
        self.pub_scan.publish(msg)

    def on_image(self, msg):
        msg.header.frame_id = "camera"
        self.pub_image.publish(msg)

    def on_imu(self, msg):
        self.yaw_rate = msg.angular_velocity.z
        msg.header.frame_id = "imu"
        self.pub_imu.publish(msg)

    def on_encoder(self, msg):
        """Wheel speed from the encoder angle over the last speed_window (0.1 s: two or three
        samples, as fresh as the racer's tyre model needs; signed, reversing reads negative),
        averaged over both rear wheels; then dead-reckon the pose with the gyro."""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        hist = self.encoders.setdefault(msg.header.frame_id, [])
        hist.append((t, msg.position[0]))
        while len(hist) > 2 and t - hist[0][0] > self.p["speed_window"]:
            hist.pop(0)
        speeds = []
        for h in self.encoders.values():
            if len(h) >= 2 and h[-1][0] > h[0][0]:
                speeds.append((h[-1][1] - h[0][1]) / (h[-1][0] - h[0][0]) * WHEEL_RADIUS)
        if not speeds:
            return
        self.speed = float(np.mean(speeds))
        if self.last_odom_t is not None and t > self.last_odom_t:
            dt = t - self.last_odom_t
            self.yaw += self.yaw_rate * dt
            self.x += self.speed * math.cos(self.yaw) * dt
            self.y += self.speed * math.sin(self.yaw) * dt
            self.publish_odom(msg.header.stamp)
        self.last_odom_t = t

    def publish_odom(self, stamp):
        q = Quaternion(z=math.sin(self.yaw / 2.0), w=math.cos(self.yaw / 2.0))
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = self.speed
        odom.twist.twist.angular.z = self.yaw_rate
        self.pub_odom.publish(odom)
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, "odom", "base_link"
        tf.transform.translation.x, tf.transform.translation.y = self.x, self.y
        tf.transform.rotation = q
        laser = TransformStamped()
        laser.header.stamp, laser.header.frame_id, laser.child_frame_id = stamp, "base_link", "laser"
        laser.transform.translation.x, laser.transform.translation.z = self.p["laser_x"], self.p["laser_z"]
        laser.transform.rotation.w = 1.0
        self.tf.sendTransform([tf, laser])

    # ---------------------------------------------------------------- commands

    def on_cmd(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = min(now - self.last_cmd_t, 0.2) if self.last_cmd_t else 0.02
        self.last_cmd_t, self.last_cmd = now, msg
        v = msg.drive.speed
        steer = float(np.clip(msg.drive.steering_angle, -MAX_STEER, MAX_STEER))
        if v < -1e-3:                       # reversing: a fixed gentle push, the way the drivers did it
            self.throttle, self.integral = -self.p["reverse_throttle"], 0.0
        elif v < 1e-3:                      # stop: throttle 0 is a hard brake in this simulator
            self.throttle, self.integral = 0.0, 0.0
        else:
            self.throttle = self.throttle_law(v, dt)
        self.pub_throttle.publish(Float32(data=float(self.throttle)))
        self.pub_steering.publish(Float32(data=steer / MAX_STEER))

    def throttle_law(self, v_target, dt):
        """Feed-forward from the steady-state gain plus a trim bounded to half of it, so the
        command never drops to zero while speed is wanted; slew-limited."""
        ff = v_target / self.p["speed_per_throttle"]
        error = v_target - self.speed
        self.integral = float(np.clip(self.integral + error * dt, -0.8, 0.8))
        trim = float(np.clip(self.p["throttle_kp"] * error + self.p["throttle_ki"] * self.integral, -0.5 * ff, 0.5 * ff))
        wanted = float(np.clip(ff + trim, 0.0, 1.0))
        slew = self.p["throttle_slew"] * dt
        return float(np.clip(wanted, self.throttle - slew, self.throttle + slew))

    def watchdog(self):
        """No command for cmd_timeout: stop. Also what makes the car stop when the stack dies."""
        if self.last_cmd_t is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_cmd_t > self.p["cmd_timeout"] and self.throttle != 0.0:
            self.throttle, self.integral = 0.0, 0.0
            self.pub_throttle.publish(Float32(data=0.0))


def main():
    rclpy.init()
    node = SimAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_throttle.publish(Float32(data=0.0))
        node.destroy_node()


if __name__ == "__main__":
    main()
