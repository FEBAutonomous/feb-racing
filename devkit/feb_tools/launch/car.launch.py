"""The real car's drivers, the RoboRacer way (f1tenth_system's bringup minus its mux and teleop,
which feb_supervisor replaces): VESC motor and servo, VESC odometry, RPLidar S2, gamepad.
Parameters in config/car.yaml. Needs the udev rules from the RoboRacer guide (/dev/sensors/*).
First driven 2026-10-18; see docs/build-car.md, Session 4."""
import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch_ros.actions import Node


def camera_nodes(cfg):
    """The RealSense D435if on the front bumper: colour only, 640x480 at 30 Hz, on the contract's
    /camera/color/image_raw. Skipped (with a note) on an image without the realsense2_camera package."""
    try:
        get_package_share_directory("realsense2_camera")
    except PackageNotFoundError:
        print("car.launch: realsense2_camera not installed in this image, no camera (./feb update brings it)")
        return []
    return [Node(package="realsense2_camera", executable="realsense2_camera_node", name="camera", namespace="",
                 parameters=[cfg, {"enable_color": True, "enable_depth": False, "enable_infra1": False, "enable_infra2": False,
                                   "enable_gyro": False, "enable_accel": False, "rgb_camera.color_profile": "640x480x30",
                                   "rgb_camera.profile": "640x480x30"}],
                 output="screen", respawn=True, respawn_delay=2.0)]


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("feb_tools"), "config", "car.yaml")
    return LaunchDescription(camera_nodes(cfg) + [
        Node(package="vesc_driver", executable="vesc_driver_node", name="vesc_driver_node", parameters=[cfg], output="screen"),
        Node(package="vesc_ackermann", executable="ackermann_to_vesc_node", name="ackermann_to_vesc_node", parameters=[cfg]),
        Node(package="vesc_ackermann", executable="vesc_to_odom_node", name="vesc_to_odom_node", parameters=[cfg]),
        Node(package="rplidar_ros", executable="rplidar_node", name="rplidar_node", parameters=[cfg], output="screen",
             remappings=[("scan", "scan_raw")]),
        Node(package="feb_tools", executable="scan_rotate", name="scan_rotate", parameters=[cfg], output="screen"),   # scan_raw -> /scan, nose at zero
        Node(package="joy", executable="joy_node", name="joy", parameters=[cfg], respawn=True, respawn_delay=1.0),
        Node(package="feb_tools", executable="pad_watch", name="pad_watch", parameters=[cfg], output="screen"),   # restarts joy_node when the pad comes back from sleep
        Node(package="feb_tools", executable="battery", name="battery", parameters=[cfg], output="screen"),   # /feb/battery from the VESC's voltage
        Node(package="feb_tools", executable="imu_relay", name="imu_relay", output="screen"),   # /sensors/imu/raw (the VESC's IMU) -> /imu, the contract's topic
        Node(package="tf2_ros", executable="static_transform_publisher", name="static_baselink_to_laser",
             arguments=["0.27", "0.0", "0.11", "0.0", "0.0", "0.0", "base_link", "laser"]),
    ])
