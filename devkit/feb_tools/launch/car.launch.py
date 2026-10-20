"""The real car's drivers, the RoboRacer way (f1tenth_system's bringup minus its mux and teleop,
which feb_supervisor replaces): VESC motor and servo, VESC odometry, RPLidar S2, gamepad.
Parameters in config/car.yaml. Needs the udev rules from the RoboRacer guide (/dev/sensors/*).
First driven 2026-10-18; see docs/build-car.md, Session 4."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("feb_tools"), "config", "car.yaml")
    return LaunchDescription([
        Node(package="vesc_driver", executable="vesc_driver_node", name="vesc_driver_node", parameters=[cfg], output="screen"),
        Node(package="vesc_ackermann", executable="ackermann_to_vesc_node", name="ackermann_to_vesc_node", parameters=[cfg]),
        Node(package="vesc_ackermann", executable="vesc_to_odom_node", name="vesc_to_odom_node", parameters=[cfg]),
        Node(package="rplidar_ros", executable="rplidar_node", name="rplidar_node", parameters=[cfg], output="screen",
             remappings=[("scan", "scan_raw")]),
        Node(package="feb_tools", executable="scan_rotate", name="scan_rotate", parameters=[cfg], output="screen"),   # scan_raw -> /scan, nose at zero
        Node(package="joy", executable="joy_node", name="joy", parameters=[cfg]),
        Node(package="tf2_ros", executable="static_transform_publisher", name="static_baselink_to_laser",
             arguments=["0.27", "0.0", "0.11", "0.0", "0.0", "0.0", "base_link", "laser"]),
    ])
