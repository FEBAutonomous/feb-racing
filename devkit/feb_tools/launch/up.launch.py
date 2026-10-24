"""The house layer under a stack: what `feb up` starts.

    ros2 launch feb_tools up.launch.py stack:="ros2 launch feb_driver drive.launch.py"

    stack      the command that drives (any ROS command); empty = drive by hand
    car        true on the Jetson: the VESC, lidar and gamepad drivers instead of the simulator bridge
    noise      sensor noise on the simulator's lidar and IMU: "1" for defaults, or "range_sigma:=0.03 ..."
    foxglove   the Foxglove WebSocket on port 8765 (default true)
    deadman    the supervisor needs a gamepad dead-man before anything drives (default: car only)
    max_speed  the supervisor's speed cap in m/s
    track      a track folder (unused here; the simulator app loads it)
    adapter    sim adapter parameters, "speed_per_throttle=25.11,throttle_kp=0" (from the driver's manifest)

Sim or car, the stack sees the same topics: /scan, /odom, /imu, /camera/color/image_raw in, /drive out.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGS = {"stack": "", "car": "false", "noise": "", "foxglove": "true", "deadman": "", "max_speed": "10.0", "track": "", "adapter": ""}


def setup(context):
    cfg = {k: LaunchConfiguration(k).perform(context) for k in ARGS}
    car = cfg["car"].lower() == "true"
    deadman = (cfg["deadman"] or cfg["car"]).lower() == "true"
    sim_time = {"use_sim_time": not car}
    share = get_package_share_directory("feb_tools")
    actions = []
    if car:
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, "launch", "car.launch.py"))))
    else:
        if cfg["noise"]:
            noise = [] if cfg["noise"] == "1" else [a.split(":=", 1) for a in cfg["noise"].split()]
            actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, "launch", "bridge_noisy.launch.py")),
                                                    launch_arguments=noise))
        else:
            actions.append(Node(package="autodrive_roboracer", executable="autodrive_bridge", name="autodrive_bridge",
                                emulate_tty=True, output="screen"))
        # adapter: "key=value,key=value" from the driver's driver.yaml, e.g. a driver with its own
        # throttle law asks for speed_per_throttle=25.11,throttle_kp=0,throttle_ki=0 (an identity)
        adapter = {k: float(v) for k, v in (kv.split("=", 1) for kv in cfg["adapter"].split(",") if "=" in kv)}
        actions.append(Node(package="feb_tools", executable="sim_adapter", name="sim_adapter", output="screen",
                            emulate_tty=True, parameters=[sim_time, adapter]))
    # the lidar watchdog: 0.5 s is the car's rule (car.yaml); a laptop's simulator stalls its scans for
    # longer than that when the scene gets busy, and a STOP there only puts the car into a wall
    sup_params = [sim_time, {"require_deadman": deadman, "max_speed": float(cfg["max_speed"]), "scan_timeout": 2.0}]
    if car:     # the gamepad's button layout lives with the car's other hardware numbers
        sup_params.append(os.path.join(get_package_share_directory("feb_tools"), "config", "car.yaml"))
    actions.append(Node(package="feb_supervisor", executable="supervisor", name="supervisor", output="screen", emulate_tty=True,
                        parameters=sup_params))
    if cfg["foxglove"].lower() == "true":
        actions.append(Node(package="foxglove_bridge", executable="foxglove_bridge", name="foxglove_bridge",
                            parameters=[{"port": 8765, "address": "0.0.0.0"}], arguments=["--ros-args", "--log-level", "warn"]))
    if cfg["stack"]:
        actions.append(ExecuteProcess(cmd=["bash", "-c", cfg["stack"]], name="stack", output="screen", emulate_tty=True, shell=False))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()] + [OpaqueFunction(function=setup)])
