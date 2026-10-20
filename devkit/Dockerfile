# FEB devkit: ROS 2 Humble + the AutoDRIVE RoboRacer bridge (BSD-2, AutoDRIVE Ecosystem) + the
# RoboRacer car's drivers + our house nodes. Same layout and entrypoint contract as the official
# RoboRacer Sim Racing League image, so a container built for FEB races also runs in the league.
#
# Two ways this image runs:
#   league / practice: ENTRYPOINT below, root, throwaway (builds src/stack, starts the bridge, runs FEB_LAUNCH)
#   the box:           box.sh as the main process, user `feb`, persistent, ~/FEB mounted (see ../feb-sim)
FROM ros:humble
ARG DEBIAN_FRONTEND=noninteractive
ARG APT_LAYER=2026-10-19   # bump to rebuild the apt layer from scratch: a cached emulated-arm64 layer once came out missing librclcpp

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip sudo git nano less curl ca-certificates gnupg iproute2 \
        ros-humble-tf-transformations \
        ros-humble-cv-bridge \
        ros-humble-rosbag2 \
        ros-humble-rosbag2-storage-mcap \
        ros-humble-foxglove-bridge \
        ros-humble-ackermann-msgs \
        ros-humble-joy \
        ros-humble-rplidar-ros \
        ros-humble-serial-driver ros-humble-asio-cmake-module ros-humble-io-context \
    && rm -rf /var/lib/apt/lists/* \
    && test -f /opt/ros/humble/lib/librclcpp.so    # a broken ROS install fails here, not three layers later

# node + the Claude Code CLI, so an assistant can run inside the box with the ROS tools
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - \
    && apt-get install -y --no-install-recommends nodejs \
    && npm install -g @anthropic-ai/claude-code \
    && rm -rf /var/lib/apt/lists/* /root/.npm

COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# the RoboRacer car's VESC driver (motor, servo, odometry), built once here; only used on the Jetson
RUN git clone --depth 1 -b humble https://github.com/f1tenth/vesc /home/autodrive_devkit/src/vesc
COPY autodrive_devkit /home/autodrive_devkit/src/autodrive_devkit
COPY feb_tools /home/autodrive_devkit/src/feb_tools
COPY feb_supervisor /home/autodrive_devkit/src/feb_supervisor
RUN cd /home/autodrive_devkit && . /opt/ros/humble/setup.sh && colcon build && rm -rf build log

# the box's user: uid 1000 (Docker Desktop maps file ownership; on Linux feb-sim renumbers it to yours)
RUN useradd -m -s /bin/bash -u 1000 feb && echo "feb ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/feb \
    && chmod 0440 /etc/sudoers.d/feb && chmod -R a+rX /home/autodrive_devkit
COPY bashrc /home/feb/.bashrc
COPY box.sh /home/autodrive_devkit/box.sh
COPY feb-up /usr/local/bin/feb-up
RUN chown feb:feb /home/feb/.bashrc && chmod +x /home/autodrive_devkit/box.sh /usr/local/bin/feb-up

COPY entrypoint.sh /home/autodrive_devkit.sh
# the members' stack is bind-mounted; do not litter it with root-owned bytecode
ENV PYTHONDONTWRITEBYTECODE=1
WORKDIR /home/autodrive_devkit
EXPOSE 4567 8765
ENTRYPOINT ["/home/autodrive_devkit.sh"]
