# Your submission image: the FEB devkit plus your stack, started from the entrypoint.
#   docker build -t <you>/feb-racer:v1 -f starter_kit/Dockerfile stack/
FROM ghcr.io/febautonomous/feb-devkit:latest
COPY . /home/autodrive_devkit/src/stack
RUN cd /home/autodrive_devkit && . /opt/ros/humble/setup.sh && colcon build --base-paths src/stack
ENV FEB_LAUNCH="ros2 launch feb_driver drive.launch.py"
