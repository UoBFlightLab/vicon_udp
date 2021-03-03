FROM ros:foxy-ros-base

# Copy in the source
COPY . /ros_ws/src/vicon_udp

WORKDIR /ros_ws

# Install dependencies
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && apt-get update \
    && rosdep install --from-paths src \
    && rm -rf /var/lib/apt/lists/

# Build the package
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build

# Use install space setup in ros_entrypoint
RUN sed -i 's#source "/opt/ros/$ROS_DISTRO/setup.bash"#source "/ros_ws/install/setup.bash"#' /ros_entrypoint.sh

# Default Vicon UDP port is 51001
EXPOSE 51001/udp

CMD [ "ros2", "launch", "vicon_udp", "vicon2pose.launch.xml" ]
