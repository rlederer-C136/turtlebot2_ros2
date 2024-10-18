# Copyright 2023-2024 Ingot Robotics

ARG from_image=ros:iron
ARG robot_workspace="/root/robot"

FROM $from_image AS kobuki_builder

RUN apt-get update && apt-get upgrade -y && apt-get install wget -y --no-install-recommends && rm -rf /var/lib/apt/lists/*

ARG robot_workspace
ENV ROBOT_WORKSPACE=$robot_workspace

# Build Kobuki drivers
ENV KOBUKI_BUILD_SPACE=$ROBOT_WORKSPACE/kobuki_build_space
WORKDIR $KOBUKI_BUILD_SPACE

RUN wget -q https://raw.githubusercontent.com/kobuki-base/kobuki_documentation/release/1.0.x/resources/colcon.meta && \
    wget -q https://raw.githubusercontent.com/kobuki-base/kobuki_documentation/release/1.0.x/resources/kobuki_standalone.repos

# Update kobuki_standalone.repos to build on iron
# Comment out foxy ament tools
RUN sed -i 's/ament_/#&/g' $KOBUKI_BUILD_SPACE/kobuki_standalone.repos
# Add lines for kobuki_ros
RUN echo "  cmv_vel_mux      : { type: 'git', url: 'https://github.com/kobuki-base/cmd_vel_mux.git', version: 'devel' }" >> $KOBUKI_BUILD_SPACE/kobuki_standalone.repos && \
    echo "  kobuki_ros       : { type: 'git', url: 'https://github.com/kobuki-base/kobuki_ros.git',  version: 'devel' }" >> $KOBUKI_BUILD_SPACE/kobuki_standalone.repos
# Update ecl_lite version to 1.2.x
RUN sed -i '/ecl_lite/s/release\/1.1.x/release\/1.2.x/g' $KOBUKI_BUILD_SPACE/kobuki_standalone.repos

RUN mkdir -p $ROBOT_WORKSPACE/src && vcs import $ROBOT_WORKSPACE/src < $KOBUKI_BUILD_SPACE/kobuki_standalone.repos
RUN touch $ROBOT_WORKSPACE/src/eigen/AMENT_IGNORE

# Install dependencies
WORKDIR $ROBOT_WORKSPACE
RUN apt-get update && rosdep install --from-paths ./src -y --ignore-src && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]

# Build release with debug symbols
ARG parallel_jobs=8
WORKDIR $ROBOT_WORKSPACE
RUN source "/opt/ros/$ROS_DISTRO/setup.bash" && colcon build --parallel-workers $parallel_jobs --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

FROM $from_image AS mobile_base

ARG robot_workspace
ENV ROBOT_WORKSPACE=$robot_workspace

WORKDIR $ROBOT_WORKSPACE
RUN mkdir -p $ROBOT_WORKSPACE/install
COPY --from=kobuki_builder $ROBOT_WORKSPACE/install/ install

# Install Kobuki dependencies with rosdep
WORKDIR $ROBOT_WORKSPACE
RUN apt-get update && apt-get upgrade -y && rosdep install --from-paths ./install/*/ -y --ignore-src && rm -rf /var/lib/apt/lists/*

# Remove Hokuyo URG node installation (already commented out in your original Dockerfile)
# Removed the following commented lines:
# RUN apt-get update && apt-get install "ros-$ROS_DISTRO-urg-node" -y --no-install-recommends && rm -rf /var/lib/apt/lists/*
# And related patch commands

# Install RealSense drivers and ROS nodes
RUN apt-get update && apt-get install "ros-$ROS_DISTRO"-realsense2-* -y --no-install-recommends && rm -rf /var/lib/apt/lists/*

# Install Nav2 bringup package
RUN apt-get update && apt-get install "ros-$ROS_DISTRO-nav2-bringup" -y --no-install-recommends && rm -rf /var/lib/apt/lists/*

# Install Turtlebot dependencies with rosdep
WORKDIR $ROBOT_WORKSPACE
RUN --mount=type=bind,source=.,target="$ROBOT_WORKSPACE/src",readonly \
    apt-get update && rosdep install --from-paths ./src -y --ignore-src && rm -rf /var/lib/apt/lists/*

# Build turtlebot2_description and turtlebot2_bringup
SHELL ["/bin/bash", "-c"]
ARG parallel_jobs=8
WORKDIR $ROBOT_WORKSPACE
RUN --mount=type=bind,source=.,target="$ROBOT_WORKSPACE/src",readonly \
    source "/opt/ros/$ROS_DISTRO/setup.bash" && \
    colcon build --packages-select turtlebot2_description turtlebot2_bringup --parallel-workers $parallel_jobs --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Install pyserial for the xv11 LIDAR
RUN apt-get update && apt-get install -y python3-serial && rm -rf /var/lib/apt/lists/*

# Clone and build xv11_lidar_python
WORKDIR $ROBOT_WORKSPACE/src
RUN git clone https://github.com/n1kn4x/xv11_lidar_python.git
WORKDIR $ROBOT_WORKSPACE
RUN source "/opt/ros/$ROS_DISTRO/setup.bash" && colcon build

COPY ros_entrypoint.sh /

# Make sure the entrypoint script is executable
RUN chmod +x /ros_entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# Kobuki udev rules for host machine
# `wget https://raw.githubusercontent.com/kobuki-base/kobuki_ftdi/devel/60-kobuki.rules`
# and put that file in `/etc/udev/rules.d/`

# Launch container with LIDAR and Kobuki USB connections
# `docker run -it --device=/dev/ttyUSB0 --device=/dev/kobuki --device=/dev/ttyACM0 <container name>`
# If running rviz on a separate machine, adding `--network=host` is a
# docker networking work-around to allow containers to communicate

# Launch turtlebot2 with
# `ros2 launch turtlebot2_bringup turtlebot2_bringup.launch.py`

# Run xv11_lidar_python node
# `ros2 run xv11_lidar_python xv11_lidar`
# `ros2 run xv11_lidar_python xv11_lidar --ros-args -p port:=/dev/ttyACM0 -p frame_id:=laser_frame`


# Run second container to do keyboard control, and in that container
# `ros2 run kobuki_keyop kobuki_keyop_node`
# or
# `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
