#!/usr/bin/bash

# Set Container Name
CONTAINER_NAME="turtlebot2-ros-iron:desktop"

echo "Building ROS2-Iron Container"
# docker build --rm -t $CONTAINER_NAME:latest .

DOCKER_BUILDKIT=1 docker build \
    --secret id=env,src=.env \
    -t turtlebot2-ros-iron:desktop \
    -f turtlebot2_ros2.dockerfile \
    --build-arg from_image=osrf/ros:iron-desktop \
    --build-arg parallel_jobs=4 . &&

echo "Creating temp container"
docker create --name temp_container turtlebot2-ros-iron:desktop &&

echo "Copying contents of root robot temp container"
docker cp temp_container:/root/robot /home/$USER/ros2ws &&

echo "Removing temp container"
docker rm temp_container &&

echo "Docker Build Completed"