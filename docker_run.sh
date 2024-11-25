#!/usr/bin/env bash

BASH_HISTORY_FILE=${PWD%/*}/.bash_history
BASH_RC_FILE=${PWD%/*}/docker/.bashrc

CONTAINER_NAME=turtlebot2-ros-iron:desktop
DOCKER_USER="oddbot"

docker_count=$(docker ps -a | grep CONTAINER_NAME | wc -l)
((docker_count=docker_count+1))

XAUTH=/tmp/.docker.xauth_$docker_count
sleep 0.1
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

# Create a string with all --device options for each device in /dev/
#device_options=""
#for device in /dev/*; do
#    if [ -e "$device" ]; then
#        device_options+="--device=$device "
#    fi
#done

docker run -it \
    --device=/dev/ttyACM0 \
    --device=/dev/kobuki \
    --device=/dev/ttyUSB0 \
    -v /dev:/dev \
    --device-cgroup-rule "c 81:* rmw" \
    --device-cgroup-rule "c 189:* rmw" \
    --volume="/home/$USER/ros2ws/robot:/root/robot" \
    --network=host \
    --privileged \
    turtlebot2-ros-iron:desktop

#docker run -it --rm \
#    --name CONTAINER_NAME-$docker_count \
#    --user $(id -u):$(id -g) \
#    --volume="${PWD%/*}:/home/$DOCKER_USER" \
#    --volume="$BASH_HISTORY_FILE:/home/$DOCKER_USER/.bash_history" \
#    --volume="$BASH_RC_FILE:/home/$DOCKER_USER/.bashrc" \
#    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
#    --volume="$XAUTH:$XAUTH" \
#    --env="XAUTHORITY=$XAUTH" \
#    --env="DISPLAY" \
#    --env="QT_X11_NO_MITSHM=1" \
#    --workdir="/home/$DOCKER_USER" \
#    $device_options \
#    --net=host \
#    --privileged \
#    CONTAINER_NAME:latest

echo "Docker container exited."
