#!/usr/bin/env bash

# This script will run an example dataset through flame inside a docker
# container and display the output in RViz.
#
# Usage:
#
# >> ./flame_docker_example.sh

nvidia-docker run -it --rm \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    flame \
    bash -ic "roscore & rviz -d /flame_ws/src/flame_ros/cfg/flame.rviz & sleep 5 && roslaunch flame_ros flame_offline_asl.launch"
