#!/bin/sh

###################################################
# Run this bash script from workspace directory 
# level
#    ./src/ros2basilisk/docker/docker_run.sh
###################################################

xhost +local:root

# Reference:
# https://www.mit.edu/~arosinol/2019/08/06/Docker_Display_GUI_with_X_server/
docker run -it --rm \
    -v "$(pwd)":/home/ubuntu/example_ws \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --net=host \
    --privileged \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    kmatsuka/ros2basilisk

xhost -local:root

