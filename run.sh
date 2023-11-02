#!/bin/bash

if [ $# -lt 2 ]; then
    echo "Usage: $0 <project_folder> <docker_image>"
    exit 1
fi

PROJECT_FOLDER=$1
DOCKER_IMAGE=$2

# Additional Docker options
DOCKER_OPTIONS="--privileged \
--gpus 0 \
--env=DISPLAY \
--env=QT_X11_NO_MITSHM=1 \
--env=XAUTHORITY=/tmp/.docker.xauth \
--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
--volume=/tmp/.docker.xauth:/tmp/.docker.xauth:rw \
--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
--net=host"


docker run -it --name tii-challenge -v $PROJECT_FOLDER:/home/catkin_ws/src/ $DOCKER_OPTIONS ros-gazebo-nvidia-tii:20.04
