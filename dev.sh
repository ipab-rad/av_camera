#!/bin/bash
# ----------------------------------------------------------------
# Build docker dev stage and add local code for live development
# ----------------------------------------------------------------

# Build docker image up to dev stage
DOCKER_BUILDKIT=1 docker build \
    -t av_camera:latest-dev \
    -f Dockerfile --target dev .

# Run docker image with local code volumes for development
docker run -it --rm --net host --privileged \
    -v /dev/shm:/dev/shm \
    -v /etc/localtime:/etc/localtime:ro \
    -v ./av_camera_launch:/opt/ros_ws/src/av_camera_launch \
    -v ./flir_camera_msgs:/opt/ros_ws/src/flir_camera_msgs \
    -v ./spinnaker_camera_driver:/opt/ros_ws/src/spinnaker_camera_driver \
    -v ./spinnaker_synchronized_camera_driver:/opt/ros_ws/src/spinnaker_synchronized_camera_driver \
    av_camera:latest-dev bash
