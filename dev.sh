#!/bin/bash
# ----------------------------------------------------------------
# Build docker dev stage and add local code for live development
# ----------------------------------------------------------------

BASH_CMD=""

# Function to print usage
usage() {
    echo "
Usage: dev.sh [-b|bash] [-h|--help]

Where:
    -b | bash       Open bash in docker container (Default in dev.sh)
    -h | --help     Show this help message
    "
    exit 1
}

# Parse command-line options
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -b|bash)
            BASH_CMD=bash
            ;;
        -h|--help)
            usage
            ;;
        *)
            echo "Unknown option: $1"
            usage
            ;;
    esac
    shift
done

# Build docker image up to dev stage
DOCKER_BUILDKIT=1 docker build \
    -t av_camera:latest-dev \
    -f Dockerfile --target dev .

# Run docker image with local code volumes for development
docker run -it --rm --net host --privileged \
    -v /dev:/dev \
    -v /tmp:/tmp \
    -v /etc/localtime:/etc/localtime:ro \
    -v ./av_camera_launch:/opt/ros_ws/src/av_camera_launch \
    av_camera:latest-dev $BASH_CMD
