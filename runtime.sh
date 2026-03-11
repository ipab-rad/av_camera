#!/bin/bash
# ---------------------------------------------------------------------------
# Build docker image and run ROS code for runtime or interactively with bash
# ---------------------------------------------------------------------------

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

# Build docker image only up to base stage
DOCKER_BUILDKIT=1 docker build \
    -t av_camera:latest \
    -f Dockerfile --target runtime .

# Run docker image without volumes
docker run -it --rm --net host --privileged \
    -v /dev:/dev \
    -v /tmp:/tmp \
    -v /etc/localtime:/etc/localtime:ro \
    av_camera:latest $BASH_CMD
