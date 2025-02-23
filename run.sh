#!/bin/bash

# Default values
PORT=40001
IMAGE="openuav:px4-cuda-12.3.0-ubuntu22.04"
CONTAINER_NAME="openuav-session"

# Help function
show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo "Options:"
    echo "  -p PORT     Specify port number (default: 40001)"
    echo "  -i IMAGE    Specify docker image (default: openuav:px4-cuda-12.3.0-ubuntu22.04)"
    echo "  -n NAME     Specify container name (default: openuav-session)"
    echo "  -h         Show this help message"
}

# Parse command line arguments
while getopts "p:i:n:h" opt; do
    case $opt in
        p) PORT="$OPTARG";;
        i) IMAGE="$OPTARG";;
        n) CONTAINER_NAME="$OPTARG";;
        h) show_help; exit 0;;
        \?) echo "Invalid option -$OPTARG" >&2; exit 1;;
    esac
done

# Run the container
docker run -d \
    --name $CONTAINER_NAME \
    --hostname $CONTAINER_NAME \
    --network=cpsvo \
    --gpus all \
    --init \
    --runtime=nvidia \
    -p $PORT:40001 \
    --privileged \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=:1 \
    -e VGL_DISPLAY=:0 \
    $IMAGE

echo "Container $CONTAINER_NAME started on port $PORT"
echo "Access via browser at http://localhost:$PORT"
