#!/bin/bash

# Exit on any error
set -e

echo "Building OpenUAV Docker images..."

# Build base image with CUDA support
echo "Building base image..."
cd autonomous_sys_build
docker build -f Dockerfile.base -t openuav:base-cuda-12.3.0-ubuntu22.04 .

# Build ROS2 Humble image
echo "Building ROS2 Humble image..."
docker build -f Dockerfile.ros -t openuav:ros-cuda-12.3.0-ubuntu22.04 .

# Build PX4 image
echo "Building PX4 image..."
docker build -f Dockerfile.px4 -t openuav:px4-cuda-12.3.0-ubuntu22.04 .

cd ..

echo "Build complete!"


