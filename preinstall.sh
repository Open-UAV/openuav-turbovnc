#!/bin/bash

# Exit on any error
set -e

echo "Installing prerequisites for OpenUAV..."

# Update package lists
sudo apt-get update

# Install basic utilities
sudo apt-get install -y \
    curl \
    wget \
    git \
    screen \
    emacs \
    vim \
    build-essential

# Install NVIDIA drivers and CUDA requirements
sudo apt-get install -y \
    nvidia-driver-535 \
    nvidia-utils-535

# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Install NVIDIA Container Toolkit
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
    && curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
    && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# Add user to docker group
sudo usermod -aG docker $USER

echo "Prerequisites installation complete!"
echo "Please log out and log back in for docker group changes to take effect."
