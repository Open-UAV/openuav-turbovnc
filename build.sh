#!/bin/bash

echo "********************************************************************************"
echo ""
echo "Setting up the host."
echo ""
echo "********************************************************************************"

cd setup_host
./basics.sh
./install-nvidia-drivers.sh
./install-nvidia-docker2.sh
./add_xhost.sh
cd ..

echo "********************************************************************************"
echo ""
echo "Creating the docker images:"
echo ""
echo "1. openuav:base-cuda-10.2-ubuntu18.04"
echo ""
echo "2. openuav:ros-cuda-10.2-ubuntu18.04"
echo ""
echo "3. openuav:px4-cuda-10.2-ubuntu18.04"
echo ""
echo "********************************************************************************"

cd autonomous_sys_build/
docker build -f Dockerfile.base -t openuav:base-cuda-10.2-ubuntu18.04 .
docker build -f Dockerfile.ros -t openuav:ros-cuda-10.2-ubuntu18.04 .
docker build -f Dockerfile.px4 -t openuav:px4-cuda-10.2-ubuntu18.04 .

echo "********************************************************************************"
echo ""
echo "All done. Now you can run the container by running run.sh."
echo ""
echo "********************************************************************************"


