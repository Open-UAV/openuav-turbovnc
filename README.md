# OpenUAV Flight simulator for single and multi UAVs

### Introduction
  The OpenUAV is a simulation environment developed for Unmanned Aerial Vehicles. 
  The goal of this framework is to have an easy to setup, UAV testing framework for single and multi-UAV simulations.
  The containerized architecture makes this framework capable of scaling and hosting mutiple simulations in an
  on-premise or cloud environment. This project is a modification of the [willkessler/nvidia-docker-novnc](https://github.com/willkessler/nvidia-docker-novnc) to include PX4/QGroundControl software packages UAV simulations.
  
  An example workflow that implements visual servoing for UAVs using Apriltags & YOLO is available in [https://github.com/DREAMS-lab/nsf_cps_challenge](https://github.com/DREAMS-lab/nsf_cps_challenge)

### Acknowledgements:
- NSF grant CNS-1521617
- USDA grant 2015-67021-23857
- GRASP Lab, University of Pennsylvania
- Penn Aerial Robotics
- School of Earth and Space Exploration, ASU
- Arizona State University
- [@willkessler] (https://github.com/willkessler)

**Authors:**
   **[Harish Anand](https://web.asu.edu/jdas/people/harish-anand), hanand4 (at) asu (dot) edu;**
   
   **[Prof. Jnaneshwar "JD" Das](https://sese.asu.edu/node/3438 "Jnaneshwar Das"), [Distributed Robotic Exploration and Mapping Systems Laboratory](https://web.asu.edu/jdas), ASU School of Earth and Space Exploration**
   


### Building a GPU-enhanced Lubuntu Desktop with nvidia-docker2

To build on a plain vanilla Google Compute GPU host:

1. Spin up a GC GPU host on the google console.  Make sure it has at least one Tesla K80 GPU, and decent amount of VCPUs (e.g. 4, and enough disk space, at least 50Gb). Zone `us-east-1c` seems to be the best choice as of April 1, 2018.
2. Upload this repo and unpack it in `/root/build` or wherever you like as a temporary location.
3. Run `preinstall.sh`. This just runs `apt-get update` and puts in `screen` and `emacs` for getting started.
4. Run `build.sh`. This will build everything needed to start up a nvidia-docker2 container with Ubuntu 16.04 and Lubuntu desktop.

To build on local machine with GPU:

1. Replace `autonomous_sys_build/xorg.conf` with the `xorg.conf` file for your GPU.
2. Run `preinstall.sh`. This runs `apt-get update` and puts in `screen` and `emacs` for getting started.
3. Run `build.sh`. This will build everything needed to start up a nvidia-docker2 container with Ubuntu 16.04 and Lubuntu desktop.

### Running the container

To run the container on this host, use `run.sh`. Note that NoVNC will
expect connections on port 40001. Then surf to your host on that port.

### Known issues

1. In the local setup, if you switch users the containers have to be restarted to update for Xorg changes.
   A fix for it is if your machine has more than one GPU, then you can run containers on the other GPU by running an xserver in it.
   
