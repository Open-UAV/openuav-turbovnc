#/bin/bash

docker run --init --runtime=nvidia --name=autonomous_sys_build -it -e DISPLAY=:1 -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0:rw -p 40001:40001 openuav:px4-cuda-10.2-ubuntu18.04
