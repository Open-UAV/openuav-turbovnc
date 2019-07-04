#launch the patrol sim
#!/bin/bash
nvidia-docker run -dit --net=openuavapp_default --name=openuavapp_x${3:-`date +%s`} -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/junk/openuav-playground/samples/formation/:/simulation -e DISPLAY=:0 --entrypoint "/home/setup.sh" openuavapp_openuav

docker ps


