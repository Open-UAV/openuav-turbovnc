#!/bin/bash -x

# Start XVnc/X/Lubuntu
chmod -f 777 /tmp/.X11-unix
rm -rf /tmp/.X*-lock
rm -rf /tmp/.X11-unix/*


# From: https://superuser.com/questions/806637/xauth-not-creating-xauthority-file (squashes complaints about .Xauthority)
touch ~/.Xauthority
xauth generate :0 . trusted || true  # Allow this to fail without stopping script

# Make sure xstartup.turbovnc is executable
chmod +x ~/.vnc/xstartup.turbovnc

# Start VNC server with explicit xstartup file
/opt/TurboVNC/bin/vncserver -SecurityTypes None -xstartup ~/.vnc/xstartup.turbovnc :1 &

sleep 3

# Extract GitHub username from container hostname
# Example: if hostname is digital-twin-darknight-007, extract darknight-007
CONTAINER_NAME=$(hostname)
GITHUB_USERNAME=$(echo $CONTAINER_NAME | sed 's/digital-twin-\(.*\)/\1/')

# Clone the repository using the GitHub username
cd /root
git clone https://github.com/${GITHUB_USERNAME}/RAS-SES-598-Space-Robotics-and-AI

# Start NoVNC with the correct command syntax
if [ $? -eq 0 ] ; then
    cd /opt/noVNC
    /opt/noVNC/utils/novnc_proxy --listen 40001 --vnc localhost:5901 --cert /root/self.pem
fi

# /opt/noVNC/utils/launch.sh --vnc localhost:5901 --cert /root/self.pem --listen 40001;
