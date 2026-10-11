#!/bin/bash
# Twin entrypoint command: the desktop exactly as before (start_desktop.sh:
# TurboVNC :1 + noVNC :40001), then the robot stack under ros2 launch so that
# `docker restart` brings back PX4 SITL, Gazebo, MicroXRCEAgent, px4_relay and
# the executor without anyone typing into the VNC desktop.
#
#   TWIN_EXECUTOR  px4 (default) | stub | none
#   TWIN_SIM       true (default) | false      start PX4 SITL + Gazebo
#   TWIN_AGENT     true (default) | false      start MicroXRCEAgent
#   TWIN_QGC       true (default) | false      start QGroundControl on the desktop
#   TWIN_TAGS      true (default) | false      apriltag_ros + tag_detector on /drone_camera
#
# This reproduces what used to be typed into the desktop terminal:
#   ros2 launch terrain_mapping_drone_control terrain_mapping.launch.py
#   MicroXRCEAgent udp4 -p 8888
#   ./run_qgroundcontrol.sh
#
# Log: /root/log/twin.log (launch output, appended across restarts).
# (no `set -u`: ROS's setup.bash reads unset variables.)

# Same rule as /root/.bashrc, which does not run for non-interactive shells:
# ROS_DOMAIN_ID from the hostname, mapped into [0,101]. digital-twin-locutus -> 53.
_h="$(printf '%s' "$(hostname)" | md5sum | cut -c1-8)"
export ROS_DOMAIN_ID=$(( 16#$_h % 102 ))
unset _h
echo "[start_twin] hostname $(hostname) -> ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

/usr/local/bin/start_desktop.sh &
desktop_pid=$!

# Wait for the VNC X server before starting anything that opens a window.
for _ in $(seq 1 60); do
    [ -S /tmp/.X11-unix/X1 ] && break
    sleep 1
done
[ -S /tmp/.X11-unix/X1 ] || echo "[start_twin] X :1 not up after 60 s; continuing anyway"
export DISPLAY=:1

source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

mkdir -p /root/log
TWIN_EXECUTOR="${TWIN_EXECUTOR:-px4}"
TWIN_SIM="${TWIN_SIM:-true}"
TWIN_AGENT="${TWIN_AGENT:-true}"
TWIN_QGC="${TWIN_QGC:-true}"
TWIN_TAGS="${TWIN_TAGS:-true}"

term() { echo "[start_twin] stopping"; kill -TERM "${launch_pid:-}" "${qgc_pid:-}" "$desktop_pid" 2>/dev/null; wait; exit 0; }
trap term TERM INT

# QGroundControl (run_qgroundcontrol.sh: xhost + su user + DISPLAY=:1), relaunched if closed.
if [ "$TWIN_QGC" = "true" ] && [ -x /root/run_qgroundcontrol.sh ]; then
    ( while true; do
          /root/run_qgroundcontrol.sh >> /root/log/qgc.log 2>&1
          echo "[start_twin] $(date -u +%FT%TZ) QGroundControl exited; relaunching in 15 s" >> /root/log/qgc.log
          sleep 15
      done ) &
    qgc_pid=$!
fi

# vglrun so Gazebo's GUI renders on the host GPU (VGL_DISPLAY), as it did when
# started from the desktop's "terminator -e vglrun bash".
while true; do
    echo "[start_twin] $(date -u +%FT%TZ) launching twin (executor=$TWIN_EXECUTOR sim=$TWIN_SIM agent=$TWIN_AGENT tags=$TWIN_TAGS)" | tee -a /root/log/twin.log
    vglrun ros2 launch mission_supervisor twin.launch.py \
        "executor:=$TWIN_EXECUTOR" "sim:=$TWIN_SIM" "agent:=$TWIN_AGENT" "tags:=$TWIN_TAGS" >> /root/log/twin.log 2>&1 &
    launch_pid=$!
    wait "$launch_pid"
    echo "[start_twin] $(date -u +%FT%TZ) twin launch exited ($?); restarting in 10 s" | tee -a /root/log/twin.log
    sleep 10
done
