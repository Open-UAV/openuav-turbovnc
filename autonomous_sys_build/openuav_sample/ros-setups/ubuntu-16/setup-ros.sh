export DEBIAN_FRONTEND=noninteractive
apt-get update && apt-get install -y lsb-core
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
apt-get update -y
apt-get -y install ros-kinetic-desktop
apt-get -y install ros-kinetic-gazebo-ros-pkgs
rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
