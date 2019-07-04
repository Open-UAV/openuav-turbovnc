#!/bin/bash

# The following installation is based on: http://wiki.ros.org/wiki/edison 
# and http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi

#### NOTE: this is not tested, during opencv compilation the Edison ran out of disk space.

if [ `whoami` == "root" ]; then 
  echo "Do not run this as root!"
  exit 1
fi

echo "*** Update sources.list ***"

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu jessie main" > /etc/apt/sources.list.d/ros-latest.list'

echo "*** Get ROS and Raspian keys ***"
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
wget http://archive.raspbian.org/raspbian.public.key -O - | sudo apt-key add -

echo "*** Update the OS ***"
sudo apt-get -y update
sudo apt-get -y upgrade

echo "*** Install required OS packages ***"
sudo apt-get -y install pkg-config
sudo apt-get -y install python-setuptools python-pip python-yaml python-argparse python-distribute python-docutils python-dateutil python-setuptools python-six

echo "*** Install required ROS packages ***"
sudo pip install rosdep rosinstall_generator wstool rosinstall

echo "*** ROSDEP ***"
sudo rosdep init
rosdep update

mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws

echo "*** rosinstall ***"
rosinstall_generator ros_comm mavros mavros_extras --rosdistro indigo --deps --wet-only --exclude roslisp --tar > indigo-ros_comm-wet.rosinstall

echo "*** wstool ***"
sudo wstool init src -j1 indigo-ros_comm-wet.rosinstall
while [ $? != 0 ]; do
  echo "*** wstool - download failures, retrying ***"
  sudo wstool update -t src -j1
done

echo "*** Install cmake and update sources.list ***"
mkdir ~/ros_catkin_ws/external_src
sudo apt-get -y install checkinstall cmake
sudo sh -c 'echo "deb-src http://mirrordirector.raspbian.org/raspbian/ testing main contrib non-free rpi" >> /etc/apt/sources.list'
sudo sh -c 'echo "deb http://http.debian.net/debian wheezy-backports main" >> /etc/apt/sources.list'
sudo apt-get -y update

echo "*** Install console bridge ***"
cd ~/ros_catkin_ws/external_src
sudo apt-get -y build-dep console-bridge
apt-get -y source -b console-bridge
sudo dpkg -i libconsole-bridge0.2*.deb libconsole-bridge-dev*.deb

echo "*** Install liblz4-dev ***"
sudo apt-get -y install liblz4-dev

###############
# MAVROS extras requires libopencv-dev which adds these deps: liburdfdom-headers-dev, liburdfdom-dev
echo "*** Install liburdfdom-headers-dev ***"
cd ~/ros_catkin_ws/external_src
sudo apt-get -y source -b liburdfdom-headers-dev
sudo dpkg -i liburdfdom-headers-dev_*.deb

echo "*** Install liburdfdom-dev ***"
cd ~/ros_catkin_ws/external_src
sudo apt-get -y install libboost-test-dev libtinyxml-dev
sudo apt-get -y source -b liburdfdom-dev
sudo dpkg -i liburdfdom*.deb

echo "*** Install libopencv-dev ***"
cd ~/ros_catkin_ws/external_src
git clone https://github.com/Itseez/opencv.git
mkdir opencv/release
cd opencv/release
cmake -D CMAKE_BUILD_TYPE=RELEASE -D ENABLE_PRECOMPILED_HEADERS=OFF -D WITH_LIBV4L=ON -D WITH_V4L=ON -D CMAKE_INSTALL_PREFIX=/usr/local ..
# cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..
make
sudo checkinstall -y --pkgname libopencv-dev make install
# < MAVROS extras deps
###############
read -p "Opencv done. Press any key to continue... " -n1 -s

echo "*** rosdep install - Errors at the end are normal ***"
cd ~/ros_catkin_ws
#  Python errors after the following command are normal.
rosdep install --from-paths src --ignore-src --rosdistro indigo -y -r --os=debian:wheezy

echo "*** Install catkin_tools ***"
sudo apt-get install --reinstall python-setuptools
sudo pip install --upgrade setuptools
sudo pip install -U catkin_tools

read -p "catkin_tools done. Press any key to continue... " -n1 -s

# echo "*** Upgrading python imports. For some reason we need to do these when using mavros_extras? ***"
# sudo pip install future
# sudo apt-get install libxml2-dev libxslt1-dev
# echo "*** This will take a while ***"
# sudo pip install --upgrade lxml

echo “******************************************************************”
echo “About to start some heavy building. Go have a looong coffee break.”
echo “******************************************************************”

echo "*** Building ROS ***"
catkin config --install-space /home/ros/indigo
catkin config --install
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
sudo catkin build
# sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /home/ros/indigo

read -p "ROS built. Press any key to continue... " -n1 -s

sudo ln -sf /home/ros /opt/

echo "*** Updating .profile and .bashrc ***"
echo "source /home/ros/indigo/setup.bash" >> ~/.profile
source ~/.profile

echo "source ~/ros_catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

cd ~/ros_catkin_ws

echo ""
echo "*** FINISHED! ***"

