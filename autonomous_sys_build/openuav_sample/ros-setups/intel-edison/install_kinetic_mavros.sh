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

echo "*** Get ROS keys ***"
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

echo "*** Update the OS ***"
sudo apt-get -y update
sudo apt-get -y upgrade

echo "*** Install required OS packages ***"
sudo apt-get -y install python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential

echo "*** ROSDEP ***"
sudo rosdep init
rosdep update

mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws

echo "*** rosinstall ***"
rosinstall_generator ros_comm mavros mavros_extras --rosdistro kinetic --deps --wet-only --exclude roslisp --tar > kinetic-ros_comm-wet.rosinstall

echo "*** wstool ***"
sudo wstool init src -j1 kinetic-ros_comm-wet.rosinstall
while [ $? != 0 ]; do
  echo "*** wstool - download failures, retrying ***"
  sudo wstool update -t src -j1
done

###############
echo "*** rosdep install - Errors at the end may or may not be normal ***"
cd ~/ros_catkin_ws
#  Python errors after the following command are normal.
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y -q -r --os=debian:jessie

# read -p "Check above if there were any rosdep errors. Exit script and proceed manually if there were." -n1 -s

echo "*** Install catkin_tools ***"
sudo apt-get -y install python-catkin-tools
# sudo apt-get install python-psutil

# echo "*** Upgrading python imports. For some reason we need to do these when using mavros_extras? ***"
# sudo pip install future
# sudo apt-get install libxml2-dev libxslt1-dev
# echo "*** This will take a while ***"
# sudo pip install --upgrade lxml

echo “******************************************************************”
echo “About to start some heavy building. Go have a looong coffee break.”
echo “******************************************************************”

echo "*** Building ROS ***"
catkin config --install-space /home/ros/kinetic
catkin config --install
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "The Edison does not have enough memory to support two jobs at once. Hence we must restrict catkin build to only use 1 job. This will take several hours..."
sudo catkin build -j1
# sudo catkin build --mem-limit 50%
# sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /home/ros/kinetic

read -p "ROS built. Press any key to continue..." -n1 -s

sudo ln -sf /home/ros /opt/

# echo "*** Updating .profile and .bashrc ***"
echo "*** Updating .bashrc ***"
# echo "source /home/ros/indigo/setup.bash" >> ~/.profile
# source ~/.profile

# echo "source ~/ros_catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "source /home/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

cd ~/ros_catkin_ws

echo ""
echo "*** FINISHED! ***"

