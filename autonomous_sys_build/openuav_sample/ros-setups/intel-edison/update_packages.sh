#!/bin/sh
#
# Updating packages from a source installation
#
# Following instructions from:
# http://answers.ros.org/question/109656/maintaining-a-ros-source-install-add-packages-update-release/
#

# update tools
sudo apt-get -y upgrade
# Note: pip could complain about mission files. if these are broken symlinks just delete them and retry.
# see: https://github.com/pypa/pip/issues/2438
sudo pip install --upgrade catkin-pkg rosinstall_generator wstool rosinstall

cd ~/ros_catkin_ws
# backup old package pointers
mv indigo-ros_comm-wet.rosinstall indigo-ros_comm-wet.rosinstall_old
# fetch latest package pointers
rosinstall_generator ros_comm mavros --rosdistro indigo --deps --wet-only --exclude roslisp --tar > indigo-ros_comm-wet.rosinstall
# check which packages have changed
diff indigo-ros_comm-wet.rosinstall indigo-ros_comm-wet.rosinstall_old

# update .rosinstall
sudo wstool merge -t src indigo-ros_comm-wet.rosinstall
# fetch sources
while [ $? != 0 ]; do
  echo "*** wstool - download failures, retrying ***"
  sudo wstool up -t src -j1 --delete-changed-uris
done

# update dependencies
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro indigo -y -r --os=debian:wheezy

# remove previously built artifacts
# Note: this seemed to be required in my case to rebuild everything correctly,
# else I encountered various issues with dependent libraries (at compile time and at runtime)
mv build_isolated build_isolated_old
mv devel_isolated devel_isolated_old

# start compilation and have a beer
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release -DMAVLINK_DIALECT=pixhawk --install-space /home/ros/indigo
