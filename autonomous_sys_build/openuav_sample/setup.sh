#!/bin/bash

#Django start server
if [ -z "$1" ]
then
	echo "missing argument. pass the directory name inside /simulation"
	exit 1
fi
python3 /django/manage.py makemigrations &> /dev/null
python3 /django/manage.py migrate &> /dev/null
cd /django/ && /usr/bin/uwsgi_python35 --http-socket :31819 -T -p 4 --module DjangoProject.wsgi  &> /tmp/a &
#Django start server

#cd /wetty/wetty
#node app.js -p 3000 &> /dev/null &
#cd

## Previous clean-up
rm -rf /root/src/Firmware/Tools/sitl_gazebo/models/f450-tmp-*
rm -f /root/src/Firmware/posix-configs/SITL/init/lpe/f450-tmp-*
rm -f /root/src/Firmware/launch/posix_sitl_multi_tmp.launch

#chmod -R 777 /root/src && chmod -R 777 /root/catkin_ws
echo "source /root/catkin_ws/devel/setup.bash" >> /home/.profile
#hack for asu class
#####################
#####################
## Run user script ##
#####################
#####################

#su term
/simulation/$1/run_this.sh
