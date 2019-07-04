import subprocess
import rospy
import sys
import os
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
import roslaunch


cur_pose = PoseStamped()
NUM_UAV = sys.argv[1]
process = subprocess.Popen(["/bin/bash","/root/src/Firmware/Tools/swarm.sh",NUM_UAV],stdout=subprocess.PIPE)
process.wait()
for line in process.stdout:
        print line

launchfile = "posix_sitl_multi_tmp.launch"
fullpath = os.path.join("/root/src/Firmware/launch/", launchfile)
subprocess.Popen(["roslaunch",fullpath])





