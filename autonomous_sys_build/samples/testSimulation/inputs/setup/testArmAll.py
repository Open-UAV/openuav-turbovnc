import rospy
import subprocess
import os
import sys
from std_msgs.msg import Float64;

from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped,Pose,Vector3,Twist,TwistStamped
from std_srvs.srv import Empty
import time 
cur_pose = PoseStamped()
def pos_cb(msg):
    global cur_pose
    cur_pose = msg

NUM_UAV = int(sys.argv[1])

mode_proxy = [None for i in range(NUM_UAV)]
arm_proxy = [None for i in range(NUM_UAV)]

def mavrosTopicStringRoot(uavID=0):
    return ('mavros' + str(uavID+1))

rospy.init_node('multi', anonymous=True)

#Comm for drones
for uavID in range(0,NUM_UAV):
    mode_proxy[uavID] = rospy.ServiceProxy(mavrosTopicStringRoot(uavID) + '/set_mode', SetMode)
    arm_proxy[uavID] = rospy.ServiceProxy(mavrosTopicStringRoot(uavID) + '/cmd/arming', CommandBool)

print 'communication initialization complete'
data = [None for i in range(NUM_UAV)]

while None in data:
    for uavID in range(0, NUM_UAV):
        try:
            data[uavID] = rospy.wait_for_message(mavrosTopicStringRoot(uavID) + '/global_position/rel_alt', Float64, timeout=5)
        except:
            pass

for uavID in range(0, NUM_UAV):
    print "wait for service"
    rospy.wait_for_service(mavrosTopicStringRoot(uavID) + '/set_mode')
    print "got service"

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    success = [None for i in range(NUM_UAV)]
    for uavID in range(0, NUM_UAV):
        try:
            success[uavID] = mode_proxy[uavID](1,'OFFBOARD')
        except rospy.ServiceException, e:
            print ("mavros/set_mode service call failed: %s"%e)

    success = [None for i in range(NUM_UAV)]
    for uavID in range(0, NUM_UAV):
        rospy.wait_for_service(mavrosTopicStringRoot(uavID) + '/cmd/arming')

    for uavID in range(0, NUM_UAV):
        try:
           success[uavID] = arm_proxy[uavID](True)
        except rospy.ServiceException, e:
           print ("mavros1/set_mode service call failed: %s"%e)
