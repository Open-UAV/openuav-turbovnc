"""
Authors: Arjun Kumar <karjun@seas.upenn.edu> and Jnaneshwar Das <jnaneshwar.das@gmail.com>
Testing simple formation control
"""

import rospy
import subprocess
import os
import sys
import math
import tf
import time
import geodesy
import numpy as np
import pid


from std_msgs.msg import Float64, Float64MultiArray, Int8
from std_srvs.srv import Empty

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, ParamGet, ParamSet, ParamPull
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Vector3, Point, Quaternion, Twist, TwistStamped


# UAV'S NUMBERED 0 -> NUM_UAV - 1
# MAKE SURE this_uav feed follows this scheme in run_this.sh
# /mavros topics follow 1 -> NUM_UAV

'''
NOTE - To run annas code, precalculate for 5 drones and put config0_0.txt into formation/ 
     - To run without annas code (will stay in rotation) set if condition at line 95 in sequencer.py from {pass -> continue}
'''

 
class TestFormation: 
   
    def __init__(self, this_uav, NUM_UAV, D_GAIN):
	
	print 'init'	
	print 'this uav = ' + str(this_uav)
	print 'num uav = ' + str(NUM_UAV)
	print 'dir_path = ' + str(os.path.dirname(os.path.realpath(__file__)))
	print 'cwd = ' + str(os.getcwd())

    	global cur_pose 
	global cur_state
	global cur_vel
	global cur_globalPose	
	global command	

	cur_pose = [PoseStamped() for i in range(NUM_UAV)]
	cur_globalPose = [NavSatFix() for i in range(NUM_UAV)]
    	cur_state = [State() for i in range(NUM_UAV)]
    	cur_vel =  [TwistStamped() for i in range(NUM_UAV)]
  
	command = Float64MultiArray()
	command.data = [0,0,0,-1]

	des_pose = PoseStamped() 	
	des_vel = TwistStamped()
	des_vel.header.frame_id = "map"	

	status = Int8()
	status.data = -1
	convergence = 0
	
	t = 0 #time at start of rotation
	ttR = 20  #time to rotate

	comDisk = 0 #40 #size of communication disk between drones
	
	#create pid controllers for R and Theta (Kp, Ki, Kd)
	cmdR = pid.PID(50,0,0)
	cmdTheta = pid.PID(50,25,0)

        rospy.init_node('offboard_test'+str(this_uav), anonymous=True)
	rate = rospy.Rate(100) #Hz
	
	mode_proxy = rospy.ServiceProxy('mavros'+str(this_uav + 1)+'/set_mode' ,SetMode)
	arm_proxy = rospy.ServiceProxy('mavros'+str(this_uav + 1)+'/cmd/arming', CommandBool)
	paramPull_proxy = rospy.ServiceProxy('mavros'+str(this_uav+1)+'/param/pull', ParamPull)
	paramGet_proxy = rospy.ServiceProxy('mavros'+str(this_uav+1)+'/param/get', ParamGet)
	paramSet_proxy = rospy.ServiceProxy('mavros'+str(this_uav+1)+'/param/set', ParamSet)
	

	print('paramPull - \n' + str(paramPull_proxy(True)))
	print('paramGet MAV_TYPE - \n' + str(paramGet_proxy("MAV_TYPE")))
	print('______________________________________________________________________________')
	
	#generating subscribers to ALL drones in the swarm
	for i in range(NUM_UAV):
    		exec('def position_cb'+ str(i) +'(msg): cur_pose['+ str(i) +'] = msg')	
		exec('def globalPosition_cb'+str(i)+'(msg): cur_globalPose['+ str(i) +'] = msg')
		exec('def velocity_cb'+ str(i) +'(msg): cur_vel['+ str(i) +'] = msg')
		exec('def state_cb'+ str(i) +'(msg): cur_state['+ str(i) +'] = msg')
		rospy.Subscriber('/mavros'+ str(i + 1) + '/local_position/pose', PoseStamped, callback= eval('position_cb'+ str(i)))
		rospy.Subscriber('/mavros'+ str(i + 1) + '/global_position/global', NavSatFix, callback= eval('globalPosition_cb'+str(i)))
        	rospy.Subscriber('/mavros'+ str(i + 1) + '/state', State, callback= eval('state_cb'+str(i)))
		rospy.Subscriber('/mavros'+ str(i + 1) + '/local_position/velocity', TwistStamped, callback=eval('velocity_cb'+ str(i)))

	#suscribe to sequencer
	rospy.Subscriber('/sequencer/command', Float64MultiArray, callback = self.command_cb)
        #publish status to sequencer
	status_pub = rospy.Publisher('/sequencer/status'+str(this_uav), Int8, queue_size = 10)
	#publish position setpoints to FCU
	pose_pub = rospy.Publisher('/mavros'+ str(this_uav + 1) + '/setpoint_position/local', PoseStamped, queue_size = 10)
	#publish velocity setpoints to FCU
	vel_pub = rospy.Publisher('/mavros'+str(this_uav + 1) + '/setpoint_velocity/cmd_vel', TwistStamped, queue_size = 10)
	#2nd subscriber to uav-state [temporary]
	rospy.Subscriber('/mavros' + str(this_uav + 1) + '/state', State, callback = self.state_cb)

	#assigning identities for drone ahead and behind in formation
	#only relevant to rotations in cmd 4
	plus_uav = (this_uav + 1) % NUM_UAV
	minus_uav = (this_uav - 1) % NUM_UAV
	
	#GOTO initial holding pattern command = [0,0,25,0]
	des_pose = cur_pose[this_uav]
	des_pose.pose.position.x = 10 * math.sin((this_uav * 2 * math.pi) / NUM_UAV)
	des_pose.pose.position.y = 10 * math.cos((this_uav * 2 * math.pi) / NUM_UAV)
	des_pose.pose.position.z = 20
	pose_pub.publish(des_pose)
	status_pub.publish(status)

	#....fix....double subscribe to status, check and set status in callback?
        print cur_state[this_uav]
	print '---------------arming-----------------------'
	
	print '----WAITING FOR STANDBY STATUS---'
	while True:
		pose_pub.publish(des_pose)
		if cur_state[this_uav].system_status == 3:
			print cur_state[this_uav]
			break

	print 'INITIAL POSITION'
	print cur_globalPose[this_uav]


	while cur_state[this_uav].mode != 'OFFBOARD' and not cur_state[this_uav].armed:
		print 'intial arming'
		mode_sent = False
		success = False
		pose_pub.publish(des_pose)		
		while not mode_sent:
			rospy.wait_for_service('mavros'+str(this_uav + 1)+'/set_mode', timeout = None)
			try:
				mode_sent =  mode_proxy(1,'OFFBOARD')
			except rospy.ServiceException as exc:
				print exc
			rate.sleep()	
		while not success:
			rospy.wait_for_service('mavros'+str(this_uav + 1)+'/cmd/arming', timeout = None)
			try: 	
				success =  arm_proxy(True)
			except rospy.ServiceException as exc:
				print exc
			rate.sleep()
	
	print 'mode_sent - ' + str(mode_sent)
	print 'arm_sent - ' + str(success)
	print 'armed?'
	rate.sleep()
	print cur_state[this_uav]

	#wait for sequencer to connect to /sequencer/status# 
	nc = status_pub.get_num_connections()
	print 'num_connections = ' +str(nc) 
	sys.stdout.flush()
	while nc == 0:
		nc = status_pub.get_num_connections()
		rate.sleep()

	print 'num_connections = ' + str(nc)
	print 'FLAG'
	print rospy.is_shutdown()
	sys.stdout.flush()
	
	rate.sleep()
	print rospy.is_shutdown()


	#MAIN LOOP
	print 'Loop INIT Time  - ' + str(time.clock())
        while not rospy.is_shutdown():

	    #'''	#hopefully temporary hack
	    while cur_state[this_uav].mode != 'OFFBOARD' and not cur_state[this_uav].armed:
		    print '------------------rearming----------------------------'
	 	    mode_sent = False
		    success = False
		    pose_pub.publish(des_pose)		
		    while not mode_sent:
		    	rospy.wait_for_service('mavros'+str(this_uav + 1)+'/set_mode', timeout = None)
			try:
				mode_sent =  mode_proxy(1,'OFFBOARD')
			except rospy.ServiceException as exc:
				print exc
		    while not success:
			rospy.wait_for_service('mavros'+str(this_uav + 1)+'/cmd/arming', timeout = None)
			try: 	
				success =  arm_proxy(True)
			except rospy.ServiceException as exc:
				print exc
		#temp end
	    #'''
	    if command.data[3] > -1 and command.data[3] < 4:

            	des_pose.pose.position.x = command.data[0] + command.data[2]*math.sin((this_uav * 2 * math.pi)/NUM_UAV)
            	des_pose.pose.position.y = command.data[1] + command.data[2]*math.cos((this_uav * 2 * math.pi)/NUM_UAV)
	    	des_pose.pose.position.z = 25
		
		convergence =  math.sqrt(math.pow(des_pose.pose.position.x-cur_pose[this_uav].pose.position.x,2)+math.pow(des_pose.pose.position.y-cur_pose[this_uav].pose.position.y,2)+math.pow(des_pose.pose.position.z-cur_pose[this_uav].pose.position.z,2))
		    
		if convergence < .5 and status != Int8(command.data[3]):

		    status = Int8(command.data[3])
		    print 'Status Set - '+ str(status) + '  time - '+ str(time.clock())
		    print 'Current Pose - ' + str(cur_pose[this_uav].pose)
		    status_pub.publish(status)

		pose_pub.publish(des_pose)
	    
	    if command.data[3] == 4:
		
		#timing on rotation
		if t == 0:
			t = time.clock()
			print t
		if time.clock() > t + ttR:	
			status = Int8(command.data[3])
			print 'Status Set - '+ str(status) + '  time - '+ str(time.clock())
			status_pub.publish(status)
			ttR = 1000000000000 
		

		r = math.sqrt(math.pow(cur_pose[this_uav].pose.position.x,2) + math.pow(cur_pose[this_uav].pose.position.y,2))

		theta = math.atan2(cur_pose[this_uav].pose.position.y, cur_pose[this_uav].pose.position.x) % (2*math.pi) #from [-pi,pi]	-> [0, 2pi]
		theta_plus = math.atan2(cur_pose[plus_uav].pose.position.y, cur_pose[plus_uav].pose.position.x) % (2*math.pi)
		theta_minus = math.atan2(cur_pose[minus_uav].pose.position.y, cur_pose[minus_uav].pose.position.x) % (2*math.pi) 
		

		#deal with wrap around
		if theta_minus < theta: #yea looks backwards
			dtheta_minus = (theta_minus + 2*math.pi) - theta
		else:
			dtheta_minus = theta_minus - theta
		if theta < theta_plus: #...again...backwards
			dtheta_plus = (theta + 2*math.pi) - theta_plus
		else:	
			dtheta_plus = theta - theta_plus
		
		

		if comDisk: #do stuff
			distPlus = math.sqrt(math.pow(cur_pose[this_uav].pose.position.x-cur_pose[plus_uav].pose.position.x,2)+math.pow(cur_pose[this_uav].pose.position.y-cur_pose[plus_uav].pose.position.y,2))
			distMinus = math.sqrt(math.pow(cur_pose[this_uav].pose.position.x-cur_pose[minus_uav].pose.position.x,2)+math.pow(cur_pose[this_uav].pose.position.y-cur_pose[minus_uav].pose.position.y,2)) 
			if distPlus > comDisk or distMinus > comDisk:
				print 'comDisk'
				print '     distPlus  - ' + str(distPlus) 
				print '     distMinus - ' + str(distMinus) 
			if distPlus > comDisk and distMinus > comDisk:
				dtheta_plus = dtheta_minus = 2
			elif distPlus > comDisk:
				dtheta_plus  = 2
				dtheta_minus = 0	
			elif distMinus > comDisk:
				dtheta_minus = 0
				dtheta_plus  = .5
	
		thetaError = (dtheta_minus - dtheta_plus)
		rError = command.data[2] - r

		thetaDot = cmdTheta.update(thetaError) + 5
		rDot = cmdR.update(rError)
		
		if thetaDot < 0:
			thetaDot = 0 		

		des_vel.twist.linear.x = (rDot*math.cos(theta) - r*thetaDot*math.sin(theta)) * 2
		des_vel.twist.linear.y = (rDot*math.sin(theta) + r*thetaDot*math.cos(theta)) * 2
		des_vel.twist.linear.z = (25 - cur_pose[this_uav].pose.position.z) * .5 
		vel_pub.publish(des_vel)

		print theta 		

	    if command.data[3] == 5:
		config = np.loadtxt('/simulation/config0_0.txt')		
		status = Int8(command.data[3])
		print 'Status Set - '+ str(status) + '  time - '+ str(time.clock())
		status_pub.publish(status)

	    if command.data[3] > 5:

            	des_pose.pose.position.x = config[ int(command.data[3] - 6), (4*this_uav)]
            	des_pose.pose.position.y = config[ int(command.data[3] - 6), (4*this_uav) + 1 ]
	    	des_pose.pose.position.z = 20 + (2*this_uav)
		
		convergence =  math.sqrt(math.pow(des_pose.pose.position.x-cur_pose[this_uav].pose.position.x,2)+math.pow(des_pose.pose.position.y-cur_pose[this_uav].pose.position.y,2)+math.pow(des_pose.pose.position.z-cur_pose[this_uav].pose.position.z,2))
		    
		if convergence < .5 and status != Int8(command.data[3]):

		    status = Int8(command.data[3])
		    print 'Status Set - '+ str(status) + '  time - '+ str(time.clock())
		    print 'Current Pose - ' + str(cur_pose[this_uav].pose)
		    print 'Cmd Pose - (' + str(config[ int(command.data[3] - 6), (4*this_uav) ]) + ',' + str(config[ int(command.data[3] - 6) ,(4*this_uav) + 1 ]) + ')'
		    status_pub.publish(status)
		pose_pub.publish(des_pose)


	    rate.sleep()
		#azimuth = math.atan2(self.leader_pose.pose.position.y-self.curr_pose.pose.position.y, self.leader_pose.pose.position.x-self.curr_pose.pose.position.x)
                #quaternion = tf.transformations.quaternion_from_euler(0, 0, azimuth)
                #self.des_pose.pose.orientation.x = quaternion[0]
                #self.des_pose.pose.orientation.y = quaternion[1]
                #self.des_pose.pose.orientation.z = quaternion[2]
                #self.des_pose.pose.orientation.w = quaternion[3]

    def copy_pose(self, pose):
        pt = pose.pose.position
        quat = pose.pose.orientation
        copied_pose = PoseStamped()
        copied_pose.header.frame_id = pose.header.frame_id
        copied_pose.pose.position = Point(pt.x, pt.y, pt.z)
        copied_pose.pose.orientation = Quaternion(quat.x, quat.y, quat.z, quat.w)
        return copied_pose

    def state_cb(self, msg):
	self.curr_state = msg

    def command_cb(self, msg):
	global command
	command = msg
	print 'COMMAND callback ----- ' + str(msg)
	print 'COMMAND CALLBACK TIME  - '+ str(time.clock())
	sys.stdout.flush()
	


	
if __name__ == "__main__":
    TestFormation(int(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3]))



