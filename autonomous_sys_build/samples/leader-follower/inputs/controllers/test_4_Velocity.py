"""
Authors: Arjun Kumar and Jnaneshwar Das
testing offboard positon control with a simple takeoff script
"""

import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TwistStamped, Vector3 
from nav_msgs.msg import Odometry
import sys



class TestFollow:
    curr_pose = PoseStamped()

    des_pose = PoseStamped()
    des_vel = TwistStamped()


    leader_odom = Odometry()
    leader_accel = Vector3()

    error = Vector3()    
    errorD = Vector3()
    errorI = Vector3()
    errorLast = Vector3()

    prev_odom = Odometry()

    isReadyToFly = False
    takeOff = False

    alpha = 1.5 #const scale of leader velocity
    kvel = 1.2  #1.2
    kaccel = 0 #1.7

    kp = 1.2
    ki = 0
    kd = 1.2

    def __init__(self, this_uav,leader_uav, H):
        rospy.init_node('offboard_test', anonymous=True)
 	
	print "INIT\n"	

        pose_pub = rospy.Publisher('/mavros'+ this_uav + '/setpoint_position/local', PoseStamped, queue_size=10)
	vel_pub = rospy.Publisher('/mavros' + this_uav + '/setpoint_velocity/cmd_vel', TwistStamped, queue_size = 10)

        rospy.Subscriber('/mavros'+ leader_uav + '/local_position/odom', Odometry, callback=self.leader_cb)
        rospy.Subscriber('/mavros'+ this_uav + '/local_position/pose', PoseStamped, callback=self.follower_cb)
        rospy.Subscriber('/mavros'+ this_uav + '/state', State, callback=self.state_cb)

        rate = rospy.Rate(10)  # Hz
        rate.sleep()

        self.des_pose = self.copy_pose(self.curr_pose)
	self.des_pose.pose.position.z = 2
	self.prev_odom.pose.pose.position.x = -9999	


	#while not self.isReadyToFly:
	   
	print self.des_pose
	print "\n"
	print self.isReadyToFly
	
	sys.stdout.flush()
		
	


        while not rospy.is_shutdown():
            if self.isReadyToFly:
#----------------------------------
		self.error.x = self.leader_odom.pose.pose.position.x - self.curr_pose.pose.position.x
		self.error.y = self.leader_odom.pose.pose.position.y - self.curr_pose.pose.position.y
		self.error.z = self.leader_odom.pose.pose.position.z - self.curr_pose.pose.position.z

		if not self.takeOff:
			pose_pub.publish(self.des_pose)
			rate.sleep()
			self.takeOff = True
			
			self.errorLast.x = self.error.x
			self.errorLast.y = self.error.y
			self.errorLast.z = self.error.z

			self.errorI.x = self.error.x
			self.errorI.y = self.error.y
			self.errorI.z = self.error.z

			continue
#----------------------------------	
		if self.prev_odom.pose.pose.position.x != -9999:
			self.leader_accel.x = self.leader_odom.twist.twist.linear.x - self.prev_odom.twist.twist.linear.x
			self.leader_accel.y = self.leader_odom.twist.twist.linear.y - self.prev_odom.twist.twist.linear.y
		
		self.errorI.x = self.errorI.x + self.error.x
		self.errorI.y = self.errorI.y + self.error.y
		self.errorI.z = self.errorI.z + self.error.z

		self.errorD.x = self.error.x - self.errorLast.x
		self.errorD.y = self.error.y - self.errorLast.y
		self.errorD.z = self.error.z - self.errorLast.z

		self.errorLast.x = self.error.x
		self.errorLast.y = self.error.y
		self.errorLast.z = self.error.z

                self.des_pose.pose.position.x = self.leader_odom.pose.pose.position.x + (self.leader_odom.twist.twist.linear.x * self.alpha)  
                self.des_pose.pose.position.y = self.leader_odom.pose.pose.position.y + (self.leader_odom.twist.twist.linear.y * self.alpha) 
                self.des_pose.pose.position.z = H
                self.des_pose.pose.orientation = self.leader_odom.pose.pose.orientation
		
		self.des_vel.twist.linear.x = ((self.leader_odom.pose.pose.position.x - self.curr_pose.pose.position.x) * self.kvel) + (self.leader_accel.x * self.kaccel)
		self.des_vel.twist.linear.y = ((self.leader_odom.pose.pose.position.y - self.curr_pose.pose.position.y) * self.kvel) + (self.leader_accel.y * self.kaccel)
		self.des_vel.twist.linear.z =  (self.leader_odom.pose.pose.position.z + .5) - self.curr_pose.pose.position.z 
                self.des_vel.twist.angular.x = 0
		self.des_vel.twist.angular.y = 0
		self.des_vel.twist.angular.z = 0	
		
		self.des_vel.twist.linear.x = (self.kp * self.error.x) + (self.kd * self.errorD.x) + (self.ki * self.errorI.x)
		self.des_vel.twist.linear.y = (self.kp * self.error.y) + (self.kd * self.errorD.y) + (self.ki * self.errorI.y)
		#self.der_vel.twist.linear.z = (self.leader_odom.pose.pose.position.z + .5) - self.curr_pose.pose.position.z

		self.prev_odom = self.leader_odom

            vel_pub.publish(self.des_vel)
#	    pose_pub.publish(self.des_pose)
            rate.sleep()
	    

    def copy_pose(self, pose):
        pt = pose.pose.position
        quat = pose.pose.orientation
        copied_pose = PoseStamped()
        copied_pose.header.frame_id = pose.header.frame_id
        copied_pose.pose.position = Point(pt.x, pt.y, pt.z)
        copied_pose.pose.orientation = Quaternion(quat.x, quat.y, quat.z, quat.w)
        return copied_pose

    def follower_cb(self, msg):
        self.curr_pose = msg

    def leader_cb(self, msg):
        self.leader_odom = msg

    def state_cb(self,msg):
        print msg.mode
        if(msg.mode=='OFFBOARD'):
            self.isReadyToFly = True
            print "readyToFly"

if __name__ == "__main__":
    TestFollow(sys.argv[1], sys.argv[2], float(sys.argv[3]))
