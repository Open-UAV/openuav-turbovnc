"""
Authors: Arjun Kumar <karjun@seas.upenn.edu> and Jnaneshwar Das <jnaneshwar.das@gmail.com>
testing follower(leader) controller using offboard position control 
"""

import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TwistStamped
import math
import sys
import tf



class TestFollow:
    curr_pose = PoseStamped()
    des_pose = PoseStamped()
    leader_pose = PoseStamped()
    leader_vel = TwistStamped()
   

    isReadyToFly = False
    #alpha = .7#const scale of leader velocity

    def __init__(self, this_uav,leader_uav, D_GAIN):
        rospy.init_node('offboard_test', anonymous=True)

        pose_pub = rospy.Publisher('/mavros'+ this_uav + '/setpoint_position/local', PoseStamped, queue_size=10)

        rospy.Subscriber('/mavros'+ leader_uav + '/local_position/pose', PoseStamped, callback=self.leader_cb)
        rospy.Subscriber('/mavros'+ this_uav + '/local_position/pose', PoseStamped, callback=self.follower_cb)
        rospy.Subscriber('/mavros'+ this_uav + '/state', State, callback=self.state_cb)
	rospy.Subscriber('/mavros'+ leader_uav + '/local_position/velocity', TwistStamped, callback=self.leaderVel_cb)

        rate = rospy.Rate(100)  # Hz
        rate.sleep()
        self.des_pose = self.copy_pose(self.curr_pose)
        while not rospy.is_shutdown():
            if self.isReadyToFly:
                self.des_pose.pose.position.x = self.leader_pose.pose.position.x + (self.leader_vel.twist.linear.x*D_GAIN)
                self.des_pose.pose.position.y = self.leader_pose.pose.position.y + (self.leader_vel.twist.linear.y*D_GAIN)
		self.des_pose.pose.position.z = self.leader_pose.pose.position.z + 1

		azimuth = math.atan2(self.leader_pose.pose.position.y-self.curr_pose.pose.position.y, self.leader_pose.pose.position.x-self.curr_pose.pose.position.x)
                quaternion = tf.transformations.quaternion_from_euler(0, 0, azimuth)
                self.des_pose.pose.orientation.x = quaternion[0]
                self.des_pose.pose.orientation.y = quaternion[1]
                self.des_pose.pose.orientation.z = quaternion[2]
                self.des_pose.pose.orientation.w = quaternion[3]


            pose_pub.publish(self.des_pose)
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
        self.leader_pose = msg

    def leaderVel_cb(self, msg):
	self.leader_vel = msg

    def state_cb(self,msg):
        print msg.mode
        if(msg.mode=='OFFBOARD'):
            self.isReadyToFly = True
            print "readyToFly"

if __name__ == "__main__":
    TestFollow(sys.argv[1], sys.argv[2], float(sys.argv[3]))
