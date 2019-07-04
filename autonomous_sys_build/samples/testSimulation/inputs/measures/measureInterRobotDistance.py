"""
testing performance measure for leader-follower
"""

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
import sys
import math


class MeasureInterRobotDistance:
    curr_pose = PoseStamped()
    measure = 0
    leader_pose = PoseStamped()

    def follower_cb(self, msg):
        self.curr_pose = msg

    def leader_cb(self, msg):
        self.leader_pose = msg

    def measureDistance(self):
        follower_x = self.curr_pose.pose.position.x
        follower_y = self.curr_pose.pose.position.y
        follower_z = self.curr_pose.pose.position.z

        leader_x = self.leader_pose.pose.position.x
        leader_y = self.leader_pose.pose.position.y
        leader_z = self.leader_pose.pose.position.z

        return math.sqrt((follower_x - leader_x) * (follower_x - leader_x) + (follower_y - leader_y) * (follower_y - leader_y))

# + (follower_z - leader_z) * (follower_z - leader_z))


    def __init__(self, this_uav, leader_uav):
        rospy.init_node('measure', anonymous=True)
        self.measure_pub = rospy.Publisher('/measure', Float64, queue_size=10)
        rospy.Subscriber('/mavros' + leader_uav + '/local_position/pose', PoseStamped, callback=self.leader_cb)
        rospy.Subscriber('/mavros' + this_uav + '/local_position/pose', PoseStamped, callback=self.follower_cb)

        rate = rospy.Rate(10)  # Hz
        rate.sleep()

        while not rospy.is_shutdown():
            self.measure = self.measureDistance()
            self.measure_pub.publish(self.measure)
            rate.sleep()

if __name__ == "__main__":
    MeasureInterRobotDistance(sys.argv[1], sys.argv[2])

