#!/usr/bin/env python3

import rospy

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

class OdomPath():

	def __init__(self):
		rospy.init_node('odom_path')
		rospy.Subscriber('/odom', Odometry, self.odom_cb)
		self.path_pub = rospy.Publisher('/odom_path', Path, queue_size=10)
		self.path = Path()

	def odom_cb(self, data):
		self.path.header = data.header
		
		pose = PoseStamped()
		pose.header = data.header
		rospy.loginfo(data.header)
		pose.pose = data.pose.pose
		self.path.poses.append(pose)

		self.path_pub.publish(self.path)

if __name__ == '__main__':
	odom_path = OdomPath()
	rospy.spin()