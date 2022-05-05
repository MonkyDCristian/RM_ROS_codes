#!/usr/bin/env python3

import rospy
import numpy as np

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class PathMaker():

	def __init__(self):
		rospy.init_node('my_path')
		self.path_pub = rospy.Publisher('/my_path', Path, queue_size=10)
		self.seq = 0
	
	def publish_path(self, x_list, y_list):
		path = Path()
		
		for x, y in zip(x_list, y_list):
			pose = PoseStamped()
			pose.header.seq = self.seq
			pose.header.stamp = rospy.Time.now()
			pose.header.frame_id = "map"
			
			pose.pose.position.x = x
			pose.pose.position.y = y
			pose.pose.position.z = 0

			pose.pose.orientation.x = 0
			pose.pose.orientation.y = 0
			pose.pose.orientation.z = 0
			pose.pose.orientation.w = 1
			
			path.header = pose.header
			path.poses.append(pose)
			self.seq+=1
		
		while True:
			self.path_pub.publish(path)
			rospy.sleep(0.2)

if __name__ == '__main__':
	x0, y0 = 0, 0
	x = np.linspace(0, 2*np.pi, num=100)
	y = np.sin(x) 
	x, y = x+x0, y+y0
	
	path = PathMaker()
	path.publish_path(x,y)
	rospy.spin()