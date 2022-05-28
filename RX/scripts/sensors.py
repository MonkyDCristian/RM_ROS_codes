#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class Odom(object):

  def __init__( self, topic:str = "\odom"):
    rospy.Subscriber(topic, Odometry, self.actuation)
    # position
    self.x, self.y, self.z = 0, 0, 0
    self.delta_x, self.delta_y, self.delta_z = 0, 0, 0
    # orientation
    self.roll, self.pitch, self.yaw = 0, 0, 0
    self.delta_roll, self.delta_pitch, self.delta_yaw = 0, 0, 0
  
  def actuation( self, data ):
    POSE = data.pose.pose
    self.x, self.y, self.z = POSE.position
    self.roll, self.pitch, self.yaw = euler_from_quaternion(POSE.orientarion.x,
                                                            POSE.orientarion.y, 
                                                            POSE.orientarion.z, 
                                                            POSE.orientarion.w)
    
    self.delta_x, self.delta_y, self.delta_z = data.twist.twist.lineal
    self.delta_roll, self.delta_pitch, self.delta_yaw self.speed = data.twist.twist.angular
    # rospy.loginfo( 'speed received: %f' % ( self.speed ))


