#! /usr/bin/env python3

import rospy

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class ReadOdom( object ):

  def __init__( self ):
    rospy.init_node( 'read_odom' )
    self.odom_sub = rospy.Subscriber( '/odom', Odometry, self.odom_cb )
    
  def odom_cb( self, msg ):
    pose = msg.pose.pose.position
    orient = msg.pose.pose.orientation
     
    x, y, z = pose.x, pose.y, pose.z
    
    roll, pitch, yaw = euler_from_quaternion( ( orient.x, orient.y,
                                                orient.z, orient.w ) )
    
    rospy.loginfo( 'Current pose - lin: (%f, %f, %f) ang: (%f, %f, %f)' % (x, y, z, roll, pitch, yaw) )


if __name__ == '__main__':

  read_odom = ReadOdom()
  rospy.spin()