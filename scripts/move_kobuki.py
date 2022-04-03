#! /usr/bin/env python3

import rospy

from geometry_msgs.msg import Vector3, Twist

class Move_kubuki( object ):

  def __init__( self ):
    rospy.init_node('move_kubuki')
    self.cmd_vel_pub = rospy.Publisher( '/yocs_cmd_vel_mux/input/navigation',  #topic
    					  Twist,                                # ros msg
    					  queue_size = 10 )                     # msg size 
    self.running = True

  def run( self ):
    
    while self.running:
      vel_l = input("velocidad lineal: ")
      vel_a = input("velocidad angular: ")
      
      if vel_l != "out":
        vel_l, vel_a = float(vel_l), float(vel_a)
        twist = Twist( Vector3(vel_l, 0.0, 0.0), Vector3(0.0, 0.0, vel_a))
	
        self.cmd_vel_pub.publish( twist )
      
      else:
      	self.running = False


if __name__ == '__main__':
  
  move = Move_kubuki()
  move.run()


