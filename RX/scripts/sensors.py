#!/usr/bin/env python3

import rospy
<<<<<<< HEAD
from std_msgs.msg import Float64

class PIDController(object):

  def __init__( self, topic:str = "controller"):
    self.set_point = rospy.Publisher( f'/{topic}/setpoint', Float64, queue_size = 1 )
    while self.set_point.get_num_connections() == 0 and not rospy.is_shutdown():
      rospy.sleep(0.2)

    self.state = rospy.Publisher( f'/{topic}/state', Float64, queue_size = 1 )
    while self.state.get_num_connections() == 0 and not rospy.is_shutdown():
      rospy.sleep(0.2)

    rospy.Subscriber(f'/{topic}/control_effort', Float64, self.actuation)
    
    self.speed = 0

    rospy.loginfo( f"ready {topic} connections")

  def pub_set_point( self, set_point ):
    self.set_point.publish( set_point )
  
  def pub_state( self, state ):
    self.state.publish( state )
  
  def actuation( self, data ):
    self.speed = float( data.data )
=======
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
>>>>>>> 02aca03bbea5795414c1e87123e189c975a5e04b
    # rospy.loginfo( 'speed received: %f' % ( self.speed ))


