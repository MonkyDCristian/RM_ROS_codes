#!/usr/bin/env python3

import rospy
import numpy as np
from pid_controller import PIDController
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class FollowTheCarrot(object):

  def __init__(self):
    rospy.init_node('follow_the_carrot')
    self.variables_init()
    self.connections_init()
    
  def variables_init(self):

    # orientation and goal orientation
    self.x, self.y = 0.0, 0.0
    self.ang = 0.0
    
    #valores objtivos
    self.goal_x, self.goal_y = 0.0, 0.0
    self.goal_ang = 0.0
    
    # last orienantation and delta ang
    self.last_ang = 0.0
    self.delta_ang = 0.0

    # message sending frequency: 10Hz
    self.hz = 10
    self.rate_obj = rospy.Rate(self.hz)

    # msg type: Twist
    self.speed_msg = Twist()
    
    # line speed
    self.speed_msg.linear.x = 0.1

    # controller topic, in launch "ns = topic"
    self.topic = "robot_ang"

  def connections_init(self):
    # Turtlebot connections
    self.cmd_vel_mux_pub = rospy.Publisher('/yocs_cmd_vel_mux/input/navigation', Twist, queue_size=10)
    
    #  angule PID controller
    self.ang_PID_controller = PIDController(self.topic)
    
    # Odometry
    rospy.Subscriber('/odom', Odometry, self.set_odom)

    # set point
    rospy.Subscriber('/goal_point', Vector3, self.run)
    
  def set_odom(self, odom_data):
    pose_data = odom_data.pose.pose
    
    self.x, self.y = pose_data.position.x, pose_data.position.y
    
    quaternion = (pose_data.orientation.x,
                  pose_data.orientation.y,
                  pose_data.orientation.z,
                  pose_data.orientation.w)

    row, pitch, yaw = euler_from_quaternion(quaternion)

    self.ang = yaw

    self.delta_ang = self.ang - self.last_ang
    self.last_ang = self.ang

    self.ang_PID_controller.pub_state(self.ang)

  def run(self, goal_point):
    rospy.loginfo(goal_point)
    self.x_goal, self.y_goal = goal_point.x, goal_point.y
    self.dist = np.sqrt((self.x_goal-self.x)**2 + (self.y_goal-self.y)**2) 
    
    # while erro > 0.2 m
    while self.dist > 0.1:
      self.dist = np.sqrt((self.x_goal-self.x)**2 + (self.y_goal-self.y)**2)
      self.goal_ang = np.arctan2((self.y_goal - self.y), (self.x_goal - self.x))
      self.ang_PID_controller.pub_set_point(self.goal_ang)
      
      rospy.loginfo(f"dist = {self.dist}, x = {self.x}, y ={self.y}")

      self.speed_msg.angular.z = self.ang_PID_controller.speed
      self.cmd_vel_mux_pub.publish(self.speed_msg)
      self.rate_obj.sleep()
    
    rospy.loginfo("ready")

if __name__ == '__main__':
  controller = FollowTheCarrot()
  rospy.spin()