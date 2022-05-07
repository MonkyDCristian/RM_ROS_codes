#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseArray
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry, Path
import calculos as cl

class Node(object):

  def __init__(self):
    rospy.init_node('nodo')
    self.variables_init()
    self.connections_init()
    rospy.spin()
    
  def variables_init(self):
    self.ready_connections = False
    self.move = False
    self.end = False

    # guarda datos 
    self.save_odom = True
    
    # valores actuales
    self.x, self.y, self.ang = 0.0, 0.0, 0.0

    #valores objtivos
    self.x_goal, self.y_goal = 0.0, 0.0
    self.ang_goal = 0.0, 0.0
    
    # ultimos valores registrados
    self.last_x, self.last_y, self.last_ang = 0.0, 0.0, 0.0
    self.last_ang_goal = 0.0

    # valores de actuación
    self.vel_angular, self.vel_lineal = 0.0, 0.1

    # frecuencia de envio de mensajes: 10Hz y tipo de mensaje
    self.hz = 10
    self.rate_obj = rospy.Rate(self.hz)
    self.speed_msg = Twist()
    
  def connections_init(self):
  
    self.cmd_vel_mux_pub = rospy.Publisher('/yocs_cmd_vel_mux/input/navigation', Twist, queue_size=10)
    self.odometry = rospy.Subscriber('/odom', Odometry, self.ubicar)
    
    """ conexiones del control P y PI del angulo """
    self.ang_setpoint = rospy.Publisher('/robot_ang/setpoint', Float64, queue_size=1)
    self.connexion_set(self.ang_setpoint)

    self.ang_state = rospy.Publisher('/robot_ang/state', Float64, queue_size=1)
    self.connexion_set(self.ang_state)
  
    self.a_actuation = rospy.Subscriber('/robot_ang/control_effort', Float64, self.ang_actuation)
    
    """ conectar al read_and_send_poses"""
    self.subscriber = rospy.Subscriber('/goal_list', Path, self.mover_robot_a_destino_ctl)

    """ init para guardar posiciones"""
    if self.save_odom:   
      with open("odom_pose.txt", "w") as odom_pose:
        odom_pose.close()

    self.ready_connections = True
  
  def connexion_set(self, connexion):
    while connexion.get_num_connections() == 0 and not rospy.is_shutdown():
      rospy.sleep(0.2)

  def ubicar(self, odom_data):
    data = odom_data.pose.pose
    self.x, self.y= data.position.x + 1, data.position.y + 1
    
    # durante el movimiento fijamos el angulo del robot
    # como el de desplazamiento
    if self.move:
      ang_act = cl.calcular_angulo_obj((self.last_x, self.last_y), (self.x, self.y))
      self.last_x, self.last_y = self.x, self.y
      
      if not ang_act:
        ang_act = self.last_ang
      
      self.ang += cl.calcular_delta_angulo(ang_act, self.last_ang)
      self.last_ang = ang_act
      #rospy.loginfo('ang: %f' % (self.ang))
    
      if self.ready_connections:
        self.ang_state.publish(self.ang)

      """ Guardar info """
      if not self.end and self.save_odom:
        with open("odom_pose.txt", "a") as odom_pose:
          odom_pose.write(str((self.x, self.y)) + '\n')

  
  def mover_robot_a_destino_ctl(self, goal_list):
    rospy.loginfo(rospy.get_caller_id() + ": The poses have arrived")
    for goal_pose in goal_list.poses:
      self.x_goal, self.y_goal = goal_pose.pose.position.x, goal_pose.pose.position.y # x + 1 para line
      
      self.vel_angular = 0.0
      
      """ mover el robot a la posicion objetivo """
      self.move = True 
      self.last_x, self.last_y = self.x, self.y
      self.ang_goal = self.ang
      self.last_ang_goal = self.ang
      while np.abs(cl.calcular_distancia_obj((self.x, self.y), (self.x_goal, self.y_goal))) >= 0.4:
        
        ang_obj_act = cl.calcular_angulo_obj((self.x, self.y), (self.x_goal, self.y_goal))
        
        self.ang_goal += cl.calcular_delta_angulo(ang_obj_act, self.last_ang_goal)
        self.last_ang_goal = ang_obj_act

        self.giro_controlado(self.ang_goal)

        self.speed_msg.linear.x, self.speed_msg.angular.z = self.vel_lineal, self.vel_angular
        self.cmd_vel_mux_pub.publish(self.speed_msg)
        self.rate_obj.sleep()

    rospy.loginfo(f'(x, y) = ({self.x},{self.y}) ; goal = ({self.x_goal}, {self.y_goal})')
       
    self.end = True
    rospy.loginfo("end")
  
  def giro_controlado(self, ang_obj):
    self.ang_setpoint.publish(ang_obj)

  def ang_actuation(self, data):
    self.vel_angular = float(data.data)
    
nodo = Node()