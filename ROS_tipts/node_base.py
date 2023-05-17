#!/usr/bin/env python3

import rospy

# import String msg
from std_msgs.msg import String

# import service 
from ros_pkg.srv import SomeGoals, SomeResponse

# import dynamic reconfigure service 
from ros_pkg.cfg import someConfig 
from dynamic_reconfigure.server import Server as DRServer

# import action service
from actionlib import SimpleActionServer
from ros_pkg.msg import SomeAction, SomeFeedback, SomeResult


class NodeName(object):

    def __init__(self):
        self.variables_init()
        self.connections_init()

    
    def variables_init(self):
        #svariable
        self.var1 = "var1"
        self.var2 = "var2"

        # msg
        self.pub_msg_type = String()
        self.sub_msg_type = String()

        self.srv_res = SomeResponse()

        # action
        self.act_srv_feedback = SomeFeedback()
        self.act_srv_result = SomeResult()
               

    def connections_init(self):
        # publishers
        
        self.pub = rospy.Publisher("pub_topic", pub_msg_type, queue_size=2)
        
        # subscribers
        rospy.Subscribe("sub_topic", sub_msg_type, self.callback)

        # service
        self.srv = rospy.Service("service_name", SomeGoals, self.execute_srv_cb)

        # dynamic reconfigure services
        srv = DRServer(someConfig, self.dynamic_config_callback)
        
        # actions
        self.act_srv = SimpleActionServer("action_name", SomeAction, execute_cb=self.execute_act_cb, auto_start = False)
        self.act_srv.start()
    
    
    # callback for subscribers
    def callback(self, msg):
        #code
        pass
        
    
    # callback for action
    def execute_srv_cb(self, msg):
        #code
        return self.srv_response


    # callback for dynamic config
    def dynamic_config_callback(self, cfg, level):
        
        self.param1 = cfg["param1"]
        self.param2 = cfg["param2"]
        #...

        return cfg
    
    # callback for action
    def execute_act_cb(self, msg):
        #code
        pass
        

    def run(self):
         # code
        pass
       

if __name__ == '__main__':
    rospy.init_node('node_name')
    node_name = NodeName()
    rospy.spin()
