#!/usr/bin/env python3

import rospy

# import msgs
from std_msgs.msg import String

# import dependencies
import cv2


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


    def connections_init(self):
        # publishers
        self.pub = rospy.Publisher("pub_topic", self.pub_msg_type, queue_size=2)
        
        # subscribers
        rospy.Subscribe("sub_topic", self.sub_msg_type, self.callback)
    
    
    # callback for subscribers
    def callback(self, msg):
        #code
        pass
        

    def run(self):
        # coder
        pass
       

if __name__ == '__main__':
    rospy.init_node('node_name')
    node_name = NodeName()
    rospy.spin() # or node_name.run()
