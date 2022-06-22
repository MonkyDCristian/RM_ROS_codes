#!/usr/bin/env python3

import cv2
import rospy 
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float64 ,Bool

from task0_vision_tools import Task0ImgProcessing, MaskConfigHSV

class Task0Vision():
    def __init__(self, show_img = False, show_rects = False):
        
        rospy.init_node( 'task0_vision' )
        
        self.bridge = CvBridge()
        self.tip = Task0ImgProcessing(draw_rect=show_rects, min_area=20)
        self.mask_cfg = MaskConfigHSV(self.tip)
            
        self.show_img = show_img
        self.bouys_found = True

        self.init_connections()
        rospy.spin()
    

    def init_connections(self):
        
        img_topic = rospy.get_param(f"{rospy.get_name()}/img_topic")
        rospy.Subscriber(img_topic, Image, self.callback)

        self.pub_alignment_error = rospy.Publisher("task0/alignment_error", Float64, queue_size=10)
        self.pub_bouys_found = rospy.Publisher("task0/bouys_found", Bool, queue_size=1)
        self.pub_img = rospy.Publisher("task0/image", Image, queue_size=10)
        self.pub_mask = rospy.Publisher("task0/mask", Image, queue_size=10)

        
    def callback(self, img):
        try:
            np_img = self.bridge.imgmsg_to_cv2( img, "bgr8" )

            dict_rects = self.tip.processing(np_img)
            
            self.send_status(dict_rects, np_img)
      
        except CvBridgeError as e:
            rospy.logerr( e )

    
    def send_status(self, dict_rects, np_img):
        
        if len(dict_rects["red"]) > 0 and  len(dict_rects["green"]) > 0:
            if not self.bouys_found:
                self.bouys_found = True

            centers = self.get_centers(dict_rects)

            obj_point = centers[2]
            w = np_img.shape[1]
            error_value = obj_point[0] - w//2

            self.pub_alignment_error.publish(error_value)
            
            if self.show_img:
                self.draw(np_img, centers)

        else:
             self.bouys_found = False
        
        self.pub_bouys_found.publish(self.bouys_found)
        
        if self.show_img:
            self.pub_img.publish(self.bridge.cv2_to_imgmsg( np_img, "bgr8" ))
            self.pub_mask.publish(self.bridge.cv2_to_imgmsg( dict_rects[f"mask_{self.mask_cfg.set_color}"]))


    def get_centers(self, dict_rects):
        centers = []
            
        for color in ["red", "green"]:
            points =  np.array(list(map(lambda rect: (rect[0]+rect[2]//2, rect[1]+ rect[3]//2), dict_rects[color])))
            pos = np.where(points[:,1:]==np.min(points[:,1:]))[0]
            centers.append(points[pos[0]])
        
        obj_point = [(centers[0][0]+centers[1][0])//2, (centers[0][1]+centers[1][1])//2]

        centers.append(obj_point)

        return centers

            
    def draw(self, np_img, centers):
        h, w = np_img.shape[:2]
        centers.append([w//2, centers[2][1]])

        cv2.line(np_img, tuple(centers[0]), tuple(centers[1]),(210,210,210),4)
        
        for n, color in enumerate([(0,0,255), (0,255,0),(255,255,255),(0,0,0)]):
            cv2.circle(np_img,tuple(centers[n]),5,color,-1)


if __name__ == '__main__':
  task0_vision = Task0Vision(show_img = True, show_rects=True)