#!/usr/bin/env python3

import rospy
import torch
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class YOLOv5Detect():
  
  def __init__(self):
   
    rospy.init_node( 'yolov5_detect' )
    self.variables_init()
    self.connections_init()
    rospy.spin()


  def variables_init(self):

    self.bridge = CvBridge()
    self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s') 
    #self.model.cpu()

  def connections_init(self):
    self.pub_detect =rospy.Publisher(f'{rospy.get_name()}/img', Image, queue_size=10)
    rospy.Subscriber(f'/sensor/logitech0/img', Image, self.detect)


  def detect(self, frame_msg):
    frame = self.bridge.imgmsg_to_cv2(frame_msg,"bgr8")
    
    results = self.model(frame, size = 320)
        
    data = results.pandas().xyxy[0]
    
    if not data.empty:
        for i in range(len(data)):
          xmin, ymin = int(data["xmin"][i]), int(data["ymin"][i])
          xmax, ymax = int(data["xmax"][i]), int(data["ymax"][i])
    
          cv2.rectangle(frame,(xmin,ymin),(xmax,ymax),(0,255,0),1)

    frame_msg = self.bridge.cv2_to_imgmsg(frame,"bgr8")
    self.pub_detect.publish(frame_msg)


if __name__ == '__main__':
  yolov5_detect = YOLOv5Detect()