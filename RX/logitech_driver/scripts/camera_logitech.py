#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cameras import USBCamera

class CameraLogiTech():
  
  def __init__(self):
   
    rospy.init_node( 'camera_logitech' )
    self.variables_init()
    self.connections_init()
    self.run()


  def variables_init(self):

    self.bridge = CvBridge()
    self.cam_id = rospy.get_param( f'{rospy.get_name()}/cam_id' )
    self.camera = USBCamera(camera_id = self.cam_id)


  def connections_init(self):
    self.pub_map =rospy.Publisher(f'sensor{rospy.get_name()}/img', Image, queue_size=10)


  def run(self):

    while True:
        frame = self.camera.get_cap()
        frame_msg = self.bridge.cv2_to_imgmsg(frame,"bgr8")
        self.pub_map.publish(frame_msg)


if __name__ == '__main__':
  cameralogitech = CameraLogiTech()