import cv2
import rospy 
from sensor_msgs.msg import Image

from task0_vision_tools import Task0ImgProcessing, TrackbarHSV, FPS

class Task0Vision():
    def __init__(self, use_trackbar=False):
        self.tip = Task0ImgProcessing(draw_rect=False, min_area=5)
       
        if use_trackbar:
            self.t_hsv_green = TrackbarHSV(self.tip,"green")
            self.t_hsv_red = TrackbarHSV(self.tip, "red")
        
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.pub_center = rospy.Publisher("task0_master", Float32, queue_size=0)
        
    def callback(self, img):
        
        
        