import cv2
import numpy as np
import time

from dynamic_reconfigure.server import Server as DRServer
from task0_pkg.cfg import HSVConfig

class Task0ImgProcessing():
    
    def __init__(self, draw_rect=False, min_area= 5):
       self.colors = {"green":{"rgb": (0, 255, 0),
                               "hsv_min": (62, 101, 55), 
                               "hsv_max": (88, 119, 215)}, 
                      "red":{"rgb": (0, 0, 255), 
                             "hsv_min": (0, 141, 60), 
                             "hsv_max": (7, 171, 136)}}
       
       self.min_area = min_area
       
       self.draw_rect = draw_rect
       
    def getContours(self, mask_img):
        # height, width, channels = img.shape
        h, w = mask_img.shape
        
        # get all contours in the image
        contours, hierarchy = cv2.findContours(mask_img,
                                              cv2.RETR_EXTERNAL, 
                                              cv2.CHAIN_APPROX_SIMPLE)
        
        # filter contours with area > min_area and that are not touching the edge of the image
        filter_contours = filter(lambda x: cv2.contourArea(x) > self.min_area
                              and np.max(x[:, :, 0]) < w - 1 
                              and np.max(x[:, :, 1]) < h - 1
                              and np.min(x) > 0, 
                              contours)
       #gey the bounding rectangle for each contour
        list_rect = list(map(lambda x : cv2.boundingRect(x), filter_contours))
       
        return list_rect
    
    def processing(self, img):
        #1 Transform from BGR to HSV
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        dict_rects = {}
        
        for color in self.colors.keys():
            #2 Filter by color
            mask = cv2.inRange(imgHSV, self.colors[color]["hsv_min"], self.colors[color]["hsv_max"])
            
            #3 Apply Gaussian filter
            mask = cv2.GaussianBlur(mask, (3, 3), 0)
            
            #5 Get contours
            rects = self.getContours(mask)
            
            #6 Draw rectangles if is aviable
            if self.draw_rect:
                map(lambda rect: cv2.rectangle(img, rect, self.colors[color]["rgb"], 2),  rects)
            
            #7 register rectangles
            dict_rects[color] = rects
            dict_rects[f"mask_{color}"] = mask
        
        return dict_rects
            

class MaskConfigHSV():
    def __init__(self, processing_class):
        self.processing_class = processing_class
        srv = DRServer(HSVConfig, self.callback)
        self.set_color = "red"
        
    def callback(self, cfg, level):
        self.set_color = cfg["color"]
        self.processing_class.colors[self.set_color]["hsv_min"] = (cfg["h_min"],cfg["s_min"],cfg["v_min"])
        self.processing_class.colors[self.set_color]["hsv_max"] = (cfg["h_max"],cfg["s_max"],cfg["v_max"])
      
        return cfg
        

class FPS:
  """Calculate and display the current FPS."""

  def __init__(self):
    # Variables to calculate FPS
    self.fps_avg_frame_count = 10
    self.frames = 0
    self.__fps = 0

  def start(self):
    self.start_time = time.time()

  def update(self):
    self.frames += 1

  def fps(self):

    if self.frames % self.fps_avg_frame_count == 0:
        end_time = time.time()
        self.__fps = self.fps_avg_frame_count / (end_time - self.start_time)
        self.start()

    return self.__fps
