import cv2
import numpy as np
import time

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
        
        return dict_rects
            

class TrackbarHSV():
    def __init__(self, processing_class, color):
        self.processing_class = processing_class
        self.color = color
        self.name = f"TrackBars {color}"
        self.init_trackbar()
        
    def init_trackbar(self):
        
        cv2.namedWindow(self.name)
        cv2.createTrackbar("Hue min", self.name, 0, 255, self.edit_hue_min)
        cv2.createTrackbar("Hue max", self.name, 0, 255, self.edit_hue_max)
        cv2.createTrackbar("sat min", self.name, 0, 255, self.edit_sat_min)
        cv2.createTrackbar("sat max", self.name, 255, 255, self.edit_sat_min)
        cv2.createTrackbar("val min", self.name, 0, 255, self.edit_val_min)
        cv2.createTrackbar("val max", self.name, 255, 255, self.edit_val_min)
    
    def edit_hue_min(self, value):
        self.processing_class.colors[self.color]["hsv_min"][0] = value
    
    def edit_hue_max(self, value):
        self.processing_class.colors[self.color]["hsv_max"][0] = value
    
    def edit_sat_min(self, value):
        self.processing_class.colors[self.color]["hsv_min"][1] = value
    
    def edit_sat_max(self, value):
        self.processing_class.colors[self.color]["hsv_max"][1] = value
    
    def edit_val_min(self, value):
        self.processing_class.colors[self.color]["hsv_min"][2] = value
    
    def edit_val_max(self, value):
        self.processing_class.colors[self.color]["hsv_max"][2] = value
        

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