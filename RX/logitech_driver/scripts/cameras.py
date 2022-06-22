import sys, time, cv2

class USBCamera:
    def __init__(self, camera_id: int = 0):
        """Initialize the object detection routine.

        Args:
          camera_id: The camera id to be passed to OpenCV.
          width: The width of the frame captured from the camera.
          height: The height of the frame captured from the camera.
        """
        self.cap = cv2.VideoCapture(camera_id)

    def get_cap(self):
        if self.cap.isOpened():
          success, frame = self.cap.read()
          
          if not success:
            sys.exit('ERROR: Unable to read from webcam. Please verify your webcam settings.')

          return cv2.flip(frame, 1)


class IMX219Camera:
    def __init__(self, cap_width:int=1280, cap_height:int =720,
                       dis_width:int =1280, dis_height:int =720,
                       framerate:int =60,flip_method:int =0):
        """Initialize the object detection routine.

        Args:
          camera_id: The camera id to be passed to OpenCV.
          width: The width of the frame captured from the camera.
          height: The height of the frame captured from the camera.
        """
        self.cap_width, self.cap_height = cap_width, cap_height 
        self.dis_width, self.dis_height = dis_width, dis_height
        self.framerate, self.flip_method = framerate, flip_method

        self.cap = cv2.VideoCapture(self.gstreamer_pipeline(), cv2.CAP_GSTREAMER)

    def gstreamer_pipeline(self):
        return (
                "nvarguscamerasrc ! "
                "video/x-raw(memory:NVMM), "
                "width=(int)%d, height=(int)%d, "
                "format=(string)NV12, framerate=(fraction)%d/1 ! "
                "nvvidconv flip-method=%d ! "
                "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
                "videoconvert ! "
                "video/x-raw, format=(string)BGR ! appsink"
                % (
                    self.cap_width,
                    self.cap_height,
                    self.framerate,
                    self.flip_method,
                    self.dis_width,
                    self.dis_height,
                  )
               )

    def get_cap(self):
        if self.cap.isOpened():
            success, frame = self.cap.read()

            if not success:
                sys.exit(
                    'ERROR: Unable to read from webcam. Please verify your webcam settings.')

            return cv2.flip(frame, 1)
            

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

