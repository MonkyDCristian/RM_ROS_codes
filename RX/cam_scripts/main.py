import cv2
from cameras import USBCamera, FPS


def main():
    camera = USBCamera(camera_id=2)
    fps = FPS()

    running = True

    fps.start()

    while running:

        frame = camera.get_cap()

        if type(frame) != bool:
            fps.update()
            print(f"fps = {fps.fps()}")
           
            cv2.imshow('frame', frame)
            cv2.waitKey(1)
            
        else:
            running = False

if __name__ == '__main__':
    main()
