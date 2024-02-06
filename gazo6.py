import cv2
from picamera2 import Picamera2
from datetime import datetime
import numpy as np
from libcamera import controls

def makecontent():
    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    content = f"[{current_time}]"
    return content

def cam():
    with Picamera2() as camera:
        camera.start()
        camera.resolution = (640, 480)
        camera.set_controls({"AfMode": controls.AfModeEnum.Continuous})
        
        while True:
            im = camera.capture_array()
            cv2.imshow("camera", im)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
        cv2.destroyAllWindows()
        
        img = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = np.array([145, 50, 0])
        upper = np.array([160, 255, 255])
        frame_mask = cv2.inRange(hsv, lower, upper)
        
        dst = cv2.bitwise_and(img, img, mask=frame_mask)
        cv2.imwrite('otameshi.jpg', dst)
        
        img = cv2.imread('otameshi.jpg', 0)
        if img is None:
            print("Failed to read processed image 'otameshi.jpg'")
            return None 

        retl, img_th = cv2.threshold(img, 0, 255, cv2.THRESH_OTSU)
        hole_area = img_th.size
        yellow_area = cv2.countNonZero(img_th)
        pink = yellow_area / hole_area * 100
        print('pink_Area=' + str(pink) + '%')
        
        with open("log.csv", "a", encoding='utf-8') as file:
            file.write(makecontent() + ',' + 'data' + ',' + 'para' + str(pink) + '\n')

        return pink

result = cam()
if result is not None:
    print("Process completed successfully.")
else:
    print("Error during processing.")
