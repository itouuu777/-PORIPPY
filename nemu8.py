import cv2
import numpy as np
from picamera2 import Picamera2
from datetime import datetime
from libcamera import controls

def make_content():
    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    content = f"[{current_time}]"
    return content

def find_nearest_point(contour, center):
    distances = [np.linalg.norm(np.array(point[0]) - np.array(center)) for point in contour]
    nearest_point_index = np.argmin(distances)
    return tuple(contour[nearest_point_index][0])

def cam():
    with Picamera2() as camera:
        try:
            camera.resolution = (3280, 2464)
            camera.start()
            
            while True:
                im = camera.capture_array()
                
                img = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
                hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                lower_red1 = np.array([0, 50, 50])
                upper_red1 = np.array([10, 255, 255])
                lower_red2 = np.array([170, 50, 50])
                upper_red2 = np.array([180, 255, 255])
                frame_mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
                frame_mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
                frame_mask_red = cv2.bitwise_or(frame_mask_red1, frame_mask_red2)

                red_contours, _ = cv2.findContours(frame_mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                if len(red_contours) > 0:
                    max_red_contour = max(red_contours, key=cv2.contourArea)
                    center = tuple(np.array(im.shape[1::-1]) // 2)
                    nearest_point = find_nearest_point(max_red_contour, center)
                    
                    
                    center_x, center_y = center
                    nearest_point = (nearest_point[0] - center_x, nearest_point[1] - center_y)
                    
                    cv2.circle(img, (center_x + nearest_point[0], center_y + nearest_point[1]), 5, (0, 255, 0), -1)
                    cv2.putText(img, f"Nearest Point: {nearest_point}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

                cv2.imshow("Object Detection", img)
                print(f"Nearest Point: [nearestpoint]", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                key = cv2.waitKey(1) & 0xFF
                if key == 27:
                    break

        finally:
            cv2.destroyAllWindows()

        return None

result = cam()
