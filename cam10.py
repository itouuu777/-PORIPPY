import cv2
from picamera2 import Picamera2
from libcamera import controls

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()

current_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
current_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
print(f"Œ»İ‚Ì‰ğ‘œ“x: {current_width}x{current_height}")

picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})

while True:
  im = picam2.capture_array()
  cv2.imshow("Camera", im)

  while True:
    ret, frame = cap.read()
    if not ret:
        print("‰f‘œ‚Ìæ“¾‚É¸”s‚µ‚Ü‚µ‚½B")
        break

   
    cv2.imshow('Frame', frame)

  
    current_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    current_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    print(f"Œ»İ‚Ì‰ğ‘œ“x: {current_width}x{current_height}")

  
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
 
  key = cv2.waitKey(1)

  if key == 27:
    break
cap.release()
picam2.stop()
cv2.destroyAllWindows()

