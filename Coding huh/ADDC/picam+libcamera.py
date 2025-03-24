from picamera2 import Picamera2
import cv2

picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("Preview")
picam2.start()

while True:
    frame = picam2.capture_array()
    cv2.imshow("Aane live feed kevay", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'): 
        break

cv2.destroyAllWindows()
picam2.stop()
