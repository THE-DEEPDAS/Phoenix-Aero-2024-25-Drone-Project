import cv2
import numpy as np

video_file = r"C:\Users\lenovo\Videos\Captures\screen recorder for windows inbuilt - Google Search - Google Chrome 2024-05-26 13-00-41.mp4"
cap = cv2.VideoCapture(video_file)

ret, frame = cap.read()
if not ret:
    print("Error: Failed to read video file")
    exit()

# Select ROI for tracking
bbox = cv2.selectROI("Select Object to Track", frame, fromCenter=False, showCrosshair=True)
track_window = (bbox[0], bbox[1], bbox[2], bbox[3])

# Initialize CamShift tracker
roi = frame[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2]]
hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv_roi, np.array((0., 60., 32.)), np.array((180., 255., 255.)))
roi_hist = cv2.calcHist([hsv_roi], [0], mask, [180], [0, 180])
cv2.normalize(roi_hist, roi_hist, 0, 255, cv2.NORM_MINMAX)

# Setup the termination criteria, either 10 iterations or move by at least 1 pixel
term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    dst = cv2.calcBackProject([hsv], [0], roi_hist, [0, 180], 1)
    
    # Apply CamShift to get the new location
    ret, track_window = cv2.CamShift(dst, track_window, term_crit)
    
    # Draw the tracked object as a bounding box
    pts = cv2.boxPoints(ret)
    pts = np.int0(pts)
    img = cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
    
    # Display the frame with the tracked object
    cv2.imshow('Tracked Object', img)
    
    # Exit if ESC pressed
    if cv2.waitKey(30) & 0xFF == 27:
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
