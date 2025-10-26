import cv2
import numpy as np
import os
import time  # Ensure time module is imported

# Constants for drone height calculations
DRONE_HEIGHT = 10  # average height in meters
CAMERA_FOV = 70  # field of view in degrees
PERSON_HEIGHT = 1.7  # average person height in meters

# Add new constants
MIN_DETECTION_CONFIDENCE = 0.8
SUSPICIOUS_OBJECT_TYPES = {
    'bag_like': {'min_area_meters': 0.3, 'max_area_meters': 1.0},  # For suspicious packages
    'stone': {'note': "Detection now based on candidate shape; evaluated later with human overlap"}
}

# Add new constants
YOLO_CONFIG = {
    'config_path': 'yolov3-tiny.cfg',  # Ensure these files are in the working directory
    'weights_path': 'yolov3-tiny.weights',
    'conf_threshold': 0.5,
    'nms_threshold': 0.4
}

# Add MobileNet-SSD model paths
MOBILENET_SSD_CONFIG = {
    'prototxt_path': 'deploy.prototxt.txt',  # Ensure these files are in the working directory
    'model_path': 'mobilenet_iter_73000.caffemodel',
    'conf_threshold': 0.5
}

# Create photos directory for saving threat images
photos_dir = "c:/Users/lenovo/Downloads/threat_photos"
os.makedirs(photos_dir, exist_ok=True)

# Calculate pixels per meter based on drone height
def calculate_pixels_per_meter(frame_height, altitude):
    angle_rad = np.deg2rad(CAMERA_FOV/2)
    ground_height = 2 * altitude * np.tan(angle_rad)
    return frame_height / ground_height

# Modified detection function: return Stone Candidate if contour is near-square
def detect_suspicious_object(contour, pixels_per_meter):
    """Improved suspicious object detection"""
    x, y, w, h = cv2.boundingRect(contour)
    aspect_ratio = max(w, h) / min(w, h)
    area_meters = cv2.contourArea(contour) / (pixels_per_meter ** 2)
    
    # Stone candidate: allow any size (peanut to watermelon) if nearly square
    if 0.7 <= aspect_ratio <= 1.3:
        return True, "Stone Candidate"
    
    # Detect suspicious packages (bag_like) using existing area thresholds
    if (SUSPICIOUS_OBJECT_TYPES['bag_like']['min_area_meters'] < area_meters < 
        SUSPICIOUS_OBJECT_TYPES['bag_like']['max_area_meters']):
        return True, "Suspicious Package"
    
    return False, ""

# HAAR cascade with adjusted parameters
haar_cascade_path = "c:/path/to/haarcascade_fullbody.xml"  # Update this path to the correct location
haar_cascade = cv2.CascadeClassifier(haar_cascade_path)

# HOG descriptor with adjusted parameters
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

# Initialize YOLO
net = cv2.dnn.readNetFromDarknet(YOLO_CONFIG['config_path'], YOLO_CONFIG['weights_path'])
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

# Load MobileNet-SSD model
net = cv2.dnn.readNetFromCaffe(MOBILENET_SSD_CONFIG['prototxt_path'], MOBILENET_SSD_CONFIG['model_path'])

cap = cv2.VideoCapture(0)

prev_positions = {}
prev_contours = []

# Function to detect throwing motion
def detect_throwing_motion(current_pose, prev_pose):
    if not prev_pose:
        return False
    
    # Calculate hand movement speed
    hand_speed = distance.euclidean(
        current_pose['right_hand'],
        prev_pose['right_hand']
    ) / (1/30)  # Assuming 30 fps
    
    # Detect throwing motion pattern
    is_throwing = (
        hand_speed > 2.0 and  # Fast hand movement
        current_pose['right_hand'][1] < prev_pose['right_hand'][1]  # Upward movement
    )
    
    return is_throwing

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Calculate scaling factor based on drone height
    pixels_per_meter = calculate_pixels_per_meter(frame.shape[0], DRONE_HEIGHT)
    min_person_height_pixels = int(PERSON_HEIGHT * pixels_per_meter * 0.7)  # 70% of expected height
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Improve HAAR cascade detection
    haar_detections = haar_cascade.detectMultiScale(
        gray,
        scaleFactor=1.03,  # More precise scaling
        minNeighbors=8,    # Stricter detection
        minSize=(min_person_height_pixels//3, min_person_height_pixels),
        flags=cv2.CASCADE_SCALE_IMAGE
    )

    # Adjusted HOG parameters
    hog_detections, _ = hog.detectMultiScale(
        frame,
        winStride=(8, 8),
        padding=(16, 16),
        scale=1.03
    )

    # Combine HOG and HAAR detections with confidence scoring
    all_detections = []
    for (x, y, w, h) in haar_detections:
        confidence = w * h / (frame.shape[0] * frame.shape[1])
        if confidence > MIN_DETECTION_CONFIDENCE:
            all_detections.append(('person', x, y, w, h, confidence))

    for (x, y, w, h) in hog_detections:
        confidence = w * h / (frame.shape[0] * frame.shape[1])
        if confidence > MIN_DETECTION_CONFIDENCE:
            all_detections.append(('person', x, y, w, h, confidence))

    # Non-maximum suppression to remove overlapping detections
    final_detections = []
    if all_detections:
        boxes = np.array([[x, y, x + w, y + h] for (_, x, y, w, h, _) in all_detections])
        confidences = np.array([conf for (_, _, _, _, _, conf) in all_detections])
        indices = cv2.dnn.NMSBoxes(boxes.tolist(), confidences.tolist(), 0.5, 0.4)
        for i in indices:
            final_detections.append(all_detections[i[0]])

    intruder_detected = False
    alert_message = ""

    # Process validated detections
    current_positions = {}
    for detection in final_detections:
        _, x, y, w, h, confidence = detection
        person_id = f"person_{len(current_positions)}"
        current_positions[person_id] = (x, y, w, h)

        # Validate size and process threats
        if 0.7 * min_person_height_pixels <= h <= 1.3 * min_person_height_pixels:
            threat_level = 0
            threat_reasons = []

            # Check for raised hands
            if y < frame.shape[0] // 4 and h > min_person_height_pixels * 0.8:
                threat_level += 2
                threat_reasons.append("Raised Arms")

            # Check for rapid movement
            if person_id in prev_positions:
                prev_x, prev_y, _, _ = prev_positions[person_id]
                movement = np.sqrt((x - prev_x)**2 + (y - prev_y)**2) / pixels_per_meter
                if movement > 2:  # Moving faster than 2 meters per frame
                    threat_level += 2
                    threat_reasons.append("Rapid Movement")

            # Visualize detection with threat level
            if threat_level > 0:
                intruder_detected = True
                color = (0, 0, 255)  # Red for threats
                thickness = 3
                alert_message = f"⚠️ Alert: {', '.join(threat_reasons)}"
            else:
                color = (0, 255, 0)  # Green for normal detection
                thickness = 2

            # Draw detection box and confidence
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, thickness)
            cv2.putText(frame, f"Conf: {confidence:.2f}", (x, y-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    # Improved suspicious object detection
    edges = cv2.Canny(gray, 100, 200)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours:
        detected, object_type = detect_suspicious_object(cnt, pixels_per_meter)
        if detected:
            x, y, w, h = cv2.boundingRect(cnt)
            # Process stone candidate only if a person holds it
            if object_type == "Stone Candidate":
                stone_center = (x + w//2, y + h//2)
                for det in final_detections:
                    _, px, py, pw, ph, _ = det
                    # Check if stone candidate center is within person's bounding box
                    if px <= stone_center[0] <= px+pw and py <= stone_center[1] <= py+ph:
                        # Check if the stone is moving
                        if cnt not in prev_contours:
                            intruder_detected = True
                            alert_message = "⚠️ Alert: Stone Threat Detected"
                            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 3)
                            cv2.putText(frame, "Stone Held", (x, y-10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                            # Save the photo of the person holding the stone
                            person_img = frame[py:py+ph, px:px+pw]
                            img_path = os.path.join(photos_dir, f"threat_{px}_{py}_{int(time.time())}.jpg")
                            cv2.imwrite(img_path, person_img)
                            break
            # Process package detection as before
            elif object_type == "Suspicious Package":
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 3)
                cv2.putText(frame, f"Suspicious: {object_type}", (x, y-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                intruder_detected = True
                alert_message = f"⚠️ Alert: Suspicious {object_type} Detected"

    # Replace YOLO detection with MobileNet-SSD detection
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Prepare input blob for MobileNet-SSD
        blob = cv2.dnn.blobFromImage(frame, 0.007843, (300, 300), 127.5)
        net.setInput(blob)
        detections = net.forward()

        # Process detections
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > MOBILENET_SSD_CONFIG['conf_threshold']:
                class_id = int(detections[0, 0, i, 1])
                if class_id == 15:  # Class ID 15 corresponds to 'person' in MobileNet-SSD
                    box = detections[0, 0, i, 3:7] * np.array([frame.shape[1], frame.shape[0], frame.shape[1], frame.shape[0]])
                    (x, y, x_max, y_max) = box.astype("int")
                    cv2.rectangle(frame, (x, y), (x_max, y_max), (0, 255, 0), 2)
                    label = f"Person: {confidence:.2f}"
                    cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # Detect throwing motion (placeholder for pose estimation logic)
                    # current_pose = {'right_hand': (x_max, y_max)}  # Example hand position
                    # if detect_throwing_motion(current_pose, prev_positions.get('person_0')):
                    #     cv2.putText(frame, "⚠️ Throwing Detected", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    #     cv2.rectangle(frame, (x, y), (x_max, y_max), (0, 0, 255), 3)

        prev_positions = current_positions.copy()
        prev_contours = contours

        if intruder_detected:
            cv2.putText(frame, alert_message, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(frame, f"Height: {DRONE_HEIGHT}m", (50, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.imshow("Drone Surveillance", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()