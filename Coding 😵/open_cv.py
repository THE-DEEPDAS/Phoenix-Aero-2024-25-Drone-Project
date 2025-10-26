import cv2
import matplotlib.pyplot as plt

# Read the image from the desktop (replace 'image_path' with your image file path)
image_path = r'C:\Users\lenovo\Pictures\celestial-pulse-light-creating-doorway-galaxy_579873-14868.jpg'
image = cv2.imread(image_path)

# Check if the image was successfully loaded
if image is None:
    print("Error: Image not found or could not be loaded.")
else:
    # Display the original image
    cv2.imshow('Original Image', image)
    cv2.waitKey(0)

    # Convert the image to grayscale
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Display the grayscale image
    cv2.imshow('Grayscale Image', gray_image)
    cv2.waitKey(0)

    # Detect edges using the Canny edge detector
    edges = cv2.Canny(gray_image, 100, 200)

    # Display the edges
    cv2.imshow('Edges', edges)
    cv2.waitKey(0)

    # Load the pre-trained Haar Cascade classifier for face detection
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

    # Detect faces in the grayscale image
    faces = face_cascade.detectMultiScale(gray_image, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    # Draw rectangles around the faces
    for (x, y, w, h) in faces:
        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)

    # Display the image with detected faces
    cv2.imshow('Faces', image)
    cv2.waitKey(0)

    # Close all OpenCV windows
    cv2.destroyAllWindows()

    # Save the processed images (optional)
    cv2.imwrite('grayscale_image.jpg', gray_image)
    cv2.imwrite('edges.jpg', edges)
    cv2.imwrite('faces_detected.jpg', image)
