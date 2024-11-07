import cv2

path = r"C:\Users\lenovo\Downloads\qr.jpg"
image = cv2.imread(path)
detector = cv2.QRCodeDetector()

data,points,straight_qr = detector.detectAndDecode(image)

if data:
    print(f'Detected QR code data: {data}')

    if points is not None:
        points = points[0].astype(int) #1st row ma corner na cordinates hoy tene int ma convert kaira as required for drawing function in opencv
        for i in range(4):
            cv2.line(image, tuple(points[i]), tuple(points[(i + 1) % 4]), (0, 255, 0), 5)


cv2.imshow('qr from image', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
