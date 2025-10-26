import cv2
image = cv2.imread(r'C:\Users\lenovo\Downloads\qr.jpg')
detector = cv2.QRCodeDetector()

data, points, var = detector.detectAndDecode(image)

if data:
    print(f'Detected QR code data: {data}')

    if points is not None:
        points = points[0].astype(int)
        for i in range(4):
            cv2.line(image, tuple(points[i]), tuple(points[(i + 1) % 4]), (0, 255, 0), 3)


cv2.imshow('qr from image', image)
cv2.waitKey(0)
cv2.destroyAllWindows()