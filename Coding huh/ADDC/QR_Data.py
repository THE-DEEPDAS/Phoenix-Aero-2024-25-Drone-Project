from pyzbar.pyzbar import decode
from PIL import Image

def scan_qr_from_image(image_path):
    image = Image.open(image_path)  
    decoded_objects = decode(image)  

    for obj in decoded_objects:
        data = obj.data.decode('utf-8') 
        print(f"Detected QR Code: {data}")

if __name__ == "__main__":
    image_path = "qr_code.png" 
    scan_qr_from_image(image_path)
