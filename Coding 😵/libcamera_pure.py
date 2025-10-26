import subprocess

def start_camera():
    process = subprocess.Popen(["libcamera-vid", "--qt-preview", "-t", "0"])
    return process

def stop_camera(process):
    process.terminate()
    process.wait()

camera_process = start_camera()
input("Kai pan dabavo stop karva mate...") 
stop_camera(camera_process)
