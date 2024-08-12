from picamera import PiCamera
from time import sleep

camera = PiCamera()

camera.start_preview()
sleep(2)
for i in range(3):
    camera.capture('/home/pi/Desktop/image%s.jpg' % i)
camera.stop_preview()
camera.close()