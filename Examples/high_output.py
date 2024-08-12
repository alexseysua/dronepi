import RPi.GPIO as GPIO
#27 for ToF change I2C mode.
#21 for IMU NRST pin.
GPIO.setmode(GPIO.BCM)
GPIO.setup(21, GPIO.OUT)
GPIO.output(21, GPIO.HIGH)

try:
  while True:
    time.sleep(0.1)
except:
  GPIO.cleanup()
