import time
import numpy
import board
import busio
import RPi.GPIO as GPIO
import vl53l5cx_ctypes as vl53l5cx
import argparse
from picamera2 import Picamera2
from pmw3901 import PMW3901, BG_CS_FRONT_BCM, BG_CS_BACK_BCM
from ina260.controller import Controller
from adafruit_pca9685 import PCA9685
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

# GPIO.setmode(GPIO.BCM)
# # GPIO.setwarnings(False) dont' use this.
# GPIO.setup(27, GPIO.OUT)
# GPIO.output(27, GPIO.HIGH)

IS_CAMERA = False

''' DISTANCE SENSOR SETUP '''
print("Uploading firmware to VL53L5CX, please wait...")
vl53 = vl53l5cx.VL53L5CX()
time.sleep(1)
print("Done!")
vl53.set_ranging_mode(vl53l5cx.RANGING_MODE_CONTINUOUS)
vl53.set_resolution(4 * 4)
time.sleep(1)

# This is a visual demo, so prefer speed over accuracy
vl53.set_ranging_frequency_hz(90)
time.sleep(1)

vl53.set_integration_time_ms(2)
time.sleep(1)
vl53.start_ranging()
time.sleep(1)

''' MOTION SENSOR SETUP '''
parser = argparse.ArgumentParser()
parser.add_argument('--rotation', type=int,
                    default=0, choices=[0, 90, 180, 270],
                    help='Rotation of sensor in degrees.', )
args = parser.parse_args()
flo = PMW3901(spi_port=0, spi_cs_gpio=BG_CS_FRONT_BCM)
flo.set_rotation(args.rotation)
tx = 0
ty = 0

''' CURRENT SENSOR SETUP '''
ina260 = Controller(address=0x40)

''' IMU SENSOR SETUP '''
i2c = busio.I2C(board.SCL, board.SDA)
bno = BNO08X_I2C(i2c)

bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_MAGNETOMETER)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

''' PWM DRIVER SETUP & LIGHTING '''
# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)
pca.frequency = 60

LED_R = pca.channels[9]
LED_G = pca.channels[8]
LED_B = pca.channels[10]

# Turn on all LEDs
LED_R.duty_cycle = 64000 # Max value is 65535 (16-bit) and off
LED_G.duty_cycle = 0     # Min value is 0 (brightest)
LED_B.duty_cycle = 40000

''' CAMERA SETUP '''
if IS_CAMERA:
    camera = Picamera2()
    camera.start_and_record_video("new_video.mp4", show_preview=False, duration = 10)

camera_timer = time.time()
prev_time = 0.0
curr_time = time.time()
while time.time() - camera_timer < 10:
    time.sleep(0.0001)
    
    ## DISTANCE SENSOR
    if vl53.data_ready():
        data = vl53.get_data()
        arr = numpy.flipud(numpy.array(data.distance_mm)).astype('float64')[0, 0:16].reshape((4, 4))
    
        # Scale view relative to the furthest distance
        distance = arr.max()

        # Scale view to a fixed distance
        distance = 512

        # Scale and clip the result to 0-255
        arr *= (255.0 / distance)
        arr = numpy.clip(arr, 0, 255)

        arr = arr.astype('uint8')
        print(arr)
    
    ## MOTION SENSOR
    try:
        x, y = flo.get_motion()
    except RuntimeError:
        continue
    tx += x
    ty += y
    print("Motion: %0.2f %0.2f, x: %0.2f y %0.2f" % (x, y, tx, ty))

    ## IMU SENSOR
    print("Acceleration:")
    accel_x, accel_y, accel_z = bno.acceleration  # pylint:disable=no-member
    print("X: %0.6f  Y: %0.6f Z: %0.6f  m/s^2" % (accel_x, accel_y, accel_z))
    print("")

    print("Gyro:")
    gyro_x, gyro_y, gyro_z = bno.gyro  # pylint:disable=no-member
    print("X: %0.6f  Y: %0.6f Z: %0.6f rads/s" % (gyro_x, gyro_y, gyro_z))
    print("")

    print("Magnetometer:")
    mag_x, mag_y, mag_z = bno.magnetic  # pylint:disable=no-member
    print("X: %0.6f  Y: %0.6f Z: %0.6f uT" % (mag_x, mag_y, mag_z))
    print("")

    print("Rotation Vector Quaternion:")
    quat_i, quat_j, quat_k, quat_real = bno.quaternion  # pylint:disable=no-member
    print(
        "I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f" % (quat_i, quat_j, quat_k, quat_real)
    )
    print("")

    ## CURRENT SENSOR
    print("Voltage: %.2f V" % ina260.voltage())
    print("Current: %.2f mA" % ina260.current())
    print("Power: %.2f mW" % ina260.power())
    print("")

    curr_time = time.time()
    FPS = 1 / (curr_time - prev_time)
    print("\nFPS: %0.2f" % FPS,"\n")
    prev_time = curr_time

if IS_CAMERA:
    camera.close()
    



