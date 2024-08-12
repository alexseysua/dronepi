import time
import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C

i2c = busio.I2C(board.SCL, board.SDA)
bno = BNO08X_I2C(i2c)

print("Initialization successful")

while True:
    time.sleep(1)
    try:
        accel_x, accel_y, accel_z = bno.acceleration
        print(f"Accel: X={accel_x} Y={accel_y} Z={accel_z}")
    except Exception as e:
        print(f"Error: {e}")
 