#!/usr/bin/env python
import time
import argparse
from pmw3901 import PMW3901, PAA5100, BG_CS_FRONT_BCM, BG_CS_BACK_BCM

print("""motion.py - Detect flow/motion in front of the PMW3901 sensor.

Press Ctrl+C to exit!
""")

parser = argparse.ArgumentParser()
parser.add_argument('--rotation', type=int,
                    default=0, choices=[0, 90, 180, 270],
                    help='Rotation of sensor in degrees.', )
args = parser.parse_args()
flo = PMW3901(spi_port=0, spi_cs_gpio=BG_CS_FRONT_BCM)
flo.set_rotation(args.rotation)
tx = 0
ty = 0
try:
    prev_time = 0.0
    curr_time = time.time()
    while True:
        try:
            x, y = flo.get_motion()
        except RuntimeError:
            continue
        tx += x
        ty += y
        print("Motion: %0.2f %0.2f x: %0.2f y %0.2f" % (x, y, tx, ty))
        time.sleep(0.001)
        curr_time = time.time()
        FPS = 1 / (curr_time - prev_time)
        print("\nFPS: %0.1f" % FPS,"\n")
        prev_time = curr_time
except KeyboardInterrupt:
    pass

