import time
import argparse
from pmw3901 import PMW3901, PAA5100, BG_CS_FRONT_BCM, BG_CS_BACK_BCM

print("""frame_capture.py - Capture the raw frame data from the PMW3901

Press Ctrl+C to exit!
""")

# Pick the right class for the specified breakout
SensorClass = PMW3901 

# For direct connection to Raspberry Pi, you can just use default SPI settings
flo = SensorClass(spi_port=0, spi_cs=1, spi_cs_gpio=BG_CS_FRONT_BCM)
flo.set_rotation(0) # choices=[0, 90, 180, 270], default=0, Rotation of sensor in degrees.

def value_to_char(value):
    charmap = [" ", "░", "▒", "▓", "█"]
    value /= 255
    value *= len(charmap) - 1
    value = int(value)
    return charmap[value] * 2  # Double chars to - sort of - correct aspect ratio


try:
    while True:
        print("Capturing...")
        data = flo.frame_capture()
        for y in range(35):
            y = 35 - y - 1 if 0 in (180, 270) else y
            for x in range(35):
                x = 35 - x - 1 if 0 in (180, 90) else x
                if 0 in (90, 270):
                    offset = (x * 35) + y
                else:
                    offset = (y * 35) + x
                value = data[offset]
                print(value_to_char(value), end="")
            print("")
        print("5...")
        time.sleep(1.0)
        print("4...")
        time.sleep(1.0)
        print("3...")
        time.sleep(1.0)
        print("2...")
        time.sleep(1.0)
        print("Get Ready!")
        time.sleep(1.0)

except KeyboardInterrupt:
    pass