# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# Outputs a 50% duty cycle PWM single on the 0th channel.
# Connect an LED and resistor in series to the pin
# to visualize duty cycle changes and its impact on brightness.
import time
import board
from adafruit_pca9685 import PCA9685

# Create the I2C bus interface.
i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = busio.I2C(board.GP1, board.GP0)    # Pi Pico RP2040

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)

# Set the PWM frequency to 60hz.
pca.frequency = 60

# Set the PWM duty cycle for channel zero to 50%. duty_cycle is 16 bits to match other PWM objects
# but the PCA9685 will only actually give 12 bits of resolution.

led_channel = pca.channels[8]

# Increase brightness:
for i in range(65535):
    time.sleep(0.01)
    led_channel.duty_cycle = i
     
# Decrease brightness:
for i in range(65535, 0, -1):
    time.sleep(0.01)
    led_channel.duty_cycle = i

led_channel = pca.channels[9]

# Increase brightness:
for i in range(65535):
    time.sleep(0.01)
    led_channel.duty_cycle = i
     
# Decrease brightness:
for i in range(65535, 0, -1):
    time.sleep(0.01)
    led_channel.duty_cycle = i

led_channel = pca.channels[10]

# Increase brightness:
for i in range(65535):
    time.sleep(0.01)
    led_channel.duty_cycle = i
     
# Decrease brightness:
for i in range(65535, 0, -1):
    time.sleep(0.01)
    led_channel.duty_cycle = i
