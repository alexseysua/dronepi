import smbus2
import time

# Create an I2C bus object
bus = smbus2.SMBus(1)

# VL53L5CX default I2C address
VL53L5CX_ADDRESS = 0x29

try:
    # Write to a register to check if the sensor is responsive
    bus.write_byte_data(VL53L5CX_ADDRESS, 0x00, 0x00)
    print("VL53L5CX detected at address 0x29")
except OSError as e:
    print(f"Error: {e}")
finally:
    bus.close()
