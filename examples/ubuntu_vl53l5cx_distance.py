import time
import numpy as np
from vl53l5cx.vl53l5cx import VL53L5CX

driver = VL53L5CX()

alive = driver.is_alive()
if not alive:
    raise IOError("VL53L5CX Device is not alive")

print("Initialising...")
t = time.time()
driver.init()
time.sleep(1)
print(f"Initialised ({time.time() - t:.1f}s)")

# Set resolution to 4x4
driver.set_resolution(4 * 4)
time.sleep(0.5)

# Set ranging frequency to 90Hz
driver.set_ranging_frequency_hz(90)
time.sleep(0.5)

#NOTE: This function causes sensor to not work. Ignore it.
# Set integration time to 2ms
#driver.set_integration_time_ms(1)
#time.sleep(0.5)

# Set ranging mode to continuous
driver.set_ranging_mode(1)
time.sleep(0.5)

# Ranging:
driver.start_ranging()
print("Start Ranging")

prev_time = 0.0
curr_time = time.time()
while True:
    if driver.check_data_ready():
        data = driver.get_ranging_data()
        arr = np.flipud(np.array(data.distance_mm[0:16]))
        arr = arr.reshape((4,4))

        # Scale view relative to the furthest distance
        distance = arr.max()

        # Scale view to a fixed distance
        distance = 512

        # Scale and clip the result to 0-255
        arr *= (255.0 / distance)
        arr = np.clip(arr, 0, 255)

        # Force to int
        arr = arr.astype('uint8')
        print(arr)

        curr_time = time.time()
        FPS = 1 / (curr_time - prev_time)
        print("\nFPS: %0.2f" % FPS,"\n")
        prev_time = curr_time
    time.sleep(0.001)  # Avoid polling *too* fast
