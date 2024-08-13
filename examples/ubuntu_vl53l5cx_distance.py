import time
import numpy
from vl53l5cx.vl53l5cx import VL53L5CX

driver = VL53L5CX()

alive = driver.is_alive()
if not alive:
    raise IOError("VL53L5CX Device is not alive")

print("Initialising...")
t = time.time()
driver.init()
print(f"Initialised ({time.time() - t:.1f}s)")

# Set resolution to 4x4
driver.set_resolution(driver.VL53L5CX_RESOLUTION_4X4)

# Set ranging frequency to 90Hz
driver.set_ranging_frequency(90)

# Set integration time to 2ms
driver.set_integration_time(2)

# Set ranging mode to continuous
driver.set_ranging_mode(driver.VL53L5CX_RANGING_MODE_CONTINUOUS)

# Ranging:
driver.start_ranging()

prev_time = 0.0
curr_time = time.time()
while True:
    if driver.check_data_ready():
        ranging_data = driver.get_ranging_data()

        # As the sensor is set in 4x4 mode by default, we have a total 
        # of 16 zones to print. For this example, only the data of first zone are 
        # print

        arr = numpy.flipud(numpy.array(ranging_data.distance_mm)).astype('float64')[0,0:16]
        arr = arr.reshape((4, 4))

        # Scale view relative to the furthest distance
        distance = arr.max()

        # Scale view to a fixed distance
        distance = 512

        # Scale and clip the result to 0-255
        arr *= (255.0 / distance)
        arr = numpy.clip(arr, 0, 255)

        # Force to int
        arr = arr.astype('uint8')
        print(arr)

        curr_time = time.time()
        FPS = 1 / (curr_time - prev_time)
        print("\nFPS: %0.2f" % FPS,"\n")
        prev_time = curr_time
        time.sleep(0.001)  # Avoid polling *too* fast

        

    
