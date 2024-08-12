
#!/usr/bin/env python3

import time
import vl53l5cx_ctypes as vl53l5cx
import numpy

print("Uploading firmware, please wait...")
vl53 = vl53l5cx.VL53L5CX()
time.sleep(1)
print("Done!")
vl53.set_resolution(4 * 4)
time.sleep(1)

# This is a visual demo, so prefer speed over accuracy
vl53.set_ranging_frequency_hz(120)
time.sleep(1)
vl53.set_integration_time_ms(1)
time.sleep(1)
vl53.start_ranging()
time.sleep(1)

prev_time = 0.0
curr_time = time.time()
while True:
    #print("Capturing...")
    #print(time.time())
    if vl53.data_ready():
        #print("Data ready!")
        data = vl53.get_data()
        arr = numpy.flipud(numpy.array(data.distance_mm).reshape((8, 8))).astype('float64')

        # Scale view relative to the furthest distance
        distance = arr.max()

        # Scale view to a fixed distance
        distance = 512

        # Scale and clip the result to 0-255
        arr *= (255.0 / distance)
        arr = numpy.clip(arr, 0, 255)

        # Invert the array : 0 - 255 becomes 255 - 0
#        if INVERSE:
 #           arr *= -1
  #          arr += 255.0

        # Force to int
        arr = arr.astype('uint8')
        print(arr)

    curr_time = time.time()
    FPS = 1 / (curr_time - prev_time)
    print("\nFPS: %0.2f" % FPS,"\n")
    prev_time = curr_time
    time.sleep(0.001)  # Avoid polling *too* fast
