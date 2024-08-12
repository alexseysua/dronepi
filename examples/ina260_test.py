from ina260.controller import Controller
import time

c = Controller(address=0x40)

prev_time = 0.0
curr_time = time.time()
while True:
    print(c.voltage())
    print(c.current())
    print(c.power())
    
    curr_time = time.time()
    FPS = 1 / (curr_time - prev_time)
    print("\nFPS: %0.2f" % FPS,"\n")
    prev_time = curr_time
    time.sleep(0.001)  # Avoid polling *too* fast
