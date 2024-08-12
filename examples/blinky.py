import time
import gpiozero

print("hello blinky!")

led = gpiozero.LED(17)

while True:
    led.on()
    time.sleep(0.5)
    led.off()
    time.sleep(0.5)
