import spidev
import time

# Open SPI bus
spi = spidev.SpiDev()
spi.open(0, 0)  # Use SPI bus 0, device 0

try:
    while True:
        # Read data from SPI bus
        data = spi.xfer([0x00])  # Read one byte from SPI bus
        
        # Print the received data
        print("Received:", data)

        # Wait for a while before reading again
        time.sleep(1)

except KeyboardInterrupt:
    # Close SPI bus on Ctrl+C
    spi.close()
