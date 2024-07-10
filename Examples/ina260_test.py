from ina260.controller import Controller

c = Controller(address=0x40)

while True:
    print(c.voltage())
    print(c.current())
    print(c.power())
