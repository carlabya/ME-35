#breathing
import machine, time
import neopixel
from machine import Pin

blue = machine.PWM(machine.Pin('GPIO0', machine.Pin.OUT))
state = (10,0,10)  # RGB

led = neopixel.NeoPixel(Pin(28),1)
led[0] = state
led.write()

while True:
    blue.freq(50)
    for i in range(0,65535,500):
        blue.duty_u16(i)     #  u16 means unsighed 16 bit integer (0-65535)
        time.sleep(0.02)
        i = 0
        


