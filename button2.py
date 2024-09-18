import machine, time
import neopixel
import uasyncio as asyncio
from mqtt import MQTTClient
from machine import I2C, Pin, PWM
import struct
from MSA311 import Acceleration
import random, network

# MQTT setup
mqtt_broker = 'broker.hivemq.com'
port = 1883
topic_sub = 'ME35-24/carlo'

# NeoPixel and button setup
led_pin = Pin(28, Pin.OUT)
button = Pin(12, Pin.IN, Pin.PULL_UP)
led = neopixel.NeoPixel(led_pin, 1)

# LED PWM setup
blue_led = PWM(Pin(7, machine.Pin.OUT))
blue_led.freq(50)

# Buzzer PWM setup
buzzer = PWM(Pin('GPIO18', Pin.OUT))

# Set up accelerometer
scl = Pin(27, Pin.OUT)
sda = Pin(26, Pin.OUT)
accelerometer = Acceleration(scl, sda)

current_color =(255, 0, 0)

            
# Turn off function
def turnoff():
    blue_led.duty_u16(0)
    led[0] = (0, 0, 0)
    led.write()


# Asynchronous task to control the NeoPixel based on accelerometer input
async def control_led():
    global current_color
    while True:

        t = Acceleration(scl, sda)
        y = t.read_accel()[1]

        r = current_color[0]
        g = current_color[1]
        b = current_color[2]


        # Check if the accelerometer indicates a press
        if y < -17000:
            # Tap detected - dim the NeoPixel
            led.write()
            for i in range(255, 0, -5):
                led[0] = (r * i // 255, g * i // 255, b * i // 255)  # Example: Red color
                led.write()
                await asyncio.sleep(0.02)
            buzzer.freq(440)
            buzzer.duty_u16(1000)
            await asyncio.sleep(0.5)
            buzzer.duty_u16(0)
                
            # Dim the light back down
            for i in range(0, 255, 5):
                led[0] = (r * i // 255, g * i // 255, b * i // 255)
                led.write()
                await asyncio.sleep(0.02)

        await asyncio.sleep(0.1)

# Asynchronous task for button control

async def button_control():
    global current_color
    while True:
        if not button.value():  # Button pressed
            red = random.randint(0,255)
            green = random.randint(0, 255)
            blue = random.randint (0, 255)

            current_color = (red, green, blue)

            led[0] = current_color  # Random color selection
            led.write()
        await asyncio.sleep(0.1)

# Asynchronous task for breathing LEDs
async def breathing():
    while True:
        for i in range(0,65535,500):
                blue_led.duty_u16(i)     #  u16 means unsighed 16 bit integer (0-65535)
                await asyncio.sleep(0.02)

        

# Main function to run the asynchronous tasks
async def main():
    task3 = asyncio.create_task(breathing())
    task1 = asyncio.create_task(control_led())
    task2 = asyncio.create_task(button_control())
    await asyncio.gather(task1,task2,task3)
    
    

try:
    asyncio.run(main())
finally:
    turnoff()
