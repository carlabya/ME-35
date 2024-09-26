from machine import Pin, PWM
import time, neopixel, machine
from mqtt import MQTTClient
from MSA311 import Acceleration


# mqtt communication code
mqtt_broker = 'broker.hivemq.com' 
port = 1883
topic_sub = 'ME35-24/carlo'       # this reads anything sent to ME35

# turn off
def turnoff():
    blue.duty_u16(0)
    led[0] = (0, 0, 0)
    led.write()
    f.duty_u16(0)

def callback(topic, msg):
    print((topic.decode(), msg.decode()))

client = MQTTClient('ME35_chris', mqtt_broker , port, keepalive=60)
client.connect()
print('Connected to %s MQTT broker' % (mqtt_broker))
client.set_callback(callback)          # set the callback if anything is read
client.subscribe(topic_sub.encode())   # subscribe to a bunch of topics

mqtt = False
while not mqtt:
    msg = client.check_msg()

    if msg != None:
        mqtt = True
    
    time.sleep(1)

# variables
f = PWM(Pin('GPIO18', Pin.OUT))
blue = PWM(Pin(7, machine.Pin.OUT))
button = Pin(12, Pin.IN, Pin.PULL_UP)
state = (10,0,10)  # RGB
state_2 = (8,18,24)
led = neopixel.NeoPixel(Pin(28),1)
scl = Pin('GPIO27', Pin.OUT)
sda = Pin('GPIO26', Pin.OUT)
t = Acceleration(scl, sda)

# initializing LED
led[0] = state
led.write()


while mqtt:

    msg = client.check_msg()
    if msg != None:
        mqtt = False
    # blue LED
    led[0] = state
    led.write()
    blue.freq(50)
    
    # breathing
    for i in range(0,65535,500):
        if not button.value():
            break
        if msg != None:
            mqtt = False
        blue.duty_u16(i)     #  u16 means unsighed 16 bit integer (0-65535)
        time.sleep(0.02)
        i = 0

    # button and neopixel
    if not button.value():
        led[0] = state_2
        led.write()
        f.freq(440)
        f.duty_u16(1000)
        time.sleep(1)
        f.duty_u16(0)

turnoff()

