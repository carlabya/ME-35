import machine
import time
import neopixel
import asyncio
import struct
import network
from machine import Pin, PWM
from MQTT import MQTTClient

class Car:
    
        driveForward = False
        driveBackward = False
        driveRight = False
        driveLeft = False
        motorOn = False  # Initially off
        motorOff = True

        def __init__(self):
            self.wlan = network.WLAN(network.STA_IF)
            
            
            # Pico board initializations: neopixel, led, buzzer
            self.neo = neopixel.NeoPixel(Pin(28), 1)
            self.button = Pin(8, Pin.IN, Pin.PULL_UP)
            self.blue_led = PWM(Pin(7, Pin.OUT))
            self.blue_led.freq(100)
            self.buzzer = PWM(Pin(18, Pin.OUT))
            self.buzzer.freq(440)

            # Motor 1 (pins 0 and 1) (Right Wheel) (Carla's Pico)
            self.motor1_dir = PWM(Pin(0, Pin.OUT))
            self.motor1_speed = PWM(Pin(1, Pin.OUT))
            self.motor1_speed.freq(1000)  # Frequency for PWM
            self.motor1_dir.freq(1000)

            # Motor 2 (pins 2 and 3) (Left Wheel) (Lucy's pico)
            # self.motor2_dir = PWM(Pin(2, Pin.OUT))
            # self.motor2_speed = PWM(Pin(3, Pin.OUT))
            # self.motor2_speed.freq(1000)  # Frequency for PWM
            # self.motor2_dir.freq(1000)

            # Flashing LEDs for turn indication
            self.turn_right_led = Pin(11, Pin.OUT)
            self.turn_left_led = Pin(12, Pin.OUT)

            # Call internet connection
            self.connect()
            self.mqtt_subscribe()

        def mqtt_subscribe(self):
            # MQTT initialization of client and subscribing to topic 'Mater'
            mqtt_broker = 'broker.hivemq.com'
            port = 1883
            topic_sub = 'ME35-24/carlo'

            def callback(topic, msg):
                topic, msg = topic.decode(), msg.decode()
                print(msg)
                if msg == "1.00, 0.00":
                    print('Start')
                    self.motorOff = False
                    self.motorOn = True
                    self.motor_control()
                    
                elif msg == "0.00, 1.00":
                    print('Stop')
                    self.motorOn = False
                    self.motorOff = True
                    self.motor_control()

                elif msg == "Right" and self.motorOn:
                    print("jumped in right function")
                    self.driveRight = True
                    self.driveForward = False
                    self.driveBackward = False
                    self.driveLeft = False
                    self.turn_Right()
                
                elif msg == "Left" and self.motorOn:
                    print("jumped in left function")
                    self.driveRight = False
                    self.driveForward = False
                    self.driveBackward = False
                    self.driveLeft = True
                    self.turn_Left()

                
                elif msg == "Forward" and self.motorOn:
                    "jumped in forward function"
                    self.driveRight = False
                    self.driveForward = True
                    self.driveBackward = False
                    self.driveLeft = False
                    self.forward()

                    
                elif msg == "Backward" and self.motorOn:
                    "jumped in backward function"
                    self.driveRight = False
                    self.driveForward = False
                    self.driveBackward = True
                    self.driveLeft = False
                    self.backward()

                    

            self.client = MQTTClient('ME35_baby', mqtt_broker, port, keepalive=60)
            self.client.connect()
            print('Connected to %s MQTT broker' % mqtt_broker)
            self.client.set_callback(callback)  # Set the callback for incoming messages
            self.client.subscribe(topic_sub.encode())  # Subscribe to a topic
            print(f'Subscribed to topic {topic_sub}')  # Debug print

        async def check_mqtt(self):
            while True:
                self.client.check_msg()
                await asyncio.sleep(.01)

        def connect(self):
            try:
                self.wlan.active(True)
                self.wlan.connect('Tufts_Robot', '')

                while self.wlan.ifconfig()[0] == '0.0.0.0':
                    print('.', end=' ')
                    time.sleep(1)

                # We should have a valid IP now via DHCP
                print(self.wlan.ifconfig())
            except Exception as e:
                print("Failed Connection: ", e)
                quit()

        def motor_control(self):
            print("in motor control")
            if self.motorOn:
                print("motor on")
                # Allow the motors to move as commands are received
            elif self.motorOff:
                print("motor off")
                # Stop motors and turn off LED signals
                self.motor1_speed.duty_u16(0)
                # self.motor2_speed.duty_u16(0)
                self.motor1_dir.duty_u16(0)
                # self.motor2_dir.duty_u16(0)
                self.turn_right_led.off()
                self.turn_left_led.off()

        def turn_Right(self):
            if self.driveRight:
                print("Turning right")
                # Motor 1 forward, Motor 2 backward
                self.motor1_dir.duty_u16(0)  # Set direction forward
                self.motor1_speed.duty_u16(65535)  # Set speed for motor 1
                # self.motor2_dir.duty_u16(0)  # Set direction backward
                # self.motor2_speed.duty_u16(65535)  # Set speed for motor 2
                
                # Flash right LED while turning right
                self.flash_led(self.turn_right_led)
                

                
                

        def turn_Left(self):
            if self.driveLeft:
                print("Turning left")
                # Motor 1 backward, Motor 2 forward
                self.motor1_dir.duty_u16(0)  # Set direction backward
                self.motor1_speed.duty_u16(30000)  # Set speed for motor 1
                # self.motor2_dir.duty_u16(0)  # Set direction forward
                # self.motor2_speed.duty_u16(65535)  # Set speed for motor 2
                
                # Flash left LED while turning left
                self.flash_led(self.turn_left_led)
              
                
                

        def forward(self):
            if self.driveForward:
                print("Backward")
                # Both motors forward
                self.motor1_dir.duty_u16(0)
                self.motor1_speed.duty_u16(65535)  # Adjust speed as needed
                # self.motor2_dir.duty_u16(0)
                # self.motor2_speed.duty_u16(65535)
                
                
        def backward(self):

            if self.driveBackward:

                print("Forward")
                # Both motors backward
                self.motor1_dir.duty_u16(65535)
                self.motor1_speed.duty_u16(0)  # Adjust speed as needed
                # self.motor2_dir.duty_u16(65535)
                # self.motor2_speed.duty_u16(0)
               

                
                
        def flash_led(self, led_pin):
            # Function to flash LED for indicating turn
            for _ in range(5):  # Flash 5 times
                led_pin.on()
                time.sleep(0.2)
                led_pin.off()
                time.sleep(0.2)

        async def main(self):
            asyncio.create_task(self.check_mqtt())
            asyncio.get_event_loop().run_forever()

# Create Car instance
c = Car()

asyncio.run(c.main())
