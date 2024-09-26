from machine import Pin, PWM
import time, neopixel, machine
from mqtt import MQTTClient

class MQTTHandler:
    def __init__(self, broker, port, topic_sub):
        self.broker = broker
        self.port = port
        self.topic_sub = topic_sub
        self.client = MQTTClient('ME35_chris', broker, port, keepalive=60)
        self.mqtt_connected = False

    def connect(self):
        self.client.connect()
        print(f'Connected to {self.broker} MQTT broker')
        self.client.set_callback(self.callback)
        self.client.subscribe(self.topic_sub.encode())

    def callback(self, topic, msg):
        print((topic.decode(), msg.decode()))

    def wait_for_connection(self):
        while not self.mqtt_connected:
            msg = self.client.check_msg()
            if msg:
                self.mqtt_connected = True
            time.sleep(1)

    def check_message(self):
        return self.client.check_msg()

class LEDController:
    def __init__(self):
        self.blue = machine.PWM(machine.Pin('GPIO0', machine.Pin.OUT))
        self.led = neopixel.NeoPixel(Pin(28), 1)
        self.f = PWM(Pin('GPIO18', Pin.OUT))
        self.state = (10, 0, 10)  # RGB
        self.state_2 = (8, 18, 24)
        self.button = Pin('GPIO20', Pin.IN)

    def turn_off(self):
        self.blue.duty_u16(0)
        self.led[0] = (0, 0, 0)
        self.led.write()
        self.f.duty_u16(0)

    def initialize_led(self):
        self.led[0] = self.state
        self.led.write()

    def breathing_effect(self):
        self.blue.freq(50)
        for i in range(0, 65535, 500):
            if not self.button.value():
                break
            self.blue.duty_u16(i)
            time.sleep(0.02)

    def handle_button_press(self):
        if not self.button.value():
            self.led[0] = self.state_2
            self.led.write()
            self.f.freq(440)
            self.f.duty_u16(1000)
            time.sleep(1)
            self.f.duty_u16(0)

def main():
    mqtt_handler = MQTTHandler('broker.hivemq.com', 1883, 'ME35-24/carlo')
    mqtt_handler.connect()
    mqtt_handler.wait_for_connection()

    led_controller = LEDController()
    led_controller.initialize_led()

    while mqtt_handler.mqtt_connected:
        msg = mqtt_handler.check_message()
        if msg:
            mqtt_handler.mqtt_connected = False

        led_controller.initialize_led()
        led_controller.breathing_effect()
        led_controller.handle_button_press()

    led_controller.turn_off()

if __name__ == '__main__':
    main()
