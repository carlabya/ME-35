import time
import bluetooth
import struct
import machine
from machine import Pin, Timer, ADC, PWM
import neopixel
import utime
from micropython import const

# Neopixel Configs
on = (10, 0, 10)
off = (0, 0, 0)
alert = (0, 255, 0)  # Red for magnetic detection

# BLE event constants
_IRQ_SCAN_RESULT = const(5)
_IRQ_SCAN_COMPLETE = const(6)
_IRQ_PERIPHERAL_CONNECT = const(7)
_IRQ_PERIPHERAL_DISCONNECT = const(8)
_IRQ_GATTC_NOTIFY = const(18)
_IRQ_GATTC_WRITE_DONE = const(20)
_IRQ_GATTC_SERVICE_DISCOVER = const(21)

# BLE UUIDs
_SERVICE_UUID = 0x1815
_MOTOR_SPEED_CHAR_UUID = 0x2A56

_FLAG_READ = const(0x0002)
_FLAG_NOTIFY = const(0x0010)

# Threshold for magnetic detection (tune based on Hall sensor and magnet strength)
MAGNETIC_FIELD_THRESHOLD = 1

class BLECentral:
    def __init__(self):
        self._ble = bluetooth.BLE()
        self._ble.active(True)
        self._ble.irq(self._irq)
        self._conn_handle = None
        self._motor_speed_handle = None

        # Motor Pins
        self.dir_pin = Pin(16, Pin.OUT)
        self.step_pin = Pin(17, Pin.OUT)
        self.wheeldir_pin = Pin(2, Pin.OUT)
        self.wheelstep_pin = Pin(3, Pin.OUT)
        self.steps_per_revolution = 1000

        # Hall Effect Sensor
        self.hall_sensor = Pin(26, machine.Pin.IN, machine.Pin.PULL_UP) # Pin reading from hall sensor

        # Buzzer and NeoPixel
        self.buzzer = PWM(Pin(18, Pin.OUT)) # Pin connected to the buzzer
        self.buzzer.freq(500)
        self.neo = neopixel.NeoPixel(Pin(28), 1)  # Pin for NeoPixel LED

        # Timer for motor stepping
        self.tim = Timer()

    def spoolstep(self, t):
        self.step_pin.value(not self.step_pin.value())

    def wheelstep(self, t):
        self.wheelstep_pin.value(not self.wheelstep_pin.value())

    def rotate_spoolmotor(self, delay):
        self.tim.init(freq=1000000 // delay, mode=Timer.PERIODIC, callback=self.spoolstep)

    def rotate_wheelmotor(self, delay):
        self.tim.init(freq=1000000 // delay, mode=Timer.PERIODIC, callback=self.wheelstep)

    def right(self):
        self.wheeldir_pin.value(0)
        self.rotate_wheelmotor(1000)
        utime.sleep_ms(self.steps_per_revolution)
        self.tim.deinit()
        utime.sleep(1)

    def left(self):
        self.wheeldir_pin.value(1)
        self.rotate_wheelmotor(1000)
        utime.sleep_ms(self.steps_per_revolution)
        self.tim.deinit()
        utime.sleep(1)

    def up(self):
        self.dir_pin.value(1)
        self.rotate_spoolmotor(1000)
        utime.sleep_ms(self.steps_per_revolution)
        self.tim.deinit()
        utime.sleep(1)

    def down(self):
        self.dir_pin.value(0)
        self.rotate_spoolmotor(1000)
        utime.sleep_ms(self.steps_per_revolution)
        self.tim.deinit()
        utime.sleep(1)

    def _irq(self, event, data):
        if event == _IRQ_SCAN_RESULT:
            addr_type, addr, adv_type, rssi, adv_data = data
            if self._find_service_in_advertisement(adv_data, _SERVICE_UUID):
                print("Found Motor Controller!")
                self._ble.gap_scan(None)
                self._ble.gap_connect(addr_type, addr)
        elif event == _IRQ_PERIPHERAL_CONNECT:
            conn_handle, addr_type, addr = data
            self.neo[0] = on
            self.neo.write()
            print("Connected")
            self._conn_handle = conn_handle
            self._ble.gattc_discover_services(conn_handle)
        elif event == _IRQ_PERIPHERAL_DISCONNECT:
            self.neo[0] = off
            self.neo.write()
            print("Disconnected")
            self._conn_handle = None
            self._motor_speed_handle = None
            self.start_scan()
        elif event == _IRQ_GATTC_NOTIFY:
            conn_handle, value_handle, notify_data = data
            decoded_data = bytes(notify_data).decode()
            print("Received data:", decoded_data)
            self.execute_motor_instructions(decoded_data)

    def start_scan(self):
        print("Scanning for BLE devices...")
        self._ble.gap_scan(2000, 30000, 30000)

    def _find_service_in_advertisement(self, adv_data, service_uuid):
        i = 0
        while i < len(adv_data):
            length = adv_data[i]
            if length == 0:
                break
            ad_type = adv_data[i + 1]
            if ad_type == 0x03:
                uuid16 = struct.unpack("<H", adv_data[i + 2: i + length + 1])[0]
                if uuid16 == service_uuid:
                    return True
            i += length + 1
        return False

    def execute_motor_instructions(self, instructions):
        try:
            directions = instructions.split(",")
            for direction in directions:
                direction = direction.strip()
                if direction == "Up":
                    self.up()
                elif direction == "Down":
                    self.down()
                elif direction == "Right":
                    self.right()
                elif direction == "Left":
                    self.left()
        except Exception as e:
            print("Error processing instructions:", e)

    def check_hall_sensor(self):
        """Check the Hall effect sensor for magnetic field detection."""
        hall_value = self.hall_sensor.value()
        if hall_value < MAGNETIC_FIELD_THRESHOLD:
            print(f"Magnetic field detected! Value: {hall_value}")
            self.neo[0] = alert
            self.neo.write()
            self.buzzer.duty_u16(1000)
            utime.sleep(0.5)  # Buzzer on for 0.5 seconds
            self.buzzer.duty_u16(0)
        else:
            self.neo[0] = off
            self.neo.write()
            print(f"Value: {hall_value}")

# Initialize BLECentral
central = BLECentral()
central.start_scan()

# Main loop
while True:
    central.check_hall_sensor()
    time.sleep(0.1)  # Check sensor at 10 Hz
