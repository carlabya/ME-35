import time
import bluetooth
import struct
from machine import Pin, Timer
import neopixel
import utime
from micropython import const

# Neopixel Configs
on = (10, 0, 10)
off = (0, 0, 0)

# BLE event constants
_IRQ_SCAN_RESULT = const(5)
_IRQ_SCAN_COMPLETE = const(6)
_IRQ_PERIPHERAL_CONNECT = const(7)
_IRQ_PERIPHERAL_DISCONNECT = const(8)
_IRQ_GATTC_NOTIFY = const(18)

# BLE UUIDs
_SERVICE_UUID = 0x1815
_MOTOR_SPEED_CHAR_UUID = 0x2A56

class BLECentral:
    def __init__(self):
        # Initialize BLE
        self._ble = bluetooth.BLE()
        self._ble.active(True)
        self._ble.irq(self._irq)
        self._conn_handle = None
        self._motor_speed_handle = None

        # Initialize motor pins
        self.dir_pin = Pin(16, Pin.OUT)
        self.step_pin = Pin(17, Pin.OUT)
        self.wheeldir_pin = Pin(2, Pin.OUT)
        self.wheelstep_pin = Pin(3, Pin.OUT)

        # Movement and timers
        self.steps_per_revolution = 1000
        self.tim = Timer()

        # Command queue
        self.command_queue = []

        # Neopixel for status
        self.neo = neopixel.NeoPixel(Pin(28), 1)
        self.neo[0] = off
        self.neo.write()

    # Timer callbacks for motor steps
    def spoolstep(self, t):
        self.step_pin.value(not self.step_pin.value())

    def wheelstep(self, t):
        self.wheelstep_pin.value(not self.wheelstep_pin.value())

    # Motor control functions
    def rotate_spoolmotor(self, delay):
        self.tim.init(freq=1000000 // delay, mode=Timer.PERIODIC, callback=self.spoolstep)

    def rotate_wheelmotor(self, delay):
        self.tim.init(freq=1000000 // delay, mode=Timer.PERIODIC, callback=self.wheelstep)

    def right(self):
        self.wheeldir_pin.value(0)
        self.rotate_wheelmotor(1000)
        utime.sleep_ms(self.steps_per_revolution)
        self.tim.deinit()
        time.sleep(1)

    def left(self):
        self.wheeldir_pin.value(1)
        self.rotate_wheelmotor(1000)
        utime.sleep_ms(self.steps_per_revolution)
        self.tim.deinit()
        time.sleep(1)

    def up(self):
        self.dir_pin.value(1)
        self.rotate_spoolmotor(1000)
        utime.sleep_ms(self.steps_per_revolution)
        self.tim.deinit()
        time.sleep(1)

    def down(self):
        self.dir_pin.value(0)
        self.rotate_spoolmotor(1000)
        utime.sleep_ms(self.steps_per_revolution)
        self.tim.deinit()
        time.sleep(1)

    # BLE IRQ handler
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
            self.start_scan()

        elif event == _IRQ_GATTC_NOTIFY:
            conn_handle, value_handle, notify_data = data
            decoded_data = bytes(notify_data).decode()
            print("Received data:", decoded_data)
            self.enqueue_motor_commands(decoded_data)

    def enqueue_motor_commands(self, instructions):
        try:
            directions = instructions.split(",")
            for direction in directions:
                direction = direction.strip()
                self.command_queue.append(direction)
            print("Commands added to queue:", directions)
        except Exception as e:
            print("Error processing instructions:", e)

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

    def start_scan(self):
        print("Scanning for BLE devices...")
        self._ble.gap_scan(2000, 30000, 30000)


# Initialize BLECentral
central = BLECentral()
central.start_scan()

# Main loop for processing commands
while True:
    if central.command_queue:
        command = central.command_queue.pop(0)
        if command == "Up":
            central.up()
            print("Moving Up")
        elif command == "Down":
            central.down()
            print("Moving Down")
        elif command == "Right":
            central.right()
            print("Moving Right")
        elif command == "Left":
            central.left()
            print("Moving Left")
    time.sleep(0.1)  # Allow other tasks to run
