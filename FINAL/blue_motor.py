import time
import bluetooth
import struct
import machine
from machine import Pin, Timer
import neopixel
import utime
from micropython import const

# Neopixel Configs
on = (10,0,10)
off = (0,0,0)

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

class BLECentral:
    def __init__(self):
        self._ble = bluetooth.BLE()
        self._ble.active(True)
        self._ble.irq(self._irq)
        self._conn_handle = None
        self._motor_speed_handle = None
        self.dir_pin = Pin(16, Pin.OUT)
        self.step_pin = Pin(17, Pin.OUT)
        self.wheeldir_pin = Pin(2, Pin.OUT)
        self.wheelstep_pin = Pin(3, Pin.OUT)
        self.steps_per_revolution = 1000
        # Initialize timer
        self.tim = Timer()

    def spoolstep(self,t):
        self.step_pin.value(not self.step_pin.value())
    def wheelstep(self,t):
        self.wheelstep_pin.value(not self.wheelstep_pin.value())
    def rotate_spoolmotor(self,delay):
        # Set up timer for stepping
        self.tim.init(freq=1000000//delay, mode=Timer.PERIODIC, callback=self.spoolstep)
    def rotate_wheelmotor(self,delay):
        # Set up timer for stepping
        self.tim.init(freq=1000000//delay, mode=Timer.PERIODIC, callback=self.wheelstep)

    def right(self):
        self.wheeldir_pin.value(0)
        self.rotate_wheelmotor(1000)
        utime.sleep_ms(self.steps_per_revolution)
        self.tim.deinit()  # stop the timer
        utime.sleep(1)
    def left(self):
        self.wheeldir_pin.value(1)
        self.rotate_wheelmotor(1000)
        utime.sleep_ms(self.steps_per_revolution)
        self.tim.deinit()  # stop the timer
        utime.sleep(1)
    def up(self):
        self.dir_pin.value(1)
        self.rotate_spoolmotor(1000)
        utime.sleep_ms(self.steps_per_revolution)
        self.tim.deinit()  # stop the timer
        utime.sleep(1)
    def down(self):
        self.dir_pin.value(0)
        self.rotate_spoolmotor(1000)
        utime.sleep_ms(self.steps_per_revolution)
        self.tim.deinit()  # stop the timer
        utime.sleep(1)

    def _irq(self, event, data):
        if event == _IRQ_SCAN_RESULT:
            addr_type, addr, adv_type, rssi, adv_data = data
            if self._find_service_in_advertisement(adv_data, _SERVICE_UUID):
                print("Found Motor Controller!")
                self._ble.gap_scan(None)
                self._ble.gap_connect(addr_type, addr)
        elif event == _IRQ_SCAN_COMPLETE:
            print("Scan complete")
        elif event == _IRQ_PERIPHERAL_CONNECT:
            conn_handle, addr_type, addr = data
            neo = neopixel.NeoPixel(Pin(28),1)
            neo[0] = on
            neo.write()
            print(f"Connection handle assigned: {self._conn_handle}")
            print("Connected")
            self._conn_handle = conn_handle
            self._ble.gattc_discover_services(conn_handle)
           

            
        elif event == _IRQ_PERIPHERAL_DISCONNECT:

            conn_handle, _, _ = data
            neo = neopixel.NeoPixel(Pin(28),1)
            neo[0] = off
            neo.write()
            print("Disconnected")
            self._conn_handle = None
            self._motor_speed_handle = None
            self.start_scan()
        elif event == _IRQ_GATTC_NOTIFY: 
            conn_handle, value_handle, notify_data = data
            decoded_data = bytes(notify_data).decode()
            print("Received data:", decoded_data)
            # self.send_message("completed")
            self.execute_motor_instructions(decoded_data)
        elif event == _IRQ_GATTC_WRITE_DONE:
            conn_handle, value_handle, status = data
            print("Write complete")
        elif event == _IRQ_GATTC_SERVICE_DISCOVER:
            conn_handle, char_handle, char_uuid = data
            print(f"Discovered characteristic: UUID={char_uuid}, Handle={char_handle}")

            if char_uuid == _MOTOR_SPEED_CHAR_UUID:
                self._motor_speed_handle = char_handle
                print(f"Motor Speed Handle Set: {char_handle}")
                self.subscribe_to_motor_speed(conn_handle, char_handle)
                print(f"Subscribing to notifications for handle: {char_handle}")

    def notify(self, direction):
        
            try:
                msg = str(direction)
                print("Notifying with message:", msg)
                self._ble.gatts_notify(64, 21, msg)
            except Exception as e:
                print(f"Error notifying: {e}")

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
                uuid16 = struct.unpack("<H", adv_data[i + 2 : i + length + 1])[0]
                if uuid16 == service_uuid:
                    return True
            i += length + 1
        return False

    def subscribe_to_motor_speed(self, conn_handle, char_handle):
        self._motor_speed_handle = char_handle
        self._ble.gattc_write(conn_handle, char_handle, b'\x01\x00', 1)

    def execute_motor_instructions(self, instructions):
        # Parse instructions
        try:
            directions = instructions.split(",")  
            for direction in directions:
                direction = direction.strip()  # Remove extra spaces
                if direction == "Up":
                    self.up()
                    print("Moving Up")
                    time.sleep(1)
                elif direction == "Down":
                    self.down()
                    print("Moving Down")
                    time.sleep(1)
                elif direction == "Right":
                    self.right()
                    print("Moving Right")
                    time.sleep(1)
                elif direction == "Left":
                    self.left()
                    print("Moving Left")
                    time.sleep(1)
            # send a "completed" message to 

            # self.send_message("completed")

            self.notify("completed")
            # print("Message Sent Back")
        except Exception as e:
            print("Error processing instructions:", e)
    

    def send_message(self, message):
            
            try:
                self._ble.gatts_write(self._conn_handle, 0x2A56, message.encode())
                print(f"Sent message: {message}")
            except Exception as e:
                print(f"Error sending message: {e}")
    


# Initialize BLECentral
central = BLECentral()
central.start_scan()

# Main loop
while True:
    time.sleep(1.5)
