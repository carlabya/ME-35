import time
import bluetooth
import struct
import machine
from machine import Pin
import neopixel


# Stepper Motor Setup
up_down_motor = machine.PWM(machine.Pin(0), freq=1000)
left_right_motor = machine.PWM(machine.Pin(1), freq=1000)

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


class BLECentral:
    def __init__(self):
        self._ble = bluetooth.BLE()
        self._ble.active(True)
        self._ble.irq(self._irq)
        self._conn_handle = None
        self._motor_speed_handle = None

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
            self.execute_motor_instructions(decoded_data)
        elif event == _IRQ_GATTC_WRITE_DONE:
            conn_handle, value_handle, status = data
            print("Write complete")
        elif event == _IRQ_GATTC_SERVICE_DISCOVER:
            conn_handle, char_handle, char_uuid = data
            if char_uuid == _MOTOR_SPEED_CHAR_UUID:
                self._motor_speed_handle = char_handle
                print(f"Found Motor Speed Characteristic: {char_handle}")
                self.subscribe_to_motor_speed(conn_handle, char_handle)

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
            directions = instructions.split(",")  # Assuming "Right,Left" format
            for direction in directions:
                direction = direction.strip()  # Remove extra spaces
                if direction == "Up":
                    print("Moving Up")
                    up_down_motor.duty_u16(50000)  # Example PWM signal
                    time.sleep(1)
                    up_down_motor.duty_u16(0)
                elif direction == "Down":
                    print("Moving Down")
                    up_down_motor.duty_u16(30000)  # Example PWM signal
                    time.sleep(1)
                    up_down_motor.duty_u16(0)
                elif direction == "Right":
                    print("Moving Right")
                    left_right_motor.duty_u16(50000)  # Example PWM signal
                    time.sleep(1)
                    left_right_motor.duty_u16(0)
                elif direction == "Left":
                    print("Moving Left")
                    left_right_motor.duty_u16(30000)  # Example PWM signal
                    time.sleep(1)
                    left_right_motor.duty_u16(0)
            # Task complete, send a "completed" message
            self.send_message("completed")
            # print("Message Sent Back")
        except Exception as e:
            print("Error processing instructions:", e)

    def send_message(self, message):
        print(self._conn_handle)
        print(self._motor_speed_handle)
        if self._conn_handle and self._motor_speed_handle:
            self._ble.gattc_write(self._conn_handle, self._motor_speed_handle, message.encode(), 1)
            print(f"Sent message: {message}")


# Initialize BLECentral
central = BLECentral()
central.start_scan()

# Main loop
while True:
    time.sleep(1.5)
