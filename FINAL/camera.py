import sensor
import time
import math
import struct
import bluetooth
import asyncio
from micropython import const

# Initialize the camera
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  # must turn this off to prevent image washout...
sensor.set_auto_whitebal(False)  # must turn this off to prevent image washout...
clock = time.clock()

# Camera calibration values
f_x = (2.8 / 3.984) * 160  # find_apriltags defaults to this if not set
f_y = (2.8 / 2.952) * 120  # find_apriltags defaults to this if not set
c_x = 160 * 0.5  # find_apriltags defaults to this if not set (the image.w * 0.5)
c_y = 120 * 0.5  # find_apriltags defaults to this if not set (the image.h * 0.5)

def degrees(radians):
    return (180 * radians) / math.pi

# Constants for BLE events
_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)
_IRQ_SCAN_RESULT = const(5)
_IRQ_SCAN_COMPLETE = const(6)
_IRQ_PERIPHERAL_CONNECT = const(7)
_IRQ_PERIPHERAL_DISCONNECT = const(8)
_IRQ_GATTC_NOTIFY = const(18)
_IRQ_GATTC_WRITE_DONE = const(20)
_IRQ_GATTC_SERVICE_DISCOVER = const(21)

_FLAG_READ = const(0x0002)
_FLAG_NOTIFY = const(0x0010)

# BLE Service and Characteristic UUIDs
_SERVICE_UUID = bluetooth.UUID(0x1815)
_MOTOR_SPEED_CHAR_UUID = (bluetooth.UUID(0x2A56), _FLAG_NOTIFY | _FLAG_READ)
_MOTOR_SERVICE = (_SERVICE_UUID, (_MOTOR_SPEED_CHAR_UUID,))

class BLEMotor:
    def __init__(self, ble, name="CarlaB"):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)
        handles = self._ble.gatts_register_services((_MOTOR_SERVICE,))
        self._handle = handles[0][0]
        self._connections = set()
        self._payload = advertising_payload(name=name, services=[_SERVICE_UUID])
        self._advertise()
        self.completed = False  # Track when a "completed" message is received

    def _irq(self, event, data):
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            self._connections.remove(conn_handle)
            self._advertise()
        elif event == _IRQ_GATTS_WRITE:
            conn_handle, value_handle = data
            value = self._ble.gatts_read(value_handle).decode().strip()
            print(f"Received message: {value}")
            if value == "completed":
                self.completed = True  # Set flag when "completed" message is received
        elif event == _IRQ_GATTC_NOTIFY:
            conn_handle, value_handle, notify_data = data
            decoded_data = bytes(notify_data).decode()
            print("Received data:", decoded_data)
            if decoded_data == "completed":
                self.completed = False

    def _advertise(self, interval_us=500000):
        self._ble.gap_advertise(interval_us, adv_data=self._payload)

    def notify(self, direction):
        for conn_handle in self._connections:
            try:
                msg = str(direction)
                print("Notifying with message:", msg)
                print(conn_handle)
                print(self._handle)
                self._ble.gatts_notify(conn_handle, self._handle, msg)
            except Exception as e:
                print(f"Error notifying: {e}")

    def _process_received_message(self, message):

            # Handle the received message
            if message == "complete":
                print("Reset command received.")
                self.completed = False  # Reset the flag
            elif message.startswith("calibrate:"):
                params = message.split(":")[1]
                print(f"Calibrating with params: {params}")
            else:
                print(f"Unhandled message: {message}")




def advertising_payload(limited_disc=False, br_edr=False, name=None, services=None):
    payload = bytearray()

    def _append(adv_type, value):
        nonlocal payload
        payload += bytes((len(value) + 1, adv_type)) + value

    flags = (0x02 if limited_disc else 0x06) + (0x00 if br_edr else 0x04)
    _append(0x01, struct.pack("B", flags))

    if name:
        _append(0x09, name.encode())

    if services:
        for uuid in services:
            uuid_bytes = bytes(uuid)
            if len(uuid_bytes) == 2:
                _append(0x03, uuid_bytes)
            elif len(uuid_bytes) == 16:
                _append(0x07, uuid_bytes)

    return payload



# Initialize BLE
ble = bluetooth.BLE()
motor_ble = BLEMotor(ble)

# Allow time for Bluetooth devices to connect
print("Waiting for BLE connection...")
time.sleep(8)  # Delay to ensure connection


firstTime = True

# Main loop
while True:
    clock.tick()
    img = sensor.snapshot()
    detected_tags = []

    # Detect and sort AprilTags
    for tag in img.find_apriltags():  # defaults to TAG36H11
        img.draw_rectangle(tag.rect, color=(255, 0, 0))
        img.draw_cross(tag.cx, tag.cy, color=(0, 255, 0))
        detected_tags.append((tag.id, tag.y_translation, tag.rotation))
    detected_tags.sort(key=lambda t: t[1], reverse=True)

    # Process detected tags into directions
    movement_instructions = []



#    while motor_ble.completed == False:
#        value_handle = "Completed"
#        value = motor_ble._ble.gatts_read(value_handle).decode().strip()
#        print(value)
#        time.sleep(.1)

#    motor_ble.completed = False  # Reset the flag for the next cycle
#   print(f"Sent instructions: {instructions_str}")

    for tag_id, _, rotation in detected_tags:
        rotation_deg = degrees(rotation)
        if tag_id == 1 and 0 <= rotation_deg <= 180:
            movement_instructions.append("Right")
        elif tag_id == 1 and 181 <= rotation_deg <= 359:
            movement_instructions.append("Left")
        elif tag_id == 0 and 0 <= rotation_deg <= 160:
            movement_instructions.append("Up")
        elif tag_id == 0 and 161 <= rotation_deg <= 300:
            movement_instructions.append("Down")

    #Remove with Apriltags for testing
    if motor_ble.completed == False:
        motor_ble.notify("Up, Left")  # Send the string
        motor_ble.completed = True

    # If there are instructions, notify once with the full set
    if movement_instructions and motor_ble.completed == False:
        instructions_str = ",".join(movement_instructions)  # Create a single string
        motor_ble.notify("Up, Left")  # Send the string and put instructions_str in the paratheses
        print(f"Sent instructions: {instructions_str}")
        motor_ble.completed = True


        # Wait for "completed" before sending new instructions
        while motor_ble.completed == True:
            time.sleep(.1)

#        motor_ble.completed = False  # Reset the flag for the next cycle
