import sensor
import time
import math
from machine import Pin
import sensor
import machine
import bluetooth
from micropython import const
import struct
import network
from machine import LED

# Initialize the camera
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  # must turn this off to prevent image washout...
sensor.set_auto_whitebal(False)  # must turn this off to prevent image washout...
clock = time.clock()

# Constants for BLE events
_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)

_FLAG_READ = const(0x0002)
_FLAG_WRITE = const(0x0008)
_FLAG_NOTIFY = const(0x0010)
_FLAG_INDICATE = const(0x0020)

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
        self._handle = handles[0][0]  # Correct way to get the first handle
        self._connections = set()
        self._payload = advertising_payload(name=name, services=[_SERVICE_UUID])
        self._advertise()

    def _irq(self, event, data):
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            self._connections.remove(conn_handle)
            self._advertise()
        elif event == _IRQ_GATTS_WRITE:
            pass  # Handle any incoming writes if needed

    def _advertise(self, interval_us=500000):
        self._ble.gap_advertise(interval_us, adv_data=self._payload)

    def notify_speed(self, speed):
        print(f"Notify speed called with speed: {speed}")  # Debug line
        for conn_handle in self._connections:
            try:
                msg = str(speed)
                #print(f"Attempting to notify speed: {speed_str}")  # Debug line before notifying
                self._ble.gatts_notify(conn_handle, self._handle, msg)
            except Exception as e:
                print(f"Error notifying speed: {e}")  # Catch and print errors


def advertising_payload(limited_disc=False, br_edr=False, name=None, services=None):
    payload = bytearray()

    def _append(adv_type, value):
        nonlocal payload
        payload += bytes((len(value) + 1, adv_type)) + value

    # Flags
    flags = (0x02 if limited_disc else 0x06) + (0x00 if br_edr else 0x04)
    _append(0x01, struct.pack("B", flags))

    # Name
    if name:
        _append(0x09, name.encode())

    # Services (UUIDs)
    if services:
        for uuid in services:
            if isinstance(uuid, bluetooth.UUID):
                uuid_bytes = bytes(uuid)
                if len(uuid_bytes) == 2:  # 16-bit UUID
                    _append(0x03, uuid_bytes)
                elif len(uuid_bytes) == 16:  # 128-bit UUID
                    _append(0x07, uuid_bytes)

    return payload

# Proportional control gain
Kp = 1.5

previous_tag_id = None  # To keep track of the last detected tag ID
ble = bluetooth.BLE()
motor_ble = BLEMotor(ble)

while True:
    clock.tick()
    img = sensor.snapshot()

    # Check for detected AprilTags
    for tag in img.find_apriltags():
        img.draw_rectangle(tag.rect, color=(255, 0, 0))
        img.draw_cross(tag.cx, tag.cy, color=(0, 255, 0))

        # Calculate control signal based on tag's x-coordinate
        control_signal = tag.cx - img.width() // 2
        control_signal *= -1
        print(control_signal)

        # moving left or right
        if control_signal in range(-10,10):
            print("Tag is centered")
        elif control_signal < -10:
            print("Tag is moving left")
        elif control_signal > 10:
            print("Tag is moving right")

        # Update last detected tag ID
        if previous_tag_id is None or tag.id != previous_tag_id:
            previous_tag_id = tag.id  # Update last detected tag ID

        motor_ble.notify_speed(control_signal)
        time.sleep(0.25)
