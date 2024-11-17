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

    def notify(self, direction):
        for conn_handle in self._connections:
            try:
                msg = str(direction)
                print("notifying with message: ", msg)
                self._ble.gatts_notify(conn_handle, self._handle, msg)
            except Exception as e:
                print(f"Error notifying: {e}")


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


ble = bluetooth.BLE()
motor_ble = BLEMotor(ble)

while True:
    clock.tick()
    img = sensor.snapshot()
    direction = ""
    # Check for detected AprilTags
    for tag in img.find_apriltags():
        img.draw_rectangle(tag.rect, color=(255, 0, 0))
        img.draw_cross(tag.cx, tag.cy, color=(0, 255, 0))
        x = str(tag.id)
        y = float(180 * tag.rotation)/math.pi
        print(y)


        if x == "0" and 0 <= y <= 180:
            direction = "Right"
        elif x == "0" and 181 <= y <= 350:
            direction = "Left"
        elif x == "1" and 0 <= y <= 160:
            direction = "Up"
        elif x == "1" and 161 <= y <= 300:
            direction = "Down"

        motor_ble.notify(direction) #Notifies the pico the direction the children choose

#        time.sleep(3) #sleep function to not overload the pico












