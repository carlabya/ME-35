# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# AprilTags Example
#
# This example shows the power of the OpenMV Cam to detect April Tags
# on the OpenMV Cam M7. The M4 versions cannot detect April Tags.
import network
from mqtt import MQTTClient
import sensor
import time
import math

SSID = "Tufts_Robot"  # Network SSID
KEY = ""  # Network key

# Init wlan module and connect to network
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(SSID, KEY)

while not wlan.isconnected():
    print('Trying to connect to "{:s}"...'.format(SSID))
    time.sleep_ms(1000)

# We should have a valid IP now via DHCP
print("WiFi Connected ", wlan.ifconfig())


client = MQTTClient("ME35_carla", "broker.hivemq.com", port=1883)
client.connect()

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  # must turn this off to prevent image washout...
sensor.set_auto_whitebal(False)  # must turn this off to prevent image washout...
clock = time.clock()

# Note! Unlike find_qrcodes the find_apriltags method does not need lens correction on the image to work.

# Please use the TAG36H11 tag family for this script - it's the recommended tag family to use.


while True:
    clock.tick()
    img = sensor.snapshot()
    x = ""
    for tag in img.find_apriltags():
        img.draw_rectangle(tag.rect, color=(255, 0, 0))
        img.draw_cross(tag.cx, tag.cy, color=(0, 255, 0))
        x = str(tag.id)
#       print(print_args)
#       print("Tag ID %d" % print_args)
#    print(clock.fps())

    if x == "0":
        client.publish("ME35-24/carlo", "Forward")
    elif x == "1":
        client.publish("ME35-24/carlo", "Backward")
    elif x == "2":
        client.publish("ME35-24/carlo", "Right")
    elif x == "3":
        client.publish("ME35-24/carlo", "Left")

    time.sleep_ms(1000)

