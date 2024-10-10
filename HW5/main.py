import machine
import time
import neopixel  
from blue_handler import BluetoothHandler  

# Right wheel (Motor 1)
motor1_dir = machine.PWM(machine.Pin(0, machine.Pin.OUT))  # Direction control
motor1_speed = machine.PWM(machine.Pin(1, machine.Pin.OUT))  # Speed control
motor1_speed.freq(1000)  # Set PWM frequency for speed control
motor1_dir.freq(1000)    # Set PWM frequency for direction control

# Left wheel (Motor 2)
motor2_dir = machine.PWM(machine.Pin(2, machine.Pin.OUT))  # Direction control
motor2_speed = machine.PWM(machine.Pin(3, machine.Pin.OUT))  # Speed control
motor2_speed.freq(1000)  # Set PWM frequency for speed control
motor2_dir.freq(1000)    # Set PWM frequency for direction control

neo_pin = machine.Pin(4)
neo_pixel = neopixel.NeoPixel(neo_pin, 1)  # 1 NeoPixel

def set_motor_speed(direction):
    base_speed = 30000  #  base speed 
    adjustment = abs(direction) * 800  

    minimum_speed = 20000  # Minimum duty cycle to keep the motors from stalling

    if direction < -25:  # Turning left
        print("Moving left")
        motor1_dir.duty_u16(65535)  # Forward direction for right motor
        motor1_speed.duty_u16(max(base_speed, minimum_speed))  # Set right motor speed
        motor2_dir.duty_u16(0)  # Reverse direction for left motor
        motor2_speed.duty_u16(max(base_speed - adjustment, minimum_speed))  # Set reduced speed for left motor
    elif direction > 25:  # Turning right
        print("Moving right")
        motor1_dir.duty_u16(0)  # Reverse direction for right motor
        motor1_speed.duty_u16(max(base_speed - adjustment, minimum_speed))  # Set reduced speed for right motor
        motor2_dir.duty_u16(65535)  # Forward direction for left motor
        motor2_speed.duty_u16(max(base_speed, minimum_speed))  # Set left motor speed
    else:  # Moving straight
        print("Centered, moving straight")
        motor1_dir.duty_u16(65535)  # Forward direction for right motor
        motor1_speed.duty_u16(base_speed)  # Set full speed for right motor
        motor2_dir.duty_u16(65535)  # Forward direction for left motor
        motor2_speed.duty_u16(base_speed)  # Set full speed for left motor

#  stop the motors (do nothing when no valid direction)
def stop_motors():
    print("Stopping motors")
    motor1_speed.duty_u16(0)  
    motor2_speed.duty_u16(0)  

# set NeoPixel color
def set_neopixel_color(color):
    neo_pixel[0] = color  
    neo_pixel.write()  

# Initialize and start BLE
bluetooth_handler = BluetoothHandler()
bluetooth_handler.start_scan()

# Main loop to check for Bluetooth messages every 1.5 seconds
last_check_time = time.time()
bluetooth_connected = False

while True:
    current_time = time.time()
    
    # Check if 1.5 seconds have passed since the last Bluetooth read
    if current_time - last_check_time >= 0.5:
        last_check_time = current_time  # Update the last check time
        
        # Read the latest Bluetooth notification
        last_notification = bluetooth_handler.get_latest_notification()
        
        if last_notification is not None:
            # Bluetooth connected, turn on NeoPixel
            if not bluetooth_connected:
                print("Bluetooth connected, turning NeoPixel on.")
                set_neopixel_color((0, 255, 0))  # Set NeoPixel to green when connected
                bluetooth_connected = True
            
            try:
                # Parse the direction from the Bluetooth message
                direction = int(last_notification)
                set_motor_speed(direction)  # Adjust motor speeds based on new direction
            except ValueError:
                print("Invalid notification data for motor speed.")
                stop_motors()  # Stop motors if data is invalid
        else:
            print("No new notification. Stopping motors.")
            stop_motors()  # Stop motors if no valid notification is received

            # Neopixel off is bluetooth disconnected
            if bluetooth_connected:
                print("Bluetooth disconnected, turning NeoPixel off.")
                set_neopixel_color((0, 0, 0))  # Set NeoPixel to off (black)
                bluetooth_connected = False

    #  delay to prevent overloading the system
    time.sleep(0.1)
