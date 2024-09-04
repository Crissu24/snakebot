# Importing Libraries
import RPi.GPIO as GPIO
import time
from adafruit_servokit import ServoKit

# Initializing GPIO and PCA9685
GPIO.setmode(GPIO.BCM)
TRIG = 15
ECHO = 14
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
kit = ServoKit(channels=16)
for i in range(6):
    kit.servo[i].angle = 90

# Function get_distance()
def get_distance():
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    start_time = time.time()
    stop_time = time.time()
    while GPIO.input(ECHO) == 0:
        start_time = time.time()
    while GPIO.input(ECHO) == 1:
        stop_time = time.time()
    elapsed_time = stop_time - start_time
    distance = (elapsed_time * 34300) / 2
    return distance

# Function avoid_obstacle()
def avoid_obstacle():
    for i in range(6):
        kit.servo[i].angle = 90
    time.sleep(1)
    # Avoidance maneuver, e.g., rotating to 45 degrees
    for i in range(6):
        if i % 2 == 0:
            kit.servo[i].angle = 135
        else:
            kit.servo[i].angle = 45
    time.sleep(1)
    # Reset angles
    for i in range(6):
        kit.servo[i].angle = 90
    time.sleep(1)

def ondulare():
    while True:
        distance = get_distance()
        if distance < 10:
            avoid_obstacle()
        else:
            for angle in range(60, 120, 5):
                for i in range(6):
                    if i % 2 == 0:
                        kit.servo[i].angle = angle
                    else:
                        kit.servo[i].angle = 180 - angle
                time.sleep(0.05)
            for angle in range(120, 60, -5):
                for i in range(6):
                    if i % 2 == 0:
                        kit.servo[i].angle = angle
                    else:
                        kit.servo[i].angle = 180 - angle
                time.sleep(0.05)

# Main Loop
try:
    ondulare()
except KeyboardInterrupt:
    for i in range(6):
        kit.servo[i].angle = 90
    print("Movement stopped by user")
    GPIO.cleanup()