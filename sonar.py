# This script is responsible for the three sonar sensors, it establishes their gpio pins and provides
# functions to get their current distance
# When executed, can be used to get whether the sonars are working properly
# Will give -1 if sonar is not working/connected


import RPi.GPIO as GPIO
import time
 
#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

#set GPIO Pins
GPIO_TRIGGER_FRONT = 23
GPIO_ECHO_FRONT = 22

GPIO_TRIGGER_LEFT = 25
GPIO_ECHO_LEFT = 9

GPIO_TRIGGER_RIGHT = 12
GPIO_ECHO_RIGHT = 6
 
FRONT = 0
LEFT = 1
RIGHT = 2
SONAR_ERROR = -1
SPEED_SOUND = 34300

#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER_FRONT, GPIO.OUT)
GPIO.setup(GPIO_ECHO_FRONT, GPIO.IN)
GPIO.setup(GPIO_TRIGGER_LEFT, GPIO.OUT)
GPIO.setup(GPIO_ECHO_LEFT, GPIO.IN)
GPIO.setup(GPIO_TRIGGER_RIGHT, GPIO.OUT)
GPIO.setup(GPIO_ECHO_RIGHT, GPIO.IN)
 
# Enter FRONT, LEFT, or RIGHT to get that direction sonar
def distance(direction):

    if direction is FRONT:
        TRIGGER = GPIO_TRIGGER_FRONT
        ECHO = GPIO_ECHO_FRONT
    elif direction is LEFT:
        TRIGGER = GPIO_TRIGGER_LEFT
        ECHO = GPIO_ECHO_LEFT
    else:
        TRIGGER = GPIO_TRIGGER_RIGHT
        ECHO = GPIO_ECHO_RIGHT

    # set Trigger to HIGH
    GPIO.output(TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(ECHO) == 0:
        StartTime = time.time()
        if (StartTime - StopTime >= 0.5):
            return SONAR_ERROR
 
    # save time of arrival
    while GPIO.input(ECHO) == 1:
        if (StopTime - StartTime >= 0.5):
            return SONAR_ERROR
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * SPEED_SOUND) / 2
 
    return distance

def average_distance(direction):
    times = 5
    dist_total = 0
    for i in range(times):
        dist = distance(direction)
        dist_total += dist
        if dist is SONAR_ERROR:
            return SONAR_ERROR
    return dist_total / times

def cleanup():
    GPIO.cleanup()

def get_dist_string(dist):
    return "%.1f cm" % dist
 
if __name__ == '__main__':
    try:
        while True:
            dist_front = distance(FRONT)
            dist_left = distance(LEFT)
            dist_right = distance(RIGHT)
            print ("FRONT = %.1f cm\nLEFT = %.1f cm\nRIGHT = %.1f cm" % (dist_front, dist_left, dist_right))
            time.sleep(1)
 
        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()