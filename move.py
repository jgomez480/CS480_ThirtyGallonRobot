# This script is used to have some basic movement for the robot
# entering f, l, r, b will move robot in direction 
# forwards, left, right, and backwards
import RPi.GPIO as GPIO # Needed for the GPIO pins
import time

# LEFT MOTOR
motor_a = 14 # Pin 12
motor_a_dir = 4  # Pin 11
# RIGHT MOTOR
motor_b = 16 # Pin 36
motor_b_dir = 19  # Pin 35

a_direction_fwd = GPIO.HIGH
a_direction_back = GPIO.LOW
b_direction_fwd = GPIO.LOW
b_direction_back = GPIO.HIGH

# Standard, prevents errors
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(motor_a, GPIO.OUT) 
GPIO.output(motor_a, GPIO.LOW) # Function for sending voltage to pin
GPIO.setup(motor_b, GPIO.OUT)
GPIO.output(motor_b, GPIO.LOW)
GPIO.setup(motor_a_dir, GPIO.OUT)
GPIO.output(motor_a_dir, GPIO.LOW)
GPIO.setup(motor_b_dir, GPIO.OUT)
GPIO.output(motor_b_dir, GPIO.LOW)
pwm_motor_a = GPIO.PWM(motor_a, 1000)
pwm_motor_a.start(0) # Starts at 0%

pwm_motor_b = GPIO.PWM(motor_b, 1000)
pwm_motor_b.start(0)

end_power = 6
sleep_time = 0.1
b_motor_add = 7

# Setting all direction pins
def move_forward():
    GPIO.output(motor_a_dir, a_direction_fwd)
    GPIO.output(motor_b_dir, b_direction_fwd)
    ramp_motors_up()

def move_backward():
    GPIO.output(motor_a_dir, a_direction_back)
    GPIO.output(motor_b_dir, b_direction_back)
    ramp_motors_up()

def move_left():
    GPIO.output(motor_a_dir, a_direction_back)
    GPIO.output(motor_b_dir, b_direction_fwd)
    ramp_motors_up()

def move_right():
    GPIO.output(motor_a_dir, a_direction_fwd)
    GPIO.output(motor_b_dir, b_direction_back)
    ramp_motors_up()

def stop(): # Set everything off 
    ramp_motors_down()

# Changing power % for motors
def ramp_motors_up():
    for power in range(end_power):
        pwm_motor_a.ChangeDutyCycle(power) 
        pwm_motor_b.ChangeDutyCycle(power + b_motor_add)
        time.sleep(sleep_time)

def ramp_motors_down():
    for power in reversed(range(end_power)):
        time.sleep(sleep_time)
        pwm_motor_a.ChangeDutyCycle(power) 
        pwm_motor_b.ChangeDutyCycle(power + b_motor_add)
    pwm_motor_b.ChangeDutyCycle(0)

def main(input):
    if (value == 'f'):
        print("forward")
        move_forward()
        time.sleep(1) 
        
    elif (value == 'b'):
        print("backwards")
        move_backward()
        time.sleep(1)

    elif (value == 'l'):
        print("left")
        move_left()
        time.sleep(0.5)

    elif (value == 'r'):
        print("right")
        move_right()
        time.sleep(0.5)

    elif (value == 'q'):
        GPIO.cleanup() # Standard, prevents errors
        exit()

    if value == 'f' or value == 'b' or value == 'l' or value == 'r':
        stop()


if __name__ == '__main__':
    try:
        while(True):
            value = input("Enter Command")
            main(value)
    except KeyboardInterrupt:
        print("Program Stopped With Ctrl+C")
        GPIO.cleanup()
