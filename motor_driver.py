import RPi.GPIO as GPIO # Needed for the GPIO pins
import time

# LEFT MOTOR
motor_a = 14 # Pin 12
motor_a_dir = 4  # Pin 11

# RIGHT MOTOR
motor_b = 16 # Pin 36
motor_b_dir = 19  # Pin 35

# Left motor a moves forward when direction pin is set low. Right motor b moves backward when dir pin set low
a_direction_fwd = GPIO.HIGH
a_direction_back = GPIO.LOW
b_direction_fwd = GPIO.LOW
b_direction_back = GPIO.HIGH

# Standard, prevents errors
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(motor_a, GPIO.OUT) 
GPIO.output(motor_a, GPIO.LOW)
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

end_power = 9    # End speed as percentage
sleep_time = 0.08 # Time between each speed increase
b_motor_add = 8  # One motor is different than the other, seems to be linear though

TIME_180 = 4   # How much time to sleep after turn for full 180 rotation
TIME_90  = 2

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


def cleanup():
    GPIO.cleanup()