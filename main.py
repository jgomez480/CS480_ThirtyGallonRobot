# This is our main script for poseidon wayfinding. It includes
# both obstacle avoidance and end of the hall detection. 
# Refer to obstacle_avoidance.py for obstacle avoidance only

import cv2
import freenect
from keras.models import load_model
import motor_driver as motors # motor_driver.py
import numpy as np
import sonar # sonar.py
import sys
import time
from timer import Timer # timer.py
import threading
import tensorflow as tf


if '-v' in sys.argv: 
    show_previews = True
else: 
    show_previews = False

def display_image(window_name, image): # Putting the -v arg in cmd will show relevant images (only works on monitor display)
    if show_previews:
        cv2.imshow(window_name, image)

cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L) # Rapsberry pi camera
# set dimensions
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

img_width, img_height = 60, 60 
Model = load_model('machine_learning/5_2_Hall_door_lobby_dectection.hdf5') 
def single_frame_predict(camera_frame):
    Label = ['Door', 'Hall', 'Lobby']
    try:
        frame = cv2.resize(camera_frame, (img_width, img_height), interpolation = cv2.INTER_AREA)
        frame = frame[..., ::-1]
        frame = tf.expand_dims(frame, axis=0)
        result = np.argmax(Model.predict(frame))
        return Label[result]
    except Exception as e:
        print("Image Error")

end_lobby_counter = 0
end_door_counter = 0
def check_for_lobby():# take frame
    global end_lobby_counter
    global end_door_counter

    while not stop_program: # If boths ends are found this bool is True
        for n in range(8): # Read multiple times to properly skip frames for time delay
            ret, frame = cap.read()
        display_image("Pi Camera", frame)

        result = single_frame_predict(frame)
        if result == "Lobby":
            end_lobby_counter += 1
            print("Result = " + result
            + "     Lobby +++: " + str(end_lobby_counter)
            + "  |  Door +++: " + str(end_door_counter))
        elif result == "Door":
            end_door_counter += 1
            print("Result = " + result
            + "     Lobby +++: " + str(end_lobby_counter)
            + "  |  Door +++: " + str(end_door_counter))
        else:
            end_lobby_counter = 0
            end_door_counter = 0

        if cv2.waitKey(10) == 27:
            break
    cap.release()



# CONSTANTS
KINECT_WIDTH = 640    # Kinect Image Width
KINECT_HEIGHT = 480   # Kinect Image Height
INIT_THRESHOLD = 180  # Initial value to change depth image gradient into binary image
INIT_DEPTH = 914      # Initial value to change depth image gradient into binary image
DEPTH_HEIGHT = 440    # From 480 - cut off bottom to not detect floor 
DEPTH_WIDTH = 620     # From 640 - cut off right side, kinect sometimes detects a random strip on right side
OBSTACLE_CHECK = True # find_contours() check for obstacle bool
OBSTACLE_AREA = 110000# How many pixels should be found to detect obstacle
OBSTACLE_WAIT = 3     # How long robot waits after detecting obstacle
OBSTACLE_AREA_SIDE = 9500      # How many pixels for detection on side of camera
OBSTACLE_AREA_TOO_WIDE = 45000 # If there are enough pixels on side but more than this, its too wide to turn away from
LEFT = True           # Direction/Side is to left
RIGHT = False         # Direction/Side is to right
OPEN_CHECK = False    # find_contours() check for opening bool
OPEN_WIDTH = 420      # total width for opening detect, goes from middle (640 / 2) out equally
OPEN_AREA = 130000    # How many pixels should be clear to detect opening
OPEN_TURN_TIMER = 3.5 # How long robot turns when trying to find opening
OPEN_ROTATIONS = 2    # How many rotations before cutting threshold_val value
LOBBY = True          # Is end the lobby (north end)
DOOR = False          # Is end the door (south end)
 
# STATES 
STATE_MOVE_FORWARD   = 0 # Start moving forward 
STATE_CHECK_OBSTACLE = 1 # Check for obstacles
STATE_FIND_OPENING   = 2 # Check for opening through obstacles
STATE_CLEAR_OBSTACLE = 3 # Make sure the obstacle is completly out of view

# Variables
state = STATE_MOVE_FORWARD
threshold_val = INIT_THRESHOLD
depth_val = INIT_DEPTH
obstacle = False        # Is an obstacle in the way
moving = False          # Is the robot rotating
moving_direction = LEFT # Which direction is the bot currently rotating
rotation_counter = 0    # How many times has the robot changed turn direction looking for opening
opening_stops = 0       # How many times has robot stopped while rotating for opening
stop_program = False    # Bool to terminate detection thread
first_rotation = True   # Is this the first rotation upon finding object
looking_for = DOOR     # Which end to look for first
first_found = True     # Has the first end been found
stop_sonars = False     # Stop checking sonars for walls
current_timer = Timer() # Timer used for various things

left_sonar_measurements = []
right_sonar_measurements = []
SONAR_WALL_DIST = 60
SONAR_WALL_AVG  = 600
SONAR_LAST_DIST = 45


# Returns true if the sonars detect that they are approaching a wall
# If sonar measurements are below certain distance and are continually decreasing
def sonar_wall_check(direction):
    if direction is LEFT:
        sonars = left_sonar_measurements
    else:
        sonars = right_sonar_measurements

    count = 8
    lowest = 1300
    highest = 0
    decreases = 0

    if len(sonars) >= count:
        total = 0
        for index in range(len(sonars)):
            dist = sonars[index]
            if dist > highest:
                highest = dist
            if dist < lowest:
                lowest = dist
            if index != 0:
                if dist - sonars[index - 1] < 0:
                    decreases += 1
                else:
                    decreases -= 1
            total += dist
        avg = total / count
        diff = highest - lowest
        sonars.pop(0)
        if lowest <= SONAR_WALL_DIST and avg <= SONAR_WALL_AVG and \
        (sonars[len(sonars) - 1] <= SONAR_LAST_DIST or sonars[len(sonars) - 2] <= SONAR_LAST_DIST) \
        and decreases > 0:
            return True
    return False


def empty_sonar_list():
    global left_sonar_measurements
    global right_sonar_measurements
    left_sonar_measurements = []
    right_sonar_measurements = []

def change_threshold_val(value):
    global threshold_val
    threshold_val = value

def change_depth(value):
    global depth_val
    depth_val = value

def change_state(new_state):
    global state
    state = new_state

# Returns contours of depth image, reversed and takes middle if for opening
def find_contours(frame, depth, obstacle_find):
    depth_gray = depth.copy() 
    contour_img = frame.copy()

    if obstacle_find:
        depth_gray = depth_gray[0:DEPTH_HEIGHT, 0:DEPTH_WIDTH] # Cuts bottom so it doesnt detect floor
    else: # Gives contours of only middle OPEN_WIDTH of screen
        right_width = int(KINECT_WIDTH - (KINECT_WIDTH - OPEN_WIDTH) / 2)
        left_width = int((KINECT_WIDTH - OPEN_WIDTH) / 2) 
        depth_gray = (255-depth_gray)
        depth_gray = depth_gray[0:DEPTH_HEIGHT, left_width:right_width]
        contour_img = contour_img[0:DEPTH_HEIGHT, left_width:right_width]

    contours, hierarchy = cv2.findContours(depth_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(contour_img, contours, -1, (255, 0, 0), 3)
    display_image('Contours', contour_img)
    return contours


# Returns contours of piece of depth image from bottom half
def find_contours_corner(frame, depth, obs, width_min, width_max):
    depth_gray = depth.copy() 
    contour_img = frame.copy()
    HEIGHT_MAX = 240
    depth_gray = depth_gray[HEIGHT_MAX:KINECT_HEIGHT, width_min:width_max]
    contour_img = contour_img[HEIGHT_MAX:KINECT_HEIGHT, width_min:width_max]
    if not obs:
        depth_gray = (255-depth_gray)
    contours, hierarchy = cv2.findContours(depth_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(contour_img, contours, -1, (255, 0, 0), 3)
    display_image('Contours', contour_img)
    return contours



def check_for_obstacle(frame, depth, raw_depth):
    global stop_sonars
    if not current_timer.has_started():
        current_timer.start()
    
    dist_front = sonar.average_distance(sonar.FRONT)
    if dist_front is sonar.SONAR_ERROR:
        print("Sonar Error")
        return

    contours = find_contours(frame, depth, OBSTACLE_CHECK)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        obstacle_found = False
        if (area >= OBSTACLE_AREA and dist_front <= 200) or area >= OBSTACLE_AREA * 1.4:
            print(dist_front)
            print("Obstacle Found, Waiting " + str(OBSTACLE_WAIT) + " seconds ---------------------------------\n") 
            motors.stop()
            time.sleep(OBSTACLE_WAIT)
            change_threshold_val(150)
            change_state(STATE_FIND_OPENING)
            if current_timer.has_started():
                current_timer.stop()
            return

    shorter_depth = 255 * np.logical_and(raw_depth >= depth_val - 228, 
                        raw_depth <= depth_val + 228)
    shorter_depth = shorter_depth.astype(np.uint8)
    shorter_depth = (255-shorter_depth)
    cv2.imwrite("Raw_depth.jpg", shorter_depth)

    greatest = 0
    contours = find_contours_corner(frame, shorter_depth, OBSTACLE_CHECK, 0, 640)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > greatest:
            greatest = area
    print("Greatest Area: " + str(greatest))

    contours = find_contours_corner(frame, shorter_depth, OBSTACLE_CHECK, 0, 200)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 500:
            print("On left side: " + str(area))
        if area >= OBSTACLE_AREA_SIDE:
            if greatest >= OBSTACLE_AREA_TOO_WIDE:
                print("Obstacle Too Wide on Left Side")
                return
            print("Obstacle Found on Left Side \n") 
            motors.stop()
            motors.move_right()
            time.sleep(0.3)
            stop_sonars = True
            change_state(STATE_MOVE_FORWARD)
            if current_timer.has_started():
                current_timer.stop()
            return

    contours = find_contours_corner(frame, shorter_depth, OBSTACLE_CHECK, 440, 640)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 500:
            print("On right side: " + str(area))
        if area >= OBSTACLE_AREA_SIDE:
            if greatest >= OBSTACLE_AREA_TOO_WIDE:
                print("Obstacle Too Wide on Right Side")
                return
            print("Obstacle Found on Right Side \n") 
            motors.stop()
            motors.move_left()
            time.sleep(0.3)
            stop_sonars = True
            change_state(STATE_MOVE_FORWARD)
            if current_timer.has_started():
                current_timer.stop()
            return

    # Sonar Wall Check -----------------------------------------------------------------

    if current_timer.get_time() > 1.5 and stop_sonars:
        print("Sonar ----------------------------")
        stop_sonars = False

    if current_timer.get_time() < 0.25 or stop_sonars:
        return

    current_timer.stop()

    dist_left = sonar.distance(sonar.LEFT)
    left_sonar_measurements.append(dist_left)
    if dist_left <= SONAR_LAST_DIST:
        print("Left: " + str(dist_left))

    dist_right = sonar.distance(sonar.RIGHT)
    right_sonar_measurements.append(dist_right)
    if dist_right <= SONAR_LAST_DIST:
        print("Right: " + str(dist_right))

    if sonar_wall_check(LEFT):
        print("Too close to wall on left side, turning right")
        motors.move_right()
        time.sleep(0.1)
        empty_sonar_list()
        motors.move_forward
        time.sleep(0.5)
        stop_sonars = True
        change_state(STATE_MOVE_FORWARD)
    elif sonar_wall_check(RIGHT):
        print("Too close to wall on right side, turing left")
        motors.move_left()
        time.sleep(0.1)
        empty_sonar_list()
        motors.move_forward
        time.sleep(0.5)
        stop_sonars = True
        change_state(STATE_MOVE_FORWARD)


# Returns the side in which the obstacle is primarily on in order to turn
# in the opposite direction
def find_obstacle_side(frame, depth):
    image_half = DEPTH_WIDTH / 2
    contours = find_contours(frame, depth, OBSTACLE_CHECK)
    obstacle = False
    result = RIGHT
    dist_front = sonar.average_distance(sonar.FRONT)
    print(dist_front)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if (area >= 500):
            print(area)
        if (area >= OBSTACLE_AREA and dist_front <= 100) or area >= OBSTACLE_AREA * 1.2:
            obstacle = True
            # Counts contour pixels on left side 0-309 and right 310-619
            left_side_count = 0
            right_side_count = 0
            for pixel in cnt:
                x = pixel[0][0]
                if x < image_half:
                    left_side_count += 1
                else:
                    right_side_count += 1

            if (left_side_count >= right_side_count): # If opening is on right obstacle on left
                result = LEFT

            # Because of the false positives of the hallway, usually if the area is over these
            # numbers, flipping them creates the right result
            if (left_side_count >= 1400 and right_side_count >= 1400 and dist_front >= 40):
                result = not result
    return obstacle, result


def rotate_find_openning(frame, depth):
    global moving
    global first_rotation
    global rotation_counter
    global moving_direction
    global opening_stops

    keep_rotating = False

    # If hasnt determined already, find side which obstacle is on
    # Rotate toward the opposite side
    if not moving:
        moving = True
        obstacle, side_on = find_obstacle_side(frame, depth)
        if not obstacle:
            print("Opening Found, Obstacle Moved --------------------------------------------")
            moving = False
            change_threshold_val(INIT_THRESHOLD)
            change_state(STATE_MOVE_FORWARD)
            return
        change_threshold_val(230)
        if side_on is LEFT:
            print("Obstacle On Left Side|XXXXXXXXXXXXXXXXXXX")
            print(">>>>> Turning Right >>>>>")
            moving_direction = RIGHT
            motors.move_right()
        else:
            print("XXXXXXXXXXXXXXXXXXX|Obstacle On Right Side")
            print("<<<<< Turning Left <<<<<")
            moving_direction = LEFT
            motors.move_left()

    # otherwise check from a section in the middle of the camera until there is an openning
    # and return to normal moving state
    else:
        if (opening_stops >= 2 and first_rotation) or (opening_stops >= 4 and not first_rotation):
            first_rotation = False
            opening_stops = 0
            rotation_counter += 1
            if moving_direction is LEFT:
                moving_direction = RIGHT
                motors.move_right()
                print(">>>>> Moving Left to Right >>>>>")
            else:
                moving_direction = LEFT
                motors.move_left()
                print("<<<<< Moving Right to Left <<<<<")

            if rotation_counter >= OPEN_ROTATIONS:
                print("Too many rotations lowering depth")
                change_threshold_val(threshold_val + 100)
                rotation_counter = 0

        contours = find_contours(frame, depth, OPEN_CHECK)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area >= OPEN_AREA:
                new_contours = find_contours_corner(frame, depth, OBSTACLE_CHECK, 0, 640)
                for new_cnt in new_contours:
                    new_area = cv2.contourArea(new_cnt)
                    if new_area >= 15000:
                        print("Something still in the way continuing rotation")
                        keep_rotating = True

                if keep_rotating:
                    keep_rotating = False
                    continue
                print("Opening Found, Finding Room To Clear ------------")
                rotation_counter = 0
                first_rotation = True
                moving = False
                change_state(STATE_CLEAR_OBSTACLE)
                if current_timer.has_started():
                    current_timer.stop()

# Keeps turning until obstacle is out of view
def clear_obstacle(frame, depth):
    if moving_direction is RIGHT:
        contours = find_contours_corner(frame, depth, OPEN_CHECK, 0, 240)
    else:
        contours = find_contours_corner(frame, depth, OPEN_CHECK, 400, 640)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area >= 48000:
            print("Free to move " + str(area))
            change_state(STATE_MOVE_FORWARD)
            change_threshold_val(INIT_THRESHOLD)    
            

# Uses sonars in addition to find an opening, this is primarily for seeing
# down the hall because of false positives from kinect sensor
def try_opening(frame, depth):
    global opening_stops
    global moving
    global first_rotation
    global rotation_counter

    keep_rotating = False

    if not current_timer.has_started():
        current_timer.start()
    elapsed_time = current_timer.get_time()
    if elapsed_time < 0.2:
        return

    print("Stopping, checking forward")
    motors.stop()
    time.sleep(0.5)
    current_timer.stop()
    opening_stops += 1
    dist = sonar.average_distance(sonar.FRONT)
    contours = find_contours(frame, depth, OPEN_CHECK)
    obstacle = False
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area >= OPEN_AREA * 0.6 and dist >= 80:
            print("AREA " + str(area) + " DIST " + str(dist))
            obstacle = True
            first_rotation = True
            moving = False
            opening_stops = 0
            rotation_counter = 0
            if current_timer.has_started():
                current_timer.stop()
            print("Opening Found, Continuing -----------------------------------------")
            new_contours = find_contours_corner(frame, depth, OBSTACLE_CHECK, 0, 640)
            con_img = frame.copy()
            cv2.drawContours(con_img, new_contours, -1, (255, 0, 0), 3)
            display_image('test', con_img)
            for new_cnt in new_contours:
                new_area = cv2.contourArea(new_cnt)
                if new_area >= 15000:
                    print("Something still in the way continuing rotation")
                    keep_rotating = True
            
            if keep_rotating:
                keep_rotating = False
                continue
            change_threshold_val(INIT_THRESHOLD)
            change_state(STATE_MOVE_FORWARD)


    if not obstacle:
        if moving_direction is LEFT:
            motors.move_left()
        else:
            motors.move_right()


def get_kinect_image():
    frame = freenect.sync_get_video()[0]
    frame = frame[:, :, ::-1] # Converts video into a BGR format for display
    depth, timestamp = freenect.sync_get_depth()
    initial_depth = depth.copy()
    depth = 255 * np.logical_and(depth >= depth_val - threshold_val, 
                                 depth <= depth_val + threshold_val)
    depth = depth.astype(np.uint8)
    depth = (255-depth)

    display_image('Depth Gradient', initial_depth)
    display_image('Depth Cutoff', depth)
    return frame, depth, initial_depth

# Returns true if model predicts end of the hallway
# Does not count door if there is an obstacle (model usually defaults to door when obstacle is present)
def is_end_found(frame, depth):
    if looking_for == LOBBY:
        return end_lobby_counter >= 6
    elif looking_for == DOOR:
        contours = find_contours(frame, depth, OPEN_CHECK)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area >= OPEN_AREA * 1.2:
                print("Open enough to check for door")
                return end_door_counter >= 9
    return False


# If end is found will turn around or end program based on program variables
def check_for_end(frame, depth):
    global looking_for
    global first_found
    global stop_program

    if is_end_found(frame, depth):
        if looking_for == LOBBY:
            print("\nEND OF HALLWAY - LOBBY")
        else:
            print("\nEND OF HALLWAY - DOOR")
            
        if not first_found:
            print("TURNING AROUND\n")
            first_found = True
            looking_for = not looking_for
            motors.move_right()
            time.sleep(motors.TIME_180)
            empty_sonar_list()
            change_state(STATE_MOVE_FORWARD)
        else:
            print("STOPPING PROGRAM")
            stop_program = True
            lobby_detection_thread.join()
            exit()


# Grabs kinect image and does computation based on current state of program
def main():
    frame, depth, initial_depth = get_kinect_image()

    if state == STATE_MOVE_FORWARD:
        motors.move_forward()
        print("Moving Forward")
        change_state(STATE_CHECK_OBSTACLE)
        print("Checking For Obstacle")
    elif state == STATE_CHECK_OBSTACLE:
        check_for_obstacle(frame, depth, initial_depth)
        check_for_end(frame, depth)
    elif state == STATE_FIND_OPENING:
        rotate_find_openning(frame, depth)
        try_opening(frame, depth)
    elif state == STATE_CLEAR_OBSTACLE:
        clear_obstacle(frame, depth)




# First time getting kinect image sometimes hangs
# Open once before moving to make sure kinect works first
print("Trying To Open Kinect...")
temp, temp, temp = get_kinect_image()
print("Kinect Opened")

# Starts the thread responsible for detecting end of the hallway
lobby_detection_thread = threading.Thread(target=check_for_lobby, args=())
lobby_detection_thread.start()


try:
    while True:
        main()
        if cv2.waitKey(10) == 27: # Needed for displaying cv2 images if -v is used
            break

    # If stopped with Ctrl + C, will stop the motors and sonars
except KeyboardInterrupt:
    print("Program Stopped With Ctrl+C")
    motors.cleanup()
    sonar.cleanup()
