# This script just tests to see if the kinect sensor is working
# will save image to .jpg file if -v is not used to show image

import freenect
import cv2
import numpy as np
import time
import sys


if '-v' in sys.argv: 
    show_previews = True
else: 
    show_previews = False

def display_image(window_name, image): # Putting the -v arg in cmd will show relevant images
    if show_previews:
        cv2.imshow(window_name, image)


threshold = 228
current_depth = 914

def change_threshold(value):
    global threshold
    threshold = value

def change_depth(value):
    global current_depth
    current_depth = value


first = True
def show_depth():
    global threshold
    global current_depth
    global first

    frame = freenect.sync_get_video()[0]
    frame = frame[:, :, ::-1] # Converts video into a BGR format for display
    depth = freenect.sync_get_depth()[0]
    initial_depth = depth.copy()
    initial_depth = depth.astype(np.uint8)
    depth = 255 * np.logical_and(depth >= current_depth - threshold, depth <= current_depth + threshold)
    depth = depth.astype(np.uint8)

    print("Working")

    depth = (255-depth)
    display_image('Depth', depth)
    display_image('Video', initial_depth)

    if not show_previews:
        cv2.imwrite("kinect_depth.jpg", depth)
        cv2.imwrite("kinect_initaldepth.jpg", initial_depth)

    depth_gray = depth.copy() 
    contour_img = frame.copy()

    contours, hierarchy = cv2.findContours(depth_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(contour_img, contours, -1, (255, 0, 0), 3)
    display_image('Contours', contour_img)
    first_frame = False

if show_previews:
    cv2.namedWindow('Depth')
    cv2.namedWindow('Video')
    cv2.createTrackbar('threshold', 'Depth', threshold,     500,  change_threshold)
    cv2.createTrackbar('depth',     'Depth', current_depth, 2048, change_depth)

print('Press ESC in window to stop')

while True:
    show_depth()
    if cv2.waitKey(10) == 27:
        break