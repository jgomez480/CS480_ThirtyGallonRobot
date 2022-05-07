# This script is for testing to make sure the raspberry pi camera is working
# will save image to .jpg file if -v is not used to show image

import cv2
import sys

cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)

# set dimensions
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440)

def main():# take frame
    ret, frame = cap.read()
    if '-v' in sys.argv: 
        cv2.imshow("Camera", frame)
    else: 
        cv2.imwrite("temp.jpg", frame)

    print("frame captured")

while True:
    main()
    if cv2.waitKey(10) == 27:
        break