# This script is for testing the end of hallway detection
# Simply prints prediction to console

import cv2
from keras.models import load_model
import numpy as np
import tensorflow as tf

cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L) # Rapsberry pi camera
# set dimensions
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

img_width, img_height = 60, 60
Model = load_model('machine_learning/5_2_Hall_door_lobby_dectection.hdf5') 
def single_frame_predict(camera_frame):
    Label = ['Door','Hall','Lobby']
    try:
        frame = cv2.resize(camera_frame, (img_width, img_height), interpolation = cv2.INTER_AREA)
        frame = frame[..., ::-1]
        frame = tf.expand_dims(frame, axis=0)
        result = np.argmax(Model.predict(frame))
        return Label[result]
    except Exception as e:
        print("Image Error")

def check_for_lobby():
    while True:
        for n in range(8): # Read multiple times to properly skip frames for time delay
            ret, frame = cap.read()

        result = single_frame_predict(frame)
        print(result)

        if cv2.waitKey(10) == 27:
            break
    cap.release()

check_for_lobby()