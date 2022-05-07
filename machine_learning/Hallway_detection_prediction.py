# -*- coding: utf-8 -*-
"""
Created on Thu Mar  3 21:49:12 2022

@author: 37577
"""

from keras.models import load_model
import time 
import tensorflow as tf
import numpy as np
import cv2


img_width, img_height = 60, 60
Model = load_model('Hall_door_lobby_dectection.hdf5') 
file_name = "test/door/door1_frame3297.jpg"



def single_frame_predict(file_name):
    Label = ['Door','Hall','Lobby']
    frame_ori = cv2.imread(file_name)
    frame = cv2.resize(frame_ori, (img_width, img_height),interpolation = cv2.INTER_AREA)
    frame = frame[..., ::-1]
    frame = tf.expand_dims(frame, axis=0)
    # print(np.argmax(Model.predict(frame)))
    result = np.argmax(Model.predict(frame))
    return Label[result]

detection_result = single_frame_predict(file_name)
print(detection_result)

def video_prediction(file_name):
    Label = ['Door','Hall','Lobby']
    vidcap = cv2.VideoCapture(file_name)
    sucess, frame_ori = vidcap.read()
    count = 0
    while sucess:
        frame = cv2.resize(frame_ori, (img_width, img_height),interpolation = cv2.INTER_AREA)
        frame = tf.expand_dims(frame, axis=0)
        sucess, frame_ori = vidcap.read()
        result = np.argmax(Model.predict(frame))
        print('frame #' + str(count) + '  ' + Label[result])
        count += 1
    vidcap.release()
        
        
video = "tgr_videos/Door/door1.mp4"
video_prediction(video)
# sucess, frame_ori = cv2.VideoCapture(video)
# start = time.time()
# for i in range(100):
#     detection_result = single_frame_predict(file_name)
# end = time.time()

# print('processing time is ', end - start)

