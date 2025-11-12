
import time
import numpy as np
import cv2
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import pickle
from keras.models import load_model
import sys
import os

client = RemoteAPIClient()
sim = client.getObject('sim')

v_Max = 6 # Units/sec
acceleration = 15 # Units/sec^2
cornering_weight = 0.6
min_corner_speed = 0.7

current_left = 0
current_right = 0
target_left = 0
target_right = 0

dt = 0.025

print("Connected")

left_motor = sim.getObject('/leftMotor')
right_motor = sim.getObject('/rightMotor')
vision_sensor = sim.getObject('/cam1')

#Load Model
model = load_model("Models/Model0.87.h5")
dict_file = open("pickledData.pkl", "rb")
category_dict = pickle.load(dict_file)

def stop():
        current_left = 0 
        current_right = 0

def ramp_vel(current_vel,target_vel,ramp,dt):
    if current_vel < target_vel:
        current_vel += ramp*dt
        current_vel = min(current_vel,target_vel)
    elif current_vel > target_vel:
        current_vel -= ramp*dt
        current_vel = max(current_vel,target_vel)
    return current_vel

input_given = False

while True:
    image, resolution = sim.getVisionSensorImg(vision_sensor)
    image = np.frombuffer(image, dtype=np.uint8).reshape(resolution[1], resolution[0], 3)
    image = cv2.flip(image, 0)  # Flip vertically
    cv2.imshow('Vision Sensor Feed', image)
    cv2.waitKey(1)

    test_img = cv2.resize(image, (128, 64))
    test_img = cv2.cvtColor(test_img, cv2.COLOR_BGR2GRAY)
    test_img = test_img/255
    test_img = test_img.reshape(1, 128, 64, 1)

    results = model.predict(test_img)
    label = np.argmax(results, axis=1)[0]
    acc = int(np.max(results, axis=1)[0]*100)   

    print(f"Moving : {category_dict[label]} with {acc}% accuracy.")

    if label == 0 or label == 1 or label == 2: #Go Forwards
        target_left = v_Max
        target_right = v_Max
        input_given = True

    if label == 1: #Go left
        target_left = target_left * (1-cornering_weight)
        input_given = True
        if target_left <= 0.8 and target_right <= 0.8:
            target_left = - min_corner_speed
            target_right = min_corner_speed

    if label == 2:#Go right
        target_right = target_right * (1-cornering_weight)   
        input_given = True
        if target_left <= 0.8 and target_right <= 0.8:
            target_left =  min_corner_speed
            target_right = - min_corner_speed

    if input_given == False:
        target_right = 0
        target_left = 0
        
    input_given = False
    current_left = ramp_vel(current_left,target_left,acceleration,dt)
    current_right = ramp_vel(current_right,target_right,acceleration,dt)

    sim.setJointTargetVelocity(left_motor, current_left)
    sim.setJointTargetVelocity(right_motor, current_right)

    #Get simulation time
    dt = sim.getSimulationTimeStep()

