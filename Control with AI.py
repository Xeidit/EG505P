import keyboard
import time
import numpy as np
import cv2
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import pickle
from keras.models import load_model
import sys
import os
import matplotlib.pyplot as plt


client = RemoteAPIClient()
sim = client.getObject('sim')

v_Max = 3 # Units/sec
acceleration = 10 # Units/sec^2
cornering_weight = 0.7 #How sensative cornering is
min_corner_speed = 0.7 #Speed below when robot turns on the spot

#Initalise speed variables
current_left = 0
current_right = 0
target_left = 0
target_right = 0

#Make plot
sens_output = []
position = []
fig, ax = plt.subplots()
loc = ax.scatter([],[])
map = ax.scatter([],[])
plt.ion()
ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)

#Initalise Time Step
dt = 0.050

print("Connected")

# Get robot components
left_motor = sim.getObject('/leftMotor')
right_motor = sim.getObject('/rightMotor')
vision_sensor = sim.getObject('/cam1')
right_sensor = sim.getObject('/proxSensorRight')
left_sensor = sim.getObject('/proxSensorLeft')
robot = sim.getObject('/PioneerP3DX')

#Load Model
model = load_model("Models/Model0.87.h5")
dict_file = open("pickledData.pkl", "rb")
category_dict = pickle.load(dict_file)

#Smooth acceleration by ramping the velocity of the wheels
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
    #Get image from camera
    image, resolution = sim.getVisionSensorImg(vision_sensor)
    image = np.frombuffer(image, dtype=np.uint8).reshape(resolution[1], resolution[0], 3)
    image = cv2.flip(image, 0)  # Flip vertically
    cv2.imshow('Vision Sensor Feed', image)
    cv2.waitKey(1)

    #Process image to something the AI can understand
    test_img = cv2.resize(image, (128, 64))
    test_img = cv2.cvtColor(test_img, cv2.COLOR_BGR2GRAY)
    test_img = test_img/255
    test_img = test_img.reshape(1, 128, 64, 1)

    #Run model to predict correct direction
    results = model.predict(test_img)
    label = np.argmax(results, axis=1)[0]
    acc = int(np.max(results, axis=1)[0]*100)   

    print(f"Moving : {category_dict[label]} with {acc}% accuracy.")

    #Move
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

    if input_given == False: # Robot slows down if there is no other input
        target_right = 0
        target_left = 0
        
    input_given = False
    current_left = ramp_vel(current_left,target_left,acceleration,dt)
    current_right = ramp_vel(current_right,target_right,acceleration,dt)

    sim.setJointTargetVelocity(left_motor, current_left)
    sim.setJointTargetVelocity(right_motor, current_right)

    #End file on keypress, this requires SUDO permissions 
    if keyboard.is_pressed("esc"):
        # Save to file
        print("End")
        break

    #Plot Data
    sim.handleProximitySensor(right_sensor) # Call right sensor to take a reading
    result, distance, point, objHandle, normal = sim.readProximitySensor(right_sensor) #Get reading from right sensor

    if result:
        worldPoint = sim.multiplyVector(sim.getObjectMatrix(right_sensor, -1), point) # If it gets a reading, make into global coordinates
        sens_output.append(worldPoint[:2]) # Save to sensor output

    sim.handleProximitySensor(left_sensor) # Call left sensor to take a reading
    result, distance, point, objHandle, normal = sim.readProximitySensor(left_sensor) # Get reading from left sensor
    if result:
        worldPoint = sim.multiplyVector(sim.getObjectMatrix(left_sensor, -1), point) # if it gets a reading make into global coordinates
        sens_output.append(worldPoint[:2]) # save to sensor output

    current_position = sim.getObjectPosition(robot) # Get robot position, already in global coordinates
    if result:
        position.append(current_position[:2])  # save it

    pos_data = np.array(position) # Make data into array
    output_data = np.array(sens_output)

    map.set_offsets(output_data) #Add data to scatter graph
    loc.set_offsets(pos_data)

    fig.canvas.draw() # Update scatter graph
    fig.canvas.flush_events()
    plt.pause(0.01)
    result = 0

    #Get simulation time
    dt = sim.getSimulationTimeStep()


#Save data so robots can read it
np.save("pos_data.npy", pos_data)
np.save("out_data.npy", output_data)
print("Data saved to sensor_data.npy")

#Save data so humans can read it
with open("map.txt", 'w') as map_txt:
    for row in sens_output:
        map_txt.write(str(row) + '\n')

with open("location.txt", 'w') as loc_txt:
    for row in position:
        loc_txt.write(str(row) + '\n')
