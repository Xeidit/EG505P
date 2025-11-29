import keyboard
import time
import numpy as np
import cv2
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import os

client = RemoteAPIClient()
sim = client.getObject('sim')

v_Max = 3 # Units/sec
acceleration = 12 # Units/sec^2
cornering_weight = 0.9
min_corner_speed = 0.7

current_left = 0
current_right = 0
target_left = 0
target_right = 0

#Gets the number of training photos already taken
#leftCount = len([f for f in os.listdir("TestData/Left") if os.path.isfile(os.path.join("TestData/Left", f))])
#rightCount = len([f for f in os.listdir("TestData/Right") if os.path.isfile(os.path.join("TestData/Right", f))])
centreCount = len([f for f in os.listdir("TestData/Centre") if os.path.isfile(os.path.join(f"TestData/Centre", f))])
centreLeftCount = len([f for f in os.listdir("TestData/CentreLeft") if os.path.isfile(os.path.join("TestData/CentreLeft", f))])
centreRightCount = len([f for f in os.listdir("TestData/CentreRight") if os.path.isfile(os.path.join("TestData/CentreRight", f))])

dt = 0.025

print("Connected")
print("Use arrow keys to control the robot. Press ESC to exit.")

#Get robot parts
left_motor = sim.getObject('/leftMotor')
right_motor = sim.getObject('/rightMotor')
vision_sensor = sim.getObject('/cam1')

def stop():
        current_left = 0 
        current_right = 0

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

#define controls
while True:
    if keyboard.is_pressed("up") or keyboard.is_pressed("w"):# Go forward
        target_left = v_Max
        target_right = v_Max
        input_given = True

    if keyboard.is_pressed("down") or keyboard.is_pressed("s"): #Reverse
        target_left = 0 - v_Max
        target_right = 0 - v_Max
        input_given = True

    if keyboard.is_pressed("left") or keyboard.is_pressed("a"): #turn left
        target_left = target_left * (1-cornering_weight)
        input_given = True
        if target_left <= 0.8 and target_right <= 0.8:
            target_left = - min_corner_speed
            target_right = min_corner_speed

    if keyboard.is_pressed("right") or keyboard.is_pressed("d"): #turn right
        target_right = target_right * (1-cornering_weight)   
        input_given = True
        if target_left <= 0.8 and target_right <= 0.8:
            target_left =  min_corner_speed
            target_right = - min_corner_speed

    if input_given == False: # robot stops when there is not input
        target_right = 0
        target_left = 0
        
    input_given = False
    current_left = ramp_vel(current_left,target_left,acceleration,dt)
    current_right = ramp_vel(current_right,target_right,acceleration,dt)

    sim.setJointTargetVelocity(left_motor, current_left)
    sim.setJointTargetVelocity(right_motor, current_right)

    #print("Left (Current,Target): ",current_left, target_left , "Right(Current,Target): ", current_right, target_right)

    image, resolution = sim.getVisionSensorImg(vision_sensor)
    image = np.frombuffer(image, dtype=np.uint8).reshape(resolution[1], resolution[0], 3)
    image = cv2.flip(image, 0)  # Flip vertically

    cv2.waitKey(1)

    #Display image
    image = cv2.resize(image,(1024,512))
    cv2.imshow('Vision Sensor Feed', image)

    #End Sim    
    if keyboard.is_pressed("esc"):
        stop()
        break
    time.sleep(dt)

    #Get simulation time
    dt = sim.getSimulationTimeStep()

    #Save Training Images
    '''
    if keyboard.is_pressed("d") and not keyboard.is_pressed("w") and leftCount <= (2*min(leftCount,rightCount,centreCount,centreLeftCount,centreRightCount)+50 and not keyboard.is_pressed("p")): #Capures pictures of too far left when you press the right key 
        leftCount = leftCount + 1 #Ensures that there is not one class massively over represented compared to others
        fileName = "TestData/Left/" + str(leftCount) + ".png"
        cv2.imwrite(fileName,image)
        print("Left Image Saved")

    if keyboard.is_pressed("a") and not keyboard.is_pressed("w") and rightCount <= (2*min(leftCount,rightCount,centreCount,centreLeftCount,centreRightCount)+50 and not keyboard.is_pressed("p")): #Captures pictures of too far right when you press left key
        rightCount = rightCount + 1
        fileName = "TestData/Right/" + str(rightCount) + ".png"
        cv2.imwrite(fileName,image)
        print("Right Image Saved")
    '''
    if not keyboard.is_pressed("p"): #Only capture images if key p is pressed
        continue

    if keyboard.is_pressed("w") and not keyboard.is_pressed("a") and not keyboard.is_pressed("d") and centreCount <= (1.2*min(centreCount,centreLeftCount,centreRightCount)+50): 
        centreCount = centreCount + 1 #Captures pictures when you press the w key
        fileName = "TestData/Centre/" + str(centreCount) + ".png"
        cv2.imwrite(fileName,image)
        print("Centre Image Saved #", centreCount)

    if keyboard.is_pressed("w") and keyboard.is_pressed("a") and centreLeftCount <= ((1.2*min(centreCount,centreLeftCount,centreRightCount)+50)): 
        centreLeftCount = centreLeftCount + 1 #Captures pictures when you press the w and a key
        fileName = "TestData/CentreLeft/" + str(centreLeftCount) + ".png"
        cv2.imwrite(fileName,image)
        print("CentreLeft Image Saved #", centreLeftCount)

    if keyboard.is_pressed("w") and keyboard.is_pressed("d") and centreRightCount <= ((1.2*min(centreCount,centreLeftCount,centreRightCount)+50)): 
        centreRightCount = centreRightCount + 1 #Captures pictures when you press the w and d key
        fileName = "TestData/CentreRight/" + str(centreRightCount) + ".png"
        cv2.imwrite(fileName,image)
        print("CentreRight Image Saved #", centreRightCount)

    print("Total Data:",centreCount+centreLeftCount+centreRightCount)



