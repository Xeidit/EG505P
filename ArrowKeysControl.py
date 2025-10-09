import keyboard
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')

v_Max = 3
ramp = 0.1
current_left = 0
current_right = 0

print("Connected")
print("Use arrow keys to control the robot. Press ESC to exit.")

left_motor = sim.getObject('/leftMotor')
right_motor = sim.getObject('/rightMotor')

def stop():
        current_left = 0 
        current_right = 0

while True:
    if keyboard.is_pressed("up") or keyboard.is_pressed("w"):
        current_left = current_left + ramp  
        current_right = current_right + ramp

    if keyboard.is_pressed("down") or keyboard.is_pressed("s"):
        current_left = current_left - ramp  
        current_right = current_right - ramp

    if keyboard.is_pressed("left") or keyboard.is_pressed("a"):
        current_left = current_left - ramp  
        current_right = current_right + ramp        

    if keyboard.is_pressed("right") or keyboard.is_pressed("d"):
        current_left = current_left + ramp  
        current_right = current_right - ramp        

    else:
        stop()

    sim.setJointTargetVelocity(left_motor, current_left)
    sim.setJointTargetVelocity(right_motor, current_right)

    print("Left: ", left_motor, "Right: ", right_motor)


    if keyboard.is_pressed("esc"):
        stop()
        break
    time.sleep(0.1)
