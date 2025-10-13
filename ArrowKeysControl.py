import keyboard
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')

v_Max = 3 # Units/sec
acceleration = 4 # Units/sec^2
cornering_weight = 0.9
min_corner_speed = 0.7

current_left = 0
current_right = 0
target_left = 0
target_right = 0

dt = 0.1

print("Connected")
print("Use arrow keys to control the robot. Press ESC to exit.")

left_motor = sim.getObject('/leftMotor')
right_motor = sim.getObject('/rightMotor')
#right_motor = sim.getObject('PioneerP3DX')


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
    if keyboard.is_pressed("up") or keyboard.is_pressed("w"):
        target_left = v_Max
        target_right = v_Max
        input_given = True

    if keyboard.is_pressed("down") or keyboard.is_pressed("s"):
        target_left = 0 - v_Max
        target_right = 0 - v_Max
        input_given = True

    if keyboard.is_pressed("left") or keyboard.is_pressed("a"):
        target_left = target_left * (1-cornering_weight)
        input_given = True
        if target_left <= 0.8 and target_right <= 0.8:
            target_left = - min_corner_speed
            target_right = min_corner_speed

    if keyboard.is_pressed("right") or keyboard.is_pressed("d"):
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

    print("Left (Current,Target): ",current_left, target_left , "Right(Current,Target): ", current_right, target_right)

    if keyboard.is_pressed("esc"):
        stop()
        break
    time.sleep(dt)

    # Get position data
