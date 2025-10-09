import keyboard
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')

print("Connected")

print("Use arrow keys to control the robot. Press ESC to exit.")

left_motor = sim.getObject('/leftMotor')
right_motor = sim.getObject('/rightMotor')

def move_forward():
    sim.setJointTargetVelocity(left_motor, 2.0)
    sim.setJointTargetVelocity(right_motor, 2.0)

def move_backward():
    sim.setJointTargetVelocity(left_motor, -2.0)
    sim.setJointTargetVelocity(right_motor, -2.0)

def turn_left():
    sim.setJointTargetVelocity(left_motor, -1.0)
    sim.setJointTargetVelocity(right_motor, 1.0)

def turn_right():
    sim.setJointTargetVelocity(left_motor, 1.0)
    sim.setJointTargetVelocity(right_motor, -1.0)

def stop():
    sim.setJointTargetVelocity(left_motor, 0.0)
    sim.setJointTargetVelocity(right_motor, 0.0)

while True:
    if keyboard.is_pressed("up") or keyboard.is_pressed("w"):
        move_forward()
    elif keyboard.is_pressed("down") or keyboard.is_pressed("s"):
        move_backward()
    elif keyboard.is_pressed("left") or keyboard.is_pressed("a"):
        turn_left()
    elif keyboard.is_pressed("right") or keyboard.is_pressed("d"):
        turn_right()
    else:
        stop()









    
    if keyboard.is_pressed("esc"):
        stop()
        break
