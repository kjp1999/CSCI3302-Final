import numpy as np
from controller import Robot, Camera, CameraRecognitionObject, Keyboard



MAX_SPEED = 6.28  # [rad/s]

# Initialize the robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Initialize camera
camera = robot.getDevice('camera-1')
camera.enable(timestep)
camera.recognitionEnable(timestep)
camera.hasRecognition()

#Initialize gps
gps = robot.getDevice('gps')
gps.enable(timestep)

#Initialize compass
compass = robot.getDevice('compass')
compass.enable(timestep)

# Initialize motors
motor_left = robot.getDevice('left wheel motor')
motor_right = robot.getDevice('right wheel motor')
motor_left.setPosition(float('inf'))
motor_right.setPosition(float('inf'))
motor_left.setVelocity(0)
motor_right.setVelocity(0)


keyboard = robot.getKeyboard()
keyboard.enable(timestep)

vL = 0
vR = 0

# Main control loop
while robot.step(timestep) != -1:
    if(len(camera.getRecognitionObjects()) >= 1):
        for val, i in enumerate(camera.getRecognitionObjects()):
        
            Object = i
            id = Object.get_id()
            position = Object.get_position()
            orientation = Object.get_orientation()
            model = Object.get_model() 
            
            #construct our rotation matrix
            alpha = orientation[2]
            beta = orientation[1]
            gamma = orientation[0]
            
            r = np.array([
            [np.cos(alpha)*np.cos(gamma)*np.cos(beta) - np.sin(gamma)*np.sin(alpha), -np.cos(gamma)*np.cos(beta)*np.sin(alpha)-np.sin(gamma)*np.cos(alpha), np.cos(gamma)*np.sin(beta)],
            [np.sin(gamma)*np.cos(beta)*np.cos(alpha), -np.cos(gamma)*np.cos(beta)*np.sin(alpha)+np.sin(gamma)*np.cos(alpha), np.sin(gamma)*np.sin(beta)],
            [-np.sin(beta)*np.cos(alpha), np.sin(beta)*np.sin(alpha), np.cos(beta)]])
            #print(r, '\n\n')
            positions = compass.getValues()
            print(positions)
            print(np.matmul(r,positions[:3]))
            #print(np.matmul(r,positions))
      
            # print(val)
            # print(Object)
            # print(id)
            # print(position)
            print(model)
            # print(orientation)
            
    # print(gps.getValues())
    # print(compass.getValues())
    key = keyboard.getKey()
    if key == keyboard.LEFT :
        vL = -MAX_SPEED
        vR = MAX_SPEED
    elif key == keyboard.RIGHT:
        vL = MAX_SPEED
        vR = -MAX_SPEED
    elif key == keyboard.UP:
        vL = MAX_SPEED
        vR = MAX_SPEED
    elif key == keyboard.DOWN:
        vL = -MAX_SPEED
        vR = -MAX_SPEED
    elif key == ord(' '):
        vL = 0
        vR = 0
    else: # slow down
            vL *= 0.75
            vR *= 0.75
    motor_left.setVelocity(vL)
    motor_right.setVelocity(vR)
    