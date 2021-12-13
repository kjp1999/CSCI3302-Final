import numpy as np
import math
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
        objects = []
        for val, i in enumerate(camera.getRecognitionObjects()):
            
            Object = i
            id = Object.get_id()
            position = Object.get_position()
            orientation = Object.get_orientation()
            model = Object.get_model()
           
            #construct our rotation matrix
            # positions = compass.getValues()
            # alpha = positions[2]
            # beta = positions[1]
            # gamma = positions[0]
            
            # r = np.array([
            # [np.cos(alpha)*np.cos(gamma)*np.cos(beta) - np.sin(gamma)*np.sin(alpha), -np.cos(gamma)*np.cos(beta)*np.sin(alpha)-np.sin(gamma)*np.cos(alpha), np.cos(gamma)*np.sin(beta)],
            # [np.sin(gamma)*np.cos(beta)*np.cos(alpha), -np.cos(gamma)*np.cos(beta)*np.sin(alpha)+np.sin(gamma)*np.cos(alpha), np.sin(gamma)*np.sin(beta)],
            # [-np.sin(beta)*np.cos(alpha), np.sin(beta)*np.sin(alpha), np.cos(beta)]])
            # #print(r, '\n\n')
            # r = np.append(r, np.array([[0,0,0]]), axis=0)
        
            # position = np.append(np.array(position), 1)
        
            # r = np.c_[r, position]  
            #print(r)
            # gps_pos = np.array(gps.getValues())
            # gps_pos = np.append(gps_pos, 0)
            
            
            
            # print(np.matmul(r,gps_pos))
            #print(np.matmul(r,positions))
      
            # print(val)
            # print(Object)
            # print(id)
            # print(position)
            # print(model)
            # print(orientation)
            
    
    
            def getBearing():
                north = compass.getValues()
                rad = np.arctan2(north[0],north[2])
                bearing = (rad - 1.5708) / math.pi * 180.0
                if (bearing < 0.0):
                    bearing = bearing + 360.0
                    
                return math.radians(bearing)
            theta = 2*math.pi - getBearing()
            pos = gps.getValues()
        
            #print(compass.getValues())
            Y_world = (position[0] * np.cos(theta) + position[2] * np.sin(theta) + pos[1]) + (-.041)
            X_world = (position[2] * np.cos(theta) - position[0] * np.sin(theta) + pos[0])
            #print(X_world, Y_world)
            
            if(model != b'Ned'):
                objects.append([model, X_world,Y_world])
            #print(objects)
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
        
    elif key == ord('J'):
        np.save("positions", objects)
        print("Saving")
    else: # slow down
            vL *= 0.75
            vR *= 0.75
    motor_left.setVelocity(vL)
    motor_right.setVelocity(vR)
    