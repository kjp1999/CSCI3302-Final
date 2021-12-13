"""arm_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, DistanceSensor, Keyboard, TouchSensor, PositionSensor
import math

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# We are using a keyboard to remote control the robot
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

dsr = [0, 0, 0, 0]
distance_sensors = [robot.getDevice('ts0'), robot.getDevice('ts1'), robot.getDevice('ts2'), robot.getDevice('ts3')]



for ds in distance_sensors:
    ds.enable(timestep)
    
psr = [0,0,0,0,0,0,0]
psr_names = ["base_sensor", "upperarm_sensor", "forearm_sensor", "wrist_sensor", "rotational_wrist_sensor", 
"left_gripper_sensor", "right_gripper_sensor"]
position_sensors = [robot.getDevice(name) for name in psr_names]

for ps in position_sensors:
    ps.enable(timestep)

BASE = 0
UPPERARM = 1
FOREARM = 2
WRIST = 3
ROTATIONAL_WRIST = 4
L_GRIPPER = 5
R_GRIPPER = 6

part_names = ["base", "upperarm", "forearm", "wrist", "rotational_wrist", "left_gripper", "right_gripper"]

target_pos = ('inf', 'inf', 'inf', 'inf', 'inf', 'inf', 'inf')
robot_parts = []

motor_ranges = [(0,6.03),(-2.44,0),(0,4.21),(-4.0491,0),(-5.79789,0),(-1.22,0),(0,1.22)]

for i in range(len(part_names)):
    robot_parts.append(robot.getDevice(part_names[i]))

# Allow sensors to properly initialize   
for i in range(10): robot.step(timestep)  

for part in robot_parts:
    part.setPosition(0)

l = [.2, .19, .14, .14, 0,  .19] # l[6]=.2m is the length from joint 3 to the gripper

upright_position = [0,-.5995,2.23-math.pi/2,-1.2146,0,0,0] #upright position[3] is to make the wrist position s.t. gripper lines up with the robot

def get_part_position(part, rad=None): #rad is None: position -> true radian angle, rad not None: true radian -> position    
    #rad is None, f: motor position -> angle, used for equations requiring the angle of the joint
    if rad is None:
        if part == UPPERARM: 
            return math.pi/2 + psr[UPPERARM] - upright_position[UPPERARM]
        else:
            return psr[part] - upright_position[part]
            
    #given an angle of the joint, find correct position
    else:
        if part == UPPERARM:
            return upright_position[part] -math.pi/2 + rad 
        else:
            return upright_position[part] + rad 
def gripper_position():
    theta1 = get_part_position(1)
    theta2 = get_part_position(2)
    theta3 = get_part_position(3)
    x = l[1]*math.cos(theta1) + l[2]*math.cos(theta1+theta2) + l[3]*math.cos(theta1+theta2+theta3)
    y = .2 + l[1]*math.sin(theta1) + l[2]*math.sin(theta1+theta2) + l[3]*math.sin(theta1+theta2+theta3)
    return x,y - .07 #y has +.07 error, x is about +-.1 off
# Main loop:
# - perform simulation steps until Webots is stopping the controller
once = True

delta = .05 #the change in position for every timestep when a key is pressed

state = 'control' 
x1 = 0 

while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    for i, ds in enumerate(distance_sensors):
        dsr[i] = ds.getValue()
        
    for i, ps in enumerate(position_sensors):
        psr[i] = ps.getValue()
    

    #COMPUTER VISION START
    
    #COMPUTER VISION END
    
    #IK START

    #IK END

    #STATES

    #KEYBOARD CONTROLS
    if state == 'control':
        key = keyboard.getKey()
        while(keyboard.getKey() != -1): pass
        if key == ord('A'):
            if psr[BASE]+delta < motor_ranges[BASE][1]:
                robot_parts[BASE].setPosition(psr[BASE]+delta)
        elif key == ord ("D"):
            if psr[BASE]-delta > motor_ranges[BASE][0]:
                robot_parts[BASE].setPosition(psr[BASE]-delta)
        elif key == ord("W"): 
            if psr[UPPERARM]-delta > motor_ranges[UPPERARM][0]:
                robot_parts[UPPERARM].setPosition(psr[UPPERARM]-delta)
        elif key == ord ("S"):
            if psr[UPPERARM]+delta < motor_ranges[UPPERARM][1]:
                robot_parts[UPPERARM].setPosition(psr[UPPERARM]+delta)
        elif key == keyboard.UP:
            if psr[FOREARM]+delta < motor_ranges[FOREARM][1]:
                robot_parts[FOREARM].setPosition(psr[FOREARM]+delta)
        elif key == ord("E"):
            if psr[WRIST]-delta > motor_ranges[WRIST][0]:
                robot_parts[WRIST].setPosition(psr[WRIST]-delta)
        elif key == ord ("C"):
            if psr[WRIST]+delta < motor_ranges[WRIST][1]:
                robot_parts[WRIST].setPosition(psr[WRIST]+delta)
        elif key == keyboard.DOWN:
            if psr[FOREARM]-delta > motor_ranges[FOREARM][0]:
                robot_parts[FOREARM].setPosition(psr[FOREARM]-delta)
        elif key == ord("U"):
            robot_parts[UPPERARM].setPosition(get_part_position(UPPERARM, math.pi/2))
            robot_parts[FOREARM].setPosition(get_part_position(FOREARM, math.pi/2))
            robot_parts[WRIST].setPosition(upright_position[WRIST])
        elif key == ord("O"):
            robot_parts[UPPERARM].setPosition(0)
            robot_parts[FOREARM].setPosition(0)
            robot_parts[WRIST].setPosition(0)
        
        elif key == ord("P"):
            print("UPPERARM:",get_part_position(UPPERARM),"FOREARM:",get_part_position(FOREARM),"WRIST:",get_part_position(WRIST))
            print("Gripper x,y:",gripper_position())
  

 