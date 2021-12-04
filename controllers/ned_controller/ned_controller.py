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

#JOINT MOTORS
joints = [robot.getDevice('joint_1'), robot.getDevice('joint_2'),
            robot.getDevice('joint_3'), robot.getDevice('joint_4'),
            robot.getDevice('joint_5'), robot.getDevice('joint_6'),
            robot.getDevice('joint_base_to_jaw_1'), robot.getDevice('joint_base_to_jaw_2')]

#DISTANCE SENSORS


#JOINT POSITION SENSORS
sensors = [robot.getDevice('joint_1_sensor'), robot.getDevice('joint_2_sensor'),
            robot.getDevice('joint_3_sensor'), robot.getDevice('joint_4_sensor'),
            robot.getDevice('joint_5_sensor'), robot.getDevice('joint_6_sensor'),
            robot.getDevice('joint_base_to_jaw_1_sensor'), robot.getDevice('joint_base_to_jaw_2_sensor')]
ps = [0, 0, 0, 0, 0, 0, 0, 0]

for s in sensors:
    s.enable(timestep)



target_pos = ('inf', 'inf', 'inf', 'inf', 'inf', 'inf', 'inf')


joint_ranges = [(-2.8,2.8),(-.8,.8),(-1.5,1.3),(-2,2),(-1.5,1.5),(-2.5,2.5), (-.01,.01), (-.01, .01)]

# Allow sensors to properly initialize   
for i in range(10): robot.step(timestep)  

#for i in range(8):
#    joints[i].setPosition(0)

length1 = 0.2213
length2 = 0.232
length3 = 0.1195

offset1 = math.pi-1.5166
offset2 = 1.37915
offset3 = -.014628

#upright_position = [0,-.5995,2.23-math.pi/2,-1.2146,0,0,0] #upright position[3] is to make the wrist position s.t. gripper lines up with the robot

delta = .05 #the change in position for every timestep when a key is pressed

state = 'control' 

def erect(): #fully erect robot
    joints[2].setPosition(joint_ranges[2][0])
    
#part (int) 1 = joint2, 2=joint3, 3=joint5
def get_angle(part):
    if part == 1:
        return offset1 + ps[1]
    elif part == 2:
        return offset2 + ps[2]
    else:
        return offset3 + ps[4]

def get_position(part, rad): #get joint position to be at angle. returns -999 if not possible
    if part == 1:
        pos = rad - offset1
        if pos >= joint_ranges[1][0] and pos <= joint_ranges[1][1]:
            return pos
        return -999
    elif part == 2:
        pos =  rad - offset2
        if pos >= joint_ranges[2][0] and pos <= joint_ranges[2][1]:
            return pos
        return -999
    elif part == 3:
        pos = offset3 - rad
        if pos >= joint_ranges[4][0] and pos <= joint_ranges[4][1]:
            return pos
        return -999

def gripper_position(world=False):
    theta1 = get_angle(1)
    theta2 = get_angle(2)
    theta3 = get_angle(3)
    x = length1*math.cos(theta1) + length2*math.cos(theta1+theta2) + length3*math.cos(theta1+theta2+theta3)
    y = length1*math.sin(theta1) + length2*math.sin(theta1+theta2) + length3*math.sin(theta1+theta2+theta3) + .17
    if world:
        x+=5.21
        y+=.74
        x*= -1
    return x,y 
    
joints[1].setPosition(get_position(1, 3*math.pi/4))
joints[2].setPosition(get_position(2, 0))
 
delta = .1

while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    for i in range(8):
        ps[i] = sensors[i].getValue()
    
    #IK START

    #IK END

    #STATES

    #KEYBOARD CONTROLS
    key = keyboard.getKey()
    if key == ord("S"):
        joints[2].setPosition(ps[2]+delta)
    elif key == ord("W"):
        joints[2].setPosition(ps[2]-delta)
    if key == keyboard.DOWN:
        joints[1].setPosition(ps[1]+delta)
    elif key == keyboard.UP:
        joints[1].setPosition(ps[1]-delta)
    elif key == ord("P"):
        #print("ps:",ps)
        print("gripper y,z in world:", gripper_position(True), "theta1:",get_angle(1), "theta2:",get_angle(2), "theta3",get_angle(3))
    