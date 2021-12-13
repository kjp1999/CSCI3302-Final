"""arm_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, DistanceSensor, Keyboard, TouchSensor, PositionSensor
import math
import numpy as np
from matplotlib import pyplot as plt

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# We are using a keyboard to remote control the robot
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

rx = 0
ry = 0
rz = 0

#JOINT MOTORS
joints = [robot.getDevice('joint_1'), robot.getDevice('joint_2'),
            robot.getDevice('joint_3'), robot.getDevice('joint_4'),
            robot.getDevice('joint_5'), robot.getDevice('joint_6'),
            robot.getDevice('joint_base_to_jaw_1'), robot.getDevice('joint_base_to_jaw_2')]

joint_ranges = [(-2.8,2.8),(-.8,.8),(-1.5,1.3),(-2,2),(-1.5,1.5),(-2.5,2.5), (-.01,.01), (-.01, .01)]

#JOINT POSITION SENSORS
sensors = [robot.getDevice('joint_1_sensor'), robot.getDevice('joint_2_sensor'),
            robot.getDevice('joint_3_sensor'), robot.getDevice('joint_4_sensor'),
            robot.getDevice('joint_5_sensor'), robot.getDevice('joint_6_sensor'),
            robot.getDevice('joint_base_to_jaw_1_sensor'), robot.getDevice('joint_base_to_jaw_2_sensor')]
ps = [0, 0, 0, 0, 0, 0, 0, 0]

for s in sensors:
    s.enable(timestep)

target_pos = ('inf', 'inf', 'inf', 'inf', 'inf', 'inf', 'inf')

# Allow sensors to properly initialize   
for i in range(10): robot.step(timestep)  

length1 = 0.2213
length2 = 0.232
length3 = 0.1195

offset1 = math.pi-1.5166
offset2 = 1.37915
offset3 = -.014628

#reference joints using these variables
j1 = 1
j2 = 2
j3 = 4

upright_position = [0,-.5995,2.23-math.pi/2,-1.2146,0,0,0] #upright position[3] is to make the wrist position s.t. gripper lines up with the robot

state = 'control' 

def erect(): #fully erect robot
    joints[2].setPosition(joint_ranges[2][0])

#part (int) 1 = joint2, 2=joint3, 3=joint5 
#pass in the part number, optional position (grabs current if not passed in), returns the conventional angle 
def get_angle(part, pos = None):
    if pos is None:
        if part == 1:
            return offset1 + ps[1]
        elif part == 2:
            return offset2 + ps[2]
        else:
            return -(offset3 + ps[4])
    else:
        if part == 1:
            return offset1 + pos
        elif part == 2:
            return offset2 + pos
        else:
            return -(offset3 + pos)

#pass in the part number and the conventional angle you want, returns the position (-999 if not possible)
def get_position(part, rad): #get joint position to be at angle. returns -999 if not possible
    if part == 1:
        pos = rad - offset1
        if pos >= joint_ranges[1][0] and pos <= joint_ranges[1][1]:
            return pos
        return -919
    elif part == 2:
        pos =  rad - offset2
        if pos >= joint_ranges[2][0] and pos <= joint_ranges[2][1]:
            return pos
        return -929
    elif part == 3:
        pos = rad - offset3
        if pos >= joint_ranges[4][0] and pos <= joint_ranges[4][1]:
            return pos
        return -939

#returns the end effectors location. If base isn't rotated, pass in True to get world coordinates. 
def gripper_position(world=False):
    if ps[0] != 0 and world == True:
        if ps[0] < .001:
            joints[0].setPosition(0)
            ps[0] = 0
        else:
            print("Base is rotated, world coordinates will not be used")
            world = False
    theta1 = get_angle(1)
    theta2 = get_angle(2)
    theta3 = get_angle(3)
    x = length1*math.cos(theta1) + length2*math.cos(theta1+theta2) + length3*math.cos(theta1+theta2+theta3)
    y = length1*math.sin(theta1) + length2*math.sin(theta1+theta2) + length3*math.sin(theta1+theta2+theta3) + .17
    if world:
        x+= -ry
        y+= rz
        x*= -1
    return x,y 
 
def adjust_x(direction = False): #adjust x: False -> left, True -> right
    goal_pos[0] += direction*.0005 + (not direction)*-.0005
    set_pos(goal_pos)
    
def adjust_y(direction = False): #adjust y: False -> left, True -> right
    goal_pos[1] += direction*.005 + (not direction)*-.005
    set_pos(goal_pos)
    
delta = .05 #the change in position for every timestep when a key is pressed

configmap = np.full((98, 80,3), (6.0, 6.0, 6.0)) #the map of configurations based on location in the matrix
map = np.zeros((98,80)) #used to map which spaces the robot can reach

def getconfig():   #initializes the configmap
    configmap = np.full((98, 80,3), (6.0, 6.0, 6.0))
    maxx = -1
    maxy = -1
    minx = 1
    miny = 1
    for t1 in np.linspace(-.8,.8,100):
        for t2 in np.linspace(-1.5,1.3,100):
            for t3 in np.linspace(-1.5,1.5,100):
                theta1 = offset1 + t1
                theta2 = offset2 + t2
                theta3 = -(offset3 + t3)
                x = length1*math.cos(theta1) + length2*math.cos(theta1+theta2) + length3*math.cos(theta1+theta2+theta3)
                y = length1*math.sin(theta1) + length2*math.sin(theta1+theta2) + length3*math.sin(theta1+theta2+theta3) + .17
                x = round(x,2)
                y = round(y,2)
                xm = int(x*100 + 52)
                ym = int(y*100 + 4)
                if configmap[xm][ym][0] == 6.0 and y>=0: #y>.01
                   maxx = x if x > maxx else maxx
                   maxy = y if y > maxy else maxy
                   minx = x if x < minx else minx
                   miny = y if y < miny else miny
             
                   configmap[xm][ym] = [t1,t2,t3]  
                   map[xm][ym] = 1 
    np.save('configmap', configmap)
    print("x",minx,maxx,"y",miny,maxy)
#getconfig()
configmap = np.load('configmap.npy')


def get_object_pos(x,y,z): #x,y,z in world reference frame -> x,y,z in robots reference frame
    px = x - rx
    py = y - ry
    pz = z - rz
    bearing = math.atan(-px/py)
    x_distance = math.sqrt(px**2 + py**2)
    if y < 0 and x > 0: 
        bearing = bearing - math.pi
    elif y < 0 and x < 0:
        bearing = bearing + math.pi
    elif x == 0 and y < 0:
        bearing = bearing - math.pi
    return [-x_distance, pz, bearing]
    
def set_pos(p, B=0, world=False):   #pass in x,y in the arm's reference frame to set its position. cannot use world coordinates if base is rotated
    if p is None:
        joints[0].setPosition(B)
        return
    if len(goal_pos) == 3:
        B = goal_pos[2]
    x = p[0]
    y = p[1]
    if world:
        x+=-ry
        y-=rz
        x*=-1
        x=round(x,2)
        y=round(y,2)
    xm = int(x*100 + 52)
    ym = int(y*100 + 4)
    if xm >= 0 and xm<98 and ym >= 0 and ym <80 and configmap[xm][ym][0] != 6:
        a = configmap[xm][ym]
        joints[1].setPosition(a[0])
        joints[2].setPosition(a[1])
        joints[4].setPosition(a[2])
        joints[0].setPosition(B)
        return configmap[xm][ym]
    else:
            print("cannot move robot to passed coordinates.")
            return [6,6,6]

objects = None
oi = 0
path = []
current = 0
goal_pos = []  #pingpong -.31, .02, can -0.42, .03
throw_state = -2
pickup_state = 0
prev_t = 0
joints[6].setPosition(.01)
joints[7].setPosition(.01)

while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    for i in range(8):
        ps[i] = sensors[i].getValue()
    if state == 'path':
        if current == len(path):
            print("path finished")
            path = []
            state = 'control'
            current = 0
        else:
            x,y = path[current][0], path[current][1]
            set_pos(x,y)
            current+=1
    #STATES
    if state == 'control': #KEYBOARD CONTROLS
        key = keyboard.getKey()
        if key == ord("S"):
            joints[2].setPosition(ps[2]+delta)
        elif key == ord("O"):
            joints[6].setPosition(ps[6]+.005)
            joints[7].setPosition(ps[7]+.005)
        elif key == ord("C"):
            joints[6].setPosition(ps[6]-.0005)
            joints[7].setPosition(ps[7]-.0005)
        elif key == ord("W"):
            joints[2].setPosition(ps[2]-delta)
        elif key == keyboard.DOWN:
            joints[1].setPosition(ps[1]+delta)
        elif key == keyboard.UP:
            joints[1].setPosition(ps[1]-delta)
        elif key == ord("A"):
            joints[0].setPosition(ps[0]+delta/4)
        elif key == ord("D"):
            joints[0].setPosition(ps[0]-delta/4)
        elif key == ord("Q"):
            joints[4].setPosition(ps[4]+delta)
        elif key == ord("Z"):
            joints[4].setPosition(ps[4]-delta)
        elif key == ord("N"):
            configmap = np.load('configmap.npy')
            print("config map loaded.")
        elif key == ord("H"):
             objects = np.load('../epuck_controller/positions.npy')
             print("object positions loaded")
        elif key == ord("Y"):
             state = 'throwaway'
             print("throwing away!")
        elif key == ord("T"):
            if objects is None:
                print("No object coordinates loaded")
            elif oi >= len(objects):
                print('finish list')
                oi = 0
                objects = None
               
            else:
                c = objects[oi]
                print("Current object:",c[0])
                
                goal_pos = get_object_pos(float(c[1]),float(c[2]),.02+rz)
                state = 'pickup'
        elif key == ord("1"):
            adjust_x(False)
        elif key == ord("2"):
            adjust_x(True)
        elif key == ord("3"):
            adjust_y(False)
        elif key == ord("4"):
            adjust_y(True)
        elif key == ord("M"):
            getconfig()
            plt.imshow(np.flip(map.T, axis=0))
            plt.show()
        elif key == ord("P"):
            #print("ps:",ps)
            #print("joint positions:", ps[1], ps[2], ps[4])
            print("gripper y,z:", gripper_position())
        
    elif state == 'pickup':
        if pickup_state == 0:
            print("picking up")
            #open gripper
            joints[6].setPosition(.01)
            joints[7].setPosition(.01)
            set_pos(None, goal_pos[2])
            prev_t = robot.getTime()
            pickup_state = 1
        elif pickup_state == 1:
            if robot.getTime() > prev_t + 1:
                pickup_state = 2
        elif pickup_state == 2:
            set_pos(goal_pos[0:2])
            prev_t = robot.getTime()
            pickup_state = 3
        elif pickup_state == 3:
            if robot.getTime() > prev_t + 1:
                adjust_y(False)
                #close gripper
                joints[6].setPosition(0)
                joints[7].setPosition(0)
                print("done.")
                state = 'control'
                pickup_state = 0
                oi += 1
    elif state == 'throwaway':
        if throw_state == -2:
            #print("-2")
            joints[1].setPosition(0)
            joints[2].setPosition(0)
            joints[4].setPosition(0)
            prev_t = robot.getTime()
            throw_state = -1
        elif throw_state == -1:
            #print("-1")
            if robot.getTime() > prev_t + 1:
                throw_state = 0
        if throw_state == 0:
            sign = -1 if ps[0] > 0 else 1 
            joints[0].setPosition(ps[0]+sign*.05)
            if abs(ps[0]) < .1:
                prev_t = robot.getTime()
                throw_state = .5
        elif throw_state == .5:
            if robot.getTime() > prev_t + 1:
                throw_state = 1
        elif throw_state == 1:
            if robot.getTime() > prev_t + 1:
                throw_state = 2
        elif throw_state == 2:
            joints[1].setPosition(ps[1]-.5)
            joints[2].setPosition(ps[2]-.5)
            if ps[1] < -0.4728319402921535 and ps[2] < -0.47306292600702626:
                joints[6].setPosition(.01)
                joints[7].setPosition(.01)
                throw_state = 3
        elif throw_state == 3:
            prev_t = robot.getTime()
            throw_state = 4
        elif throw_state == 4:
            if robot.getTime() > prev_t + 1:
                for i in range(6):
                    joints[i].setPosition(0)
                print("done!")
                throw_state = -2
                state = 'control'