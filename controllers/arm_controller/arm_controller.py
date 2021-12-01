"""arm_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, DistanceSensor, Keyboard, TouchSensor, PositionSensor

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

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

for i in range(len(part_names)):
    robot_parts.append(robot.getDevice(part_names[i]))
    


# Allow sensors to properly initialize   
for i in range(10): robot.step(timestep)  

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    for i, ds in enumerate(distance_sensors):
        dsr[i] = ds.getValue()
        
    for i, ps in enumerate(position_sensors):
        psr[i] = ps.getValue()
    
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
#her