"""object_detection controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, DistanceSensor, Keyboard, Motor
import math
import sys
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
#timestep = int(robot.getBasicTimeStep())
time_step = int(robot.getBasicTimeStep())

#output_given = FALSE

#getting the device instances
#3 distance sensors
ds = [] #array for the 3 sensors
dsNames = ['ds_front',
           'ds_right',
           'ds_left']
for i in range(3):
    ds.append(robot.getDevice(dsNames[i]))
    ds[i].enable(time_step)
    
wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
for i in range(4):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)
   
# ds = robot.getDevice('distance sensor')
# ds.enable(timestep)

#speaker = robot.getDevice('speaker')


#maryam code
NUMBER_OF_JOINTS = 12

motor_names = [
    "front left shoulder abduction motor",  "front left shoulder rotation motor",  "front left elbow motor",
    "front right shoulder abduction motor", "front right shoulder rotation motor", "front right elbow motor",
    "rear left shoulder abduction motor",   "rear left shoulder rotation motor",   "rear left elbow motor",
    "rear right shoulder abduction motor",  "rear right shoulder rotation motor",  "rear right elbow motor"
]

FL_ABD, FL_ROT, FL_ELB = 0, 1, 2
FR_ABD, FR_ROT, FR_ELB = 3, 4, 5
RL_ABD, RL_ROT, RL_ELB = 6, 7, 8
RR_ABD, RR_ROT, RR_ELB = 9, 10, 11

#marking the matrix position


keyboard = robot.getKeyboard()
keyboard.enable(time_step)

motors = []
for i, name in enumerate(motor_names):
    m = robot.getDevice(name)
    m.setPosition(0.0)
    m.setVelocity(m.getMaxVelocity()) #max veloity = 10.0 (i printed it lol)
    motors.append(m)

# Neutral standing posture
STAND_POSTURE = [
    -0.1, 0.0, 0.0,   # FL: abd, rot, elbow
     0.1, 0.0, 0.0,   # FR
    -0.1, 0.0, 0.0,   # RL
     0.1, 0.0, 0.0    # RR
]
#setting position for each joint when idle


def apply_posture(posture):
    for i in range(NUMBER_OF_JOINTS):
        motors[i].setPosition(posture[i])
        #for each jpint set it the associated number in stand_posture


def step():
    if robot.step(time_step) == -1:
        return False
    return True
    #getting the timestep


def set_gait(mode, t): 
    
    #assigning standing matrix to pos
    pos = STAND_POSTURE[:]

    if mode == "idle":
        apply_posture(pos)
        return
        #if idle = set to the standing matrix again
    
    freq = 2.0  
    phase = 2.0 * math.pi * freq * t #phase = 

    
    A_shoulder = 0.35  
    A_elbow = 0.45
    elbow_offset = 0.5

    
    def leg_pattern(is_front_left, is_front_right, is_rear_left, is_rear_right,
                    base_phase, direction_scale, turning_scale=0.0):
        
        ph = phase + base_phase

        
        shoulder = direction_scale * A_shoulder * math.sin(ph)

        
        elbow = elbow_offset + A_elbow * math.sin(ph + math.pi / 2.0)

        abd_base_left = -0.12
        abd_base_right = 0.12

        if is_front_left:
            pos[FL_ABD] = abd_base_left + turning_scale
            pos[FL_ROT] = shoulder
            pos[FL_ELB] = elbow
        if is_front_right:
            pos[FR_ABD] = abd_base_right - turning_scale
            pos[FR_ROT] = shoulder
            pos[FR_ELB] = elbow
        if is_rear_left:
            pos[RL_ABD] = abd_base_left + turning_scale
            pos[RL_ROT] = shoulder
            pos[RL_ELB] = elbow
        if is_rear_right:
            pos[RR_ABD] = abd_base_right - turning_scale
            pos[RR_ROT] = shoulder
            pos[RR_ELB] = elbow

    

    if mode == "forward":
        leg_pattern(True, False, False, True, base_phase=0.0, direction_scale=+1)
        leg_pattern(False, True, True, False, base_phase=math.pi, direction_scale=+1)

    elif mode == "backward":
        leg_pattern(True, False, False, True, base_phase=0.0, direction_scale=-1)
        leg_pattern(False, True, True, False, base_phase=math.pi, direction_scale=-1)

    elif mode == "left":
        leg_pattern(True, False, True, False, base_phase=0.0, direction_scale=-1, turning_scale=+0.05)
        leg_pattern(False, True, False, True, base_phase=math.pi, direction_scale=+1, turning_scale=+0.05)

    elif mode == "right":
        leg_pattern(True, False, True, False, base_phase=0.0, direction_scale=+1, turning_scale=-0.05)
        leg_pattern(False, True, False, True, base_phase=math.pi, direction_scale=-1, turning_scale=-0.05)

    apply_posture(pos)


print("Arrow keys to move Spot:")
print("  UP    = forward")
print("  DOWN  = backward")
print("  LEFT  = turn left")
print("  RIGHT = turn right")
print("  SPACE = stop / stand")

current_mode = "idle"
apply_posture(STAND_POSTURE)

while step():
    key = keyboard.getKey()
    while key != -1:
        if key == Keyboard.UP:
            current_mode = "forward"
        elif key == Keyboard.DOWN:
            current_mode = "backward"
        elif key == Keyboard.LEFT:
            current_mode = "left"
        elif key == Keyboard.RIGHT:
            current_mode = "right"
        elif key == ord(' '):
            current_mode = "idle"
        key = keyboard.getKey()

    t = robot.getTime()
    set_gait(current_mode, t)
    
    speed = 3.0
    for j in range(4):
        wheels[j].setVelocity(speed)
    
    dsValues = []
    for i in range(3):
        dsValues.append(ds[i].getValue()) 
        #note = lookup table changed so x = 1 rather than 0.1 to increase distance    
    
    if dsValues[0] < 250.0 or dsValues[1] < 250.0 or dsValues[2] <250.0:
        print("Immediate Obstacle Detected")
        for i in range(NUMBER_OF_JOINTS):
            motors[i].setVelocity(0.0) 
    elif dsValues[0] < 950.0 or dsValues[1] < 950.0 or dsValues[2] < 950.0:    
       for i in range(NUMBER_OF_JOINTS):
            motors[i].setVelocity(1.5) 
    else:
        for i in range(NUMBER_OF_JOINTS):
            motors[i].setVelocity(motors[i].getMaxVelocity()) 
        


#Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(time_step) != -1:
    pass

# Enter here exit cleanup code.
