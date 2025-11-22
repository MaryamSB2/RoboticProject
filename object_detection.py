"""object_detection controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, DistanceSensor, Keyboard, Motor
import math
import sys
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
time_step = int(robot.getBasicTimeStep())

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
   
obstacle_output = False

def step():
    if robot.step(time_step) == -1:
        return False
    return True
    #getting the timestep

while step():
    
    dsValues = []
    for i in range(3):
        dsValues.append(ds[i].getValue()) 
        #note = lookup table changed so x = 1 rather than 0.1 to increase distance    
        
    if dsValues[0] < 350.0 or dsValues[1] < 350.0 or dsValues[2] <350.0:
        for i in range(4):
            wheels[i].setVelocity(0.0) 
        if obstacle_output == False:
            print("Immediate Obstacle Detected")
            obstacle_output = True
    elif dsValues[0] < 500.0 or dsValues[1] < 500.0 or dsValues[2] < 500.0:    
       for i in range(4):
            wheels[i].setVelocity(2) 
    elif dsValues[0] < 950.0 or dsValues[1] < 950.0 or dsValues[2] < 950.0:    
       for i in range(4):
            wheels[i].setVelocity(5) 
    else:
        for i in range(4):
            wheels[i].setVelocity(wheels[i].getMaxVelocity()) 
        obstacle_output = False
        


#Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(time_step) != -1:
    pass

# Enter here exit cleanup code.
