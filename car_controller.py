"""car_controller controller."""
#controller for the dynamic obstacles (yellow gates) used in testing. Moves the gates back and forth.
# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Get device instances------------------
#wheels
wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
for i in range(4):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)
#distance sensors
ds = []
dsNames = ['ds_left', 'ds_right']
for i in range(2):
    ds.append(robot.getDevice(dsNames[i]))
    ds[i].enable(timestep)

#set wheel speeds------------------------------
speed = 10.0
for i in range(4):
    wheels[i].setVelocity(speed)

# Main loop:
while robot.step(timestep) != -1:
    # Read the sensors:
    if ds[0].getValue() < 1000: #if obstacle detected by left sensor
        for i in range(4):
            wheels[i].setVelocity(-10) #reverse the direction of the wheels
    elif ds[1].getValue() < 1000: # if the obstacle is detected by the right sensor (away from me)
        for i in range(4):
            wheels[i].setVelocity(10) #make the wheels go forwaes at a speed of 10
