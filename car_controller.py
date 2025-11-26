"""car_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
for i in range(4):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)
    
ds = []
dsNames = ['ds_left', 'ds_right']
for i in range(2):
    ds.append(robot.getDevice(dsNames[i]))
    ds[i].enable(timestep)
    
speed = 10.0
for i in range(4):
    wheels[i].setVelocity(speed)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # #Enter here functions to send actuator commands, like:
     # #motor.setPosition(10.0)
    if ds[0].getValue() < 1000: #left sensor (towards me)
        for i in range(4):
            wheels[i].setVelocity(-10)
    elif ds[1].getValue() < 1000: # right sensor (away from me)
        for i in range(4):
            wheels[i].setVelocity(10)

# Enter here exit cleanup code.
