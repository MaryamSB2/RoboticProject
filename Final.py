#Import distance sensors, motor, GPS, inertial unit and speaker from the controller module
from controller import Robot, DistanceSensor, Motor, GPS, InertialUnit, Speaker
import math

#Create robot instance and get world time step
robot = Robot()
time_step = int(robot.getBasicTimeStep())

#Get device instances
speaker = robot.getDevice("speaker")

ds = []
dsNames = ['ds_front', 'ds_right', 'ds_left', 'ds_right_side', 'ds_left_side'] 
for name in dsNames:
    sensor = robot.getDevice(name)
    sensor.enable(time_step)
    ds.append(sensor)

gps = robot.getDevice("gps")
gps.enable(time_step)

imu = robot.getDevice("inertial unit")
imu.enable(time_step)

wheels = []
wheelNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
for name in wheelNames:
    motor = robot.getDevice(name)
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)
    wheels.append(motor)

#Assign right and left wheels, and get the max velocity
RIGHT_WHEELS = [wheels[0], wheels[3]]  
LEFT_WHEELS  = [wheels[1], wheels[2]]  
MAX_SPEED = wheels[0].getMaxVelocity()

#Explicitly set the goal coordinates
GOAL_X = -4.91442  
GOAL_Y = -3.14033     
GOAL_TOL = 0.50 #set the tolerance to reach the goal within

#Boolean to ensure the verbal output only occurs once per obstacle/goal reached rather than every step
obstacle_output = False

def step():
    return robot.step(time_step) != -1

#Define function to set the speeds of the left and right wheels to an inputted value
def set_wheel_speeds(v_left, v_right):
    for m in LEFT_WHEELS:
        m.setVelocity(v_left)
    for m in RIGHT_WHEELS:
        m.setVelocity(v_right)

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

#Main loop
while step():
    dsValues = [ds[i].getValue() for i in range(5)] #read all distance sensor values
    pos = gps.getValues() #read GPS position

    #Set robot x and y positions from the GPS coordinates
    robot_x = pos[0]
    robot_y = pos[1]

    angle = imu.getRollPitchYaw() #read orientation from the IMU
    robot_angle = angle[2]  #set robot angle from IMU orientation 

    #classifying obstacle type depending on distance sensor values
    close_obstacle = (dsValues[0] < 350.0 or
                      dsValues[1] < 350.0 or
                      dsValues[2] < 350.0 or 
                      dsValues[3] < 350.0 or
                      dsValues[4] < 350.0)

    mid_obstacle = (dsValues[0] < 500.0 or
                    dsValues[1] < 500.0 or
                    dsValues[2] < 500.0 or
                    dsValues[3] < 500.0 or
                    dsValues[4] < 500.0) 
                    

    far_obstacle = (dsValues[0] < 950.0 or
                    dsValues[1] < 950.0 or
                    dsValues[2] < 950.0 or
                    dsValues[3] < 950.0 or
                    dsValues[4] < 950.0)
                
    #calculate distance to goal            
    dx = GOAL_X - robot_x
    dy = GOAL_Y - robot_y
    distance = math.hypot(dx, dy)

    #calculate desired angle
    goal_angle = math.atan2(dy, dx)
    heading_error = normalize_angle(goal_angle - robot_angle)

    ang_gain = 3.0
    lin_gain = 6.0

    #If Spot reaches the goal within the tolerance
    if distance < GOAL_TOL:
        set_wheel_speeds(0.0, 0.0) #halt Spot
        print("Goal reached") #output to console
        speaker.speak("Goal reached", 1.0) #output announcement that the goal is reached to the user via the speaker
        break #exit loop

    #Calculate motion commands
    v = lin_gain * distance             
    v = min(v, 0.4 * MAX_SPEED)         

    t = ang_gain * heading_error          
    
    left_speed  = v - t
    right_speed = v + t

    #limit the speeds to the maximum allowed speeds
    max_abs = max(abs(left_speed), abs(right_speed), MAX_SPEED)
    if max_abs > MAX_SPEED:
        left_speed  = left_speed  * MAX_SPEED / max_abs
        right_speed = right_speed * MAX_SPEED / max_abs

    #Obstacle detection
    if close_obstacle: #if obstacle < 350
        set_wheel_speeds(-0.5 * MAX_SPEED, 0.5 * MAX_SPEED) #perform emergency avoidance
        if not obstacle_output: #if the output hasn't already been given
            #output message to the user via console and speaker
            print("Obstacle Detected")
            speaker.speak("obstacle detected", 1.0)
            obstacle_output = True #prevent message outputting each step
    elif mid_obstacle: #if obstacle < 500 and >= 350
        set_wheel_speeds(0.5 * left_speed, 0.5 * right_speed) #slow the speeds by half
    elif far_obstacle: #if obstalce < 950 and >=500
        set_wheel_speeds(left_speed, right_speed) #proceed as normal
    else: #no obstacles within range
        set_wheel_speeds(left_speed, right_speed) #proceed as normal
        obstacle_output = False #allow for outputs to be given if an obstacle is detected
