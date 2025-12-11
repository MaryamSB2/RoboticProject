from controller import Robot, DistanceSensor, Motor, GPS, InertialUnit, Speaker
import math

robot = Robot()
time_step = int(robot.getBasicTimeStep())

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

RIGHT_WHEELS = [wheels[0], wheels[3]]  
LEFT_WHEELS  = [wheels[1], wheels[2]]  
MAX_SPEED = wheels[0].getMaxVelocity()

GOAL_X = -4.91442  
GOAL_Y = -3.14033     
GOAL_TOL = 0.50       

obstacle_output = False

def step():
    return robot.step(time_step) != -1

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

while step():
    dsValues = [ds[i].getValue() for i in range(5)]
    pos = gps.getValues()      
    
    robot_x = pos[0]
    robot_y = pos[1]

    angle = imu.getRollPitchYaw()
    robot_angle = angle[2]         

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
                
                
    dx = GOAL_X - robot_x
    dy = GOAL_Y - robot_y
    
    distance = math.hypot(dx, dy)
    goal_angle = math.atan2(dy, dx)
    
    heading_error = normalize_angle(goal_angle - robot_angle)

    ang_gain = 3.0
    lin_gain = 6.0

    if distance < GOAL_TOL:
        set_wheel_speeds(0.0, 0.0)
        print("Goal reached")
        speaker.speak("Goal reached", 1.0)
        break

    v = lin_gain * distance             
    v = min(v, 0.4 * MAX_SPEED)         

    t = ang_gain * heading_error          
    
    left_speed  = v - t
    right_speed = v + t

    max_abs = max(abs(left_speed), abs(right_speed), MAX_SPEED)
    if max_abs > MAX_SPEED:
        left_speed  = left_speed  * MAX_SPEED / max_abs
        right_speed = right_speed * MAX_SPEED / max_abs

 
    if close_obstacle:
        set_wheel_speeds(-0.5 * MAX_SPEED, 0.5 * MAX_SPEED)
        if not obstacle_output:
            print("Obstacle Detected")
            speaker.speak("obstacle detected", 1.0)
            obstacle_output = True
    elif mid_obstacle:
        set_wheel_speeds(0.5 * left_speed, 0.5 * right_speed)
    elif far_obstacle:
        set_wheel_speeds(left_speed, right_speed)
    else:
        set_wheel_speeds(left_speed, right_speed)
        obstacle_output = False
