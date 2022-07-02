#! /usr/bin/env python
import math
import rospy
import numpy as np
from universal_pid import universal_pid
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive , AckermannDriveStamped

speed = 0.0 
steering_angle = 0.0
acceleration = 0.0
distance_set = []
current_gear = 0 #Notrunning
simulator_mode = False

wf_pid_controller = universal_pid(0.2, 0.3, 0.0, 0.4)
# speed_pid_controller = universal_pid(0.2)

def get_range(data, angle, deg=True):
    """
    this method outputs the distance to a certain angle

    Args:
        data: the LaserScan data outputed by Lidar
        angle: the angle in range (-pi, +pi) in radiens,
            0 is front and positive is to the left.
            For example, 90 will be directly to the left of the Lidar
        deg: (default: True) whether you input degree or radians
    Returns:
        the distance in meter
    """
    if deg:
        angle = np.deg2rad(angle)
    # on TianRacer, the front of the Lidar is the rear of the car
    if simulator_mode == False:
        angle = angle + np.pi
        if angle > np.pi:
            angle -= np.pi * 2
    dis = data.ranges[int((angle - data.angle_min) / data.angle_increment)]
    if simulator_mode == False:
        if dis < data.range_min:
            dis = 0.15
        if dis > data.range_max:
            dis = 12.0
    return dis


def callback(data):
    global speed
    global steering_angle
    global distance_set
    global acceleration

    distance_set = [get_range(data, i) for i in range(90, -91, -1)]
    steering_angle = 0.0
    acceleration = 0.0
    perception(data)

    if simulator_mode == True:
        drive_msg = AckermannDrive(steering_angle=steering_angle, speed=speed, acceleration = acceleration)
        drive_st_msg = AckermannDriveStamped(drive=drive_msg)
        drive_pub_simulator.publish(drive_st_msg)
    else:
        drive_msg = AckermannDrive(steering_angle=steering_angle, speed=speed, acceleration = acceleration)
        drive_pub.publish(drive_msg)

def perception(data):
    """
    run all the perception modules, and return a tuple including a boolean and a float value
        boolean: shows whether the perception modules is triggered and activated currently
        float: shows the change of steering angle need to be adjusted according to the perception

    Args:
        data: the LaserScan data outputed by Lidar
        angle: the angle in range (-pi, +pi) in radiens,
            0 is front and positive is to the left.
            For example, 90 will be directly to the left of the Lidar
        deg: (default: True) whether you input degree or radians
    Returns:
        the distance in meter
    """
    global speed
    global steering_angle
    global distance_set
    global acceleration

    swerve_percep = swerve_perception(data)
    lnf_percep = long_narrow_front_perception()
    swf_percep = short_wide_front_perception()
    wf_percep = wall_following_perception(distance_apart = 0.8,data = data)
    bilateral_percep = bilateral_perception()

    # add all the changes of steering angle from the perception done above
    # steering_angle = swerve_percep[1] + swf_percep[1] + lnf_percep[1]
    steering_angle = swerve_percep[1] + swf_percep[1] + lnf_percep[1]
    
    # when the specific perception is working, the gear will be shifted automatically 
    #   e.g.: when the car is swerving, the gear will be shifted down for a slower speed
    
    if swerve_percep[0] or swf_percep[0] or lnf_percep[0] or bilateral_percep[0]:
        isfullspeed = True
        if swf_percep[0]:
            isfullspeed = False
            shift_gear(4) if simulator_mode == True else shift_gear(1)
        if swerve_percep[0]:
            isfullspeed = False
            shift_gear(4) if simulator_mode == True else shift_gear(1)
        if isfullspeed == True:
            shift_gear(4) if simulator_mode == True else shift_gear(1)
    else:
        steering_angle += 0

def shift_gear(_gear):
    global current_gear
    global speed
    Gear = {-1:-0.2, 0:0.0, 1:0.5, 2:0.75, 3:1.0, 4:2.0, 5:3.0, 6:4.0, 7:5.0, 8:9999.0}
    if current_gear != _gear:
        current_gear = _gear
        speed = Gear[_gear]
        print(_gear)

def swerve_perception(data):
    global distance_set
    alldistance_set = [get_range(data, i) for i in range(180, -181, -1)]

    if min(distance_set[80:100]) < 2.0:
        if min(distance_set[80:100]) < 0.75:
            if max(alldistance_set[60:120]) < max(alldistance_set[210:270]):
                return (True,-0.6)
            else:
                return (True,0.6) 
        return (True, 0.0)
    return (False,0.0)

def wall_following_perception(distance_apart,data):
    global wf_pid_controller
    error = get_wf_error(distance_apart,data)
    steering_angle = wf_pid_controller.get_value(error)

    return (True,steering_angle)

def get_wf_error(distance_apart,data):
    global distance_set

    steering_angle = 0 
    # the angle between the two laser rays
    THETA = 20
    # target distance from wall
    TARGET_DIS = distance_apart
    # the distance to project the car forward
    LOOK_AHEAD_DIS = 0.5

    # naming convention according to above pdf
    b = distance_set[180]
    a = get_range(data, - 90 + THETA)

    # print(f"a{a:1.1f} b{b:1.1f}")
    alpha = np.arctan((a * np.cos(THETA) - b) / (a * np.sin(THETA)))

    Dt = b * np.cos(alpha)
    projected_dis = Dt + LOOK_AHEAD_DIS * np.sin(alpha)
    error = TARGET_DIS - projected_dis
    return error

def long_narrow_front_perception():
    global distance_set
    steering_angle = 0.0
    #max(distance_set[84:97])
    for i in range(15):
        if distance_set[89 - i] < distance_set[91 + i]:
            if 2.0 < distance_set[89 - i] < 10.0:
                steering_angle = -0.15
                break
        else:
            if 2.0 < distance_set[91 + i] < 10.0:
                steering_angle = 0.15
                break
    else:
        return (False,0.0)
    return (True,steering_angle)
    # else:
    # return (False, 0.0)    
        

def short_wide_front_perception():
    global distance_set
    steering_angle = 0.0
    for i in range(30):
        if distance_set[89 - i] < distance_set[91 + i]:
            if distance_set[90 - i] < 0.5:
                steering_angle = -0.4
                break
            if distance_set[89 - i] < 1.6:
                #set_speedmode(2)
                steering_angle = -0.20
                break
        else:
            if distance_set[91 + i] < 0.5:
                steering_angle = 0.4
                break
            if distance_set[91 + i] < 1.6:
                #set_speedmode(2)
                steering_angle = 0.20
                break
    else:
        return (False,0.0)
    return (True,steering_angle)


def bilateral_perception():
    global distance_set
    steering_angle = 0.0
    count = 0.0
    for i in range(60):
        if distance_set[0 + i] < distance_set[180 - i]:
            if distance_set[0 + i] < 0.2:
                steering_angle = -0.3
                break
            if distance_set[0 + i] < 0.5:
                steering_angle = -0.150
                break
        else:
            if distance_set[180 - i] < 0.2:
                steering_angle = 0.3
                break
            if distance_set[180 - i] < 0.5:
                steering_angle = 0.150
                break
    else:
        return (False,0.0)
    return (True,steering_angle)

rospy.init_node("auto_driver")
scan_topic = '/scan'
scan_sub = rospy.Subscriber(scan_topic, LaserScan, callback) 
drive_pub = rospy.Publisher('/tianracer/ackermann_cmd', AckermannDrive, queue_size=10)
drive_pub_simulator = rospy.Publisher('auto_drive', AckermannDriveStamped, queue_size=1)

rospy.spin()


