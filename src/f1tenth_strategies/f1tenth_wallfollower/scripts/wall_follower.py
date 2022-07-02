#Wall Follower version 1.0
#! /usr/bin/env python
import math

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped

speed = 0.0
steering_angle = 0.0
distance_set = []

speed_mode = 0
distant_narrow_front_perception_mode = 1
near_wide_front_perception_mode = 1
wall_following_mode = 1
wheel_mode = 1


isdangerous = False
FILLER_VALUE = 100.0


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
    dis = data.ranges[int((angle - data.angle_min) / data.angle_increment)]
    if dis < data.range_min or dis > data.range_max:
        dis = FILLER_VALUE
    return dis


def callback(data):
    global speed
    global steering_angle
    global distance_set
    distance_set = [get_range(data, i) for i in range(90, -91, -1)]

    set_speedmode(4)
    wall_following(distance_apart=1.0)
    distant_narrow_front_perception()
    near_wide_front_perception()
    wheel()

    # steering_angle = 0 radian in range [-0.4189, 0.4189]
    drive_msg = AckermannDrive(steering_angle=steering_angle, speed=speed)
    drive_st_msg = AckermannDriveStamped(drive=drive_msg)
    drive_pub.publish(drive_st_msg)

def set_speedmode(speed_m):
    global speed_mode
    global speed
    speed_mode = speed_m
    if speed_mode == 0:
        speed = 0.0
    elif speed_mode == 1:
        speed = 2.5
    elif speed_mode == 2:
        speed = 3.0
    elif speed_mode == 3:
        speed = 4.0
    elif speed_mode == 4:
        speed = 7.0
    print("current speed mode: " + str(speed_mode))


def wall_following(distance_apart):
    global wall_following_mode
    global speed
    global steering_angle
    global distance_set
    if wall_following_mode != 0:
        is_leftwallfollowing = True if wall_following_mode == 1 else False
        i = 0 if is_leftwallfollowing is True else 180
        if distance_set[i] == distance_apart:
            steering_angle = 0
        else:
            d_x = distance_apart - distance_set[i]
            result = 0.001 * math.pow(d_x, 3) + 1.0 / 25.0 * d_x
            if result > 0.025:
                result = 0.025
            if result < -0.025:
                result = -0.025
            steering_angle = - result if is_leftwallfollowing is True else result
            if math.fabs(steering_angle) == 0.025:
                #set_speedmode(2)
                pass


def distant_narrow_front_perception():
    global distant_narrow_front_perception_mode
    global speed
    global steering_angle
    global distance_set

    if distant_narrow_front_perception_mode != 0:
        for i in range(5):
            if distance_set[90 - i] < distance_set[91 + i]:
                if 1.0 < distance_set[90 - i] < 1.6:
                    steering_angle = -0.15
                    break
            else:
                if 1.0 < distance_set[91 + i] < 1.6:
                    steering_angle = 0.15
                    break


def near_wide_front_perception():
    global near_wide_front_perception_mode
    global speed
    global steering_angle
    global distance_set
    if near_wide_front_perception_mode != 0:
        for i in range(45):
            count = 0
            if distance_set[90 - i] < distance_set[91 + i]:
                if distance_set[90 - i] < 0.3:
                    set_speedmode(1)
                    steering_angle = -0.3189
                    count = count + 1
                if distance_set[90 - i] < 0.7:
                    #set_speedmode(2)
                    steering_angle = -0.15
                    count = count + 1
            else:
                if distance_set[91 + i] < 0.3:
                    set_speedmode(1)
                    steering_angle = 0.3189
                    count = count + 1
                if distance_set[91 + i] < 0.7:
                    #set_speedmode(2)
                    steering_angle = 0.15
                    count = count + 1
            if count == 4:
                break


def wheel():
    global speed
    global steering_angle
    global distance_set
    global isdangerous
    if wheel_mode != 0:
        if distance_set[90] > 5:
            pass
        elif distance_set[90] > 1.6:
            set_speedmode(2)
        else:
            set_speedmode(1)
            left_sum = 0
            right_sum = 0
            for i in distance_set[20:90]:
                left_sum = left_sum + i
            for i in distance_set[91:160]:
                right_sum = right_sum + i
            if left_sum / 30 > right_sum / 30:
                steering_angle = 0.4189
            else:
                steering_angle = - 0.4189

rospy.init_node("auto_driver")

scan_topic = '/scan'
scan_sub = rospy.Subscriber(scan_topic, LaserScan, callback)

drive_topic = 'auto_drive'
drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)

rospy.spin()