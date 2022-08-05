#! /usr/bin/env python
import math

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

scan_topic = '/scan'
scan_sub = rospy.Subscriber(scan_topic, LaserScan, callback)

FILLER_VALUE = 100.0

def callback(data):

    ...
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

