#! /usr/bin/env python
import math
import sys
import os
from turtle import pos

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../..")

from libs.LidarAnalyzer.lidaranalyzer import LidarAnalyzer

import rospy
import numpy as np
import threading
from sensor_msgs.msg import LaserScan

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from scipy.spatial.transform import Rotation as R
import numpy as np

import time

FILLER_VALUE = 100.0
odom = None
la = LidarAnalyzer()

isprevdone = True

def callback(data):
    global isprevdone
    global odom

    a = time.time()
    if isprevdone == None:
        return
    if not isprevdone:
        return
    else:
        isprevdone = False

    global speed
    global steering_angle
    global distance_set

    distance_set = [get_range(data, i) for i in range(90, -135, -1)]
    distance_set += [0.3 for i in range(-135, -180, -1)]
    distance_set += [0.3 for i in range(180, 135, -1)]
    distance_set += [get_range(data, i) for i in range(135, 90, -1)]
    angle_set = [math.radians(ang) for ang in range(len(distance_set))]

    data = np.array([angle_set, distance_set]).T
    np.savetxt(os.path.abspath(__file__).replace("planner.py","") + "/" + "data4.csv", data, fmt="%f", delimiter=",") 

    raw_x, raw_y, raw_yaw = la.analyze(angle_set, distance_set)

    

    path = Path()
    path.header.frame_id = "/map"
    path.header.stamp = rospy.get_rostime()
    
    for (x, y, yaw) in zip(raw_x, raw_y, raw_yaw):
        goal = PoseStamped()
        goal.header.frame_id = "/map"
        goal.pose.position.x, goal.pose.position.y = -y, -x

        [goal.pose.orientation.w,
        goal.pose.orientation.x,
        goal.pose.orientation.y,
        goal.pose.orientation.z] = _createQuaternionFromYaw(yaw)

        if odom != None:

            _x = goal.pose.position.x
            _y = goal.pose.position.y
            yaw = _createYawFromQuaternion(odom.pose.pose.orientation)
            goal.pose.position.x = math.cos(yaw)*_x - math.sin(yaw)*_y
            goal.pose.position.y = math.cos(yaw)*_y + math.sin(yaw)*_x;

            goal.pose.position.x += odom.pose.pose.position.x
            goal.pose.position.y += odom.pose.pose.position.y

        path.poses.append(goal)
    
    path_pub.publish(path)
    
    isprevdone = True
    print(time.time()- a)


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

def _createQuaternionFromYaw(yaw):
    # input: r p y
    r = R.from_euler('zyx', [0, 0, yaw], degrees=False).as_quat()
    # output: w x y z
    return [r[3], r[2], r[1], r[0]]

def _createYawFromQuaternion(quaternion):
    [y, p, r] = R.from_quat([quaternion.x,
                             quaternion.y,
                             quaternion.z,
                             quaternion.w]).as_euler('zyx', degrees=False)
    return y

scan_topic = '/scan'
scan_sub = rospy.Subscriber(scan_topic, LaserScan, callback)

path_topic = '/local_path'
path_pub = rospy.Publisher(path_topic, Path, queue_size=10)

def callback2(data):
    global odom
    odom = data


odom_topic = '/odom'
odom_pub = rospy.Subscriber(odom_topic, Odometry, callback2)

def call_rosspin():
    rospy.spin()

def main():
    isprevdone = True
    rospy.init_node("planner", anonymous=True)
    control_rate = 0.5
    rate = rospy.Rate(1.0 / control_rate)
    spin_thread = threading.Thread(target=call_rosspin).start()

    while not rospy.core.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()

