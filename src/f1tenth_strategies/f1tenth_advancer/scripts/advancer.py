#Wall Follower version 1.0
#! /usr/bin/env python
from dis import dis
import math
import threading
from turtle import right
import rospy
import numpy as np
import sys, os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../../")
from libs.FurthestMidPointFinder.furthestmidpointfinder import find_furthest_midpoint

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped


class F1tenthAdvancer:
    def __init__(self, min_speed = 2.0, max_speed = 4.0, safety_dist = 8.0):
        self.MIN_SPEED = min_speed
        self.MAX_SPEED = max_speed
        self.LIDAR_2_FRONTCAR_DISTANCE = 0.15
        self.BRAKE_DISTANCE = 0.9
        self.SAFETY_DISTANCE = 2.0
        self.MAX_REACT_DISTANCE = safety_dist
        self.MAX_STEERING_ANGLE = 0.4189
        self.FILLER_VALUE = 100.0

        self.MIN_MAP_WIDTH = 2.0
        self.DISTANCE_APART = 1.0

        self.speed = 0
        self.steering_angle = 0
        self.lidar_data = []
        self.speed_mode = 0

        self._scan_topic = '/scan'
        self._scan_subscriber = rospy.Subscriber(self._scan_topic, LaserScan, self.scan_data)

        self._drive_topic = '/auto_drive'
        self._drive_publisher = rospy.Publisher(self._drive_topic, AckermannDriveStamped, queue_size=1)
    
    def scan_data(self, msg):
        self.lidar_data = [self.get_range(msg, i) for i in range(90, -91, -1)]

    def get_range(self, msg, angle, deg=True):
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
        dis = msg.ranges[int((angle - msg.angle_min) / msg.angle_increment)]
        if dis < msg.range_min or dis > msg.range_max:
            dis = self.FILLER_VALUE
        return dis
    
    def step(self):
        if self.lidar_data == []:
            return
        self.steering_angle = 0

        # self.steering_angle += self.wall_following()
        min_front_dist = self.distant_narrow_front_perception()
        self.steering_angle += self.near_wide_front_perception(min_front_dist)
        self.side_perception()
        
        drive_msg    = AckermannDrive(steering_angle = self.steering_angle,  speed = self.speed)
        drive_st_msg = AckermannDriveStamped(drive=drive_msg)
        self._drive_publisher.publish(drive_st_msg)

        print("steering: ", self.steering_angle)
        print("speed: ", self.speed)
        print("min distance: ", min_front_dist)
        
    def wall_following(self):
        is_leftwallfollowing = True
        i = 0 if is_leftwallfollowing is True else 180
        if self.lidar_data[i] == self.DISTANCE_APART:
            steering_angle = 0
        else:
            d_x = self.DISTANCE_APART - self.lidar_data[i]
            result = 0.005 * math.pow(d_x, 3) + 1.0 / 25.0 * d_x
            if result > 0.225:
                result = 0.225
            if result < -0.225:
                result = -0.225
            steering_angle = - result if is_leftwallfollowing is True else result
        return steering_angle

    def distant_narrow_front_perception(self): 
        min_dist = self.lidar_data[80]
        for i in range(19):
            if min_dist > self.lidar_data[81 + i]:
                min_dist = self.lidar_data[81 + i]
        if self.MAX_REACT_DISTANCE <= min_dist:
            min_dist = self.MAX_REACT_DISTANCE
        
        if min_dist <= self.LIDAR_2_FRONTCAR_DISTANCE:
            self.speed = 0
        elif min_dist <= self.BRAKE_DISTANCE:
            self.speed = self.MIN_SPEED / (self.BRAKE_DISTANCE - self.LIDAR_2_FRONTCAR_DISTANCE) * (min_dist - self.LIDAR_2_FRONTCAR_DISTANCE)
        elif min_dist <= self.SAFETY_DISTANCE:
            self.speed = self.MIN_SPEED
        else:
            input = (min_dist - self.SAFETY_DISTANCE) / (self.MAX_REACT_DISTANCE - self.SAFETY_DISTANCE)
            self.speed = self.easeOutCubic(input) * (self.MAX_SPEED - self.MIN_SPEED) + self.MIN_SPEED
        return min_dist        

    def easeInCubic(self, x):
        return x * x * x

    def easeInSine(self, x):
        return 1 - math.cos(x * math.pi / 2)

    def easeOutCubic(self, x):
        return 1 - math.pow(1 - x, 3)

    def easeInOutSine(self, x):
        return -(math.cos(math.pi * x) - 1) / 2

    def easeInOutCubic(self, x):
        if x < 0.5:
            return 4 * x * x * x
        else:
            return 1 - pow(-2 * x + 2, 3) / 2

    def near_wide_front_perception(self, min_front_dist):
        steering_delta = 0

        angles = [i for i in range(-90, 90, 1)]
        dists  = self.lidar_data[0:180]
        midpt_angle, midpt_dist = find_furthest_midpoint(angles, dists)
        
        if min_front_dist <= midpt_dist:
            if midpt_angle > 0:
                input = midpt_angle / 90
                steering_delta += - self.easeInSine(input) * self.MAX_STEERING_ANGLE
            else:
                input = midpt_angle / 90
                steering_delta += self.easeInSine(input) * self.MAX_STEERING_ANGLE
        if min_front_dist <= self.MIN_MAP_WIDTH:
            leftcount = 0
            rightcount = 0
            for i in range(90):
                diff = self.lidar_data[90 - i] - self.lidar_data[91 + i]
                if diff > self.MIN_MAP_WIDTH / 2.0:
                    leftcount += 1
                elif diff < - self.MIN_MAP_WIDTH / 2.0:
                    rightcount += 1
            if leftcount > rightcount:
                steering_delta += 0.3
            else:
                steering_delta -= 0.3
        return steering_delta

    
    def side_perception(self):
        left_side_angles = [i for i in range(-45, -30, 1)]
        left_side_dists  = self.lidar_data[30:45]

        right_side_angles = [i for i in range(45, 60, 1)]
        right_side_dists  = self.lidar_data[135:150]

        min_left_angle, min_left_dist = self.min_dist_direction(left_side_angles, left_side_dists)
        min_right_angle, min_right_dist = self.min_dist_direction(right_side_angles, right_side_dists)
        
        # if min_left_dist < 0.4:
        #     self.steering_angle = - 0.4
        # if min_right_dist < 0.4:
        #     self.steering_angle =   0.4
            
    
    def min_dist_direction(self, angles, dists):
        min_dist_index = 0
        for i in range(len(dists)):
            if dists[min_dist_index] > dists[i]:
                min_dist_index = i
        return angles[min_dist_index], dists[min_dist_index]



def call_rosspin():
    rospy.spin()

if __name__ == '__main__':
    try:
        print(__file__ + " start!!")
        rospy.init_node('advancer', anonymous=True)
        rospy_thread = rospy.Rate(120)
        f1tenth = F1tenthAdvancer(3.0, 5.0, 8.0)

        spin_thread = threading.Thread(target=call_rosspin).start()

        while not rospy.core.is_shutdown():
            # os.system('clear')
            f1tenth.step()
            rospy_thread.sleep()

    except rospy.ROSInterruptException:
        pass
