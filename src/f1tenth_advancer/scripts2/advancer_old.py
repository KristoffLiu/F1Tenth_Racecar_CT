# Wall Follower version 1.0
#! /usr/bin/env python


import math
import threading
from tkinter.messagebox import RETRY
import rospy
import numpy as np
import time
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../../")
from libs.EasingFunctions import easingfunctions
from libs.FurthestMidPointFinder.furthestmidpointfinder import find_furthest_midpoint

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from frontrectangleperception import FrontRectPerceptor

class F1tenthAdvancerConfig:
    def __init__(self):
        pass


class F1tenthAdvancer:
    def __init__(self,
                 is_simulator=True,
                 min_speed=2.0,
                 max_speed=4.0,
                 safety_dist=8.0):
        self.is_simulator = is_simulator

        self.MIN_SPEED = min_speed
        self.MAX_SPEED = max_speed
        self.LIDAR_2_FRONTCAR_DISTANCE = 0.15
        self.BRAKE_DISTANCE = 0.5
        self.SAFETY_DISTANCE = 2.0
        self.MAX_REACT_DISTANCE = safety_dist
        self.MAX_STEERING_ANGLE = 0.5
        self.FILLER_VALUE = 100.0

        self.CAR_WIDTH = 0.185

        self.MIN_ROAD_WIDTH = 1.5
        self.DISTANCE_APART = 1.0

        self.NOTICE_GAP_WIDTH = 0.05

        self.speed = 0
        self.steering_angle = 0
        self.min_front_dist = 0

        self.lidar_data = []
        self.odom = None

        self.count = 0

        self.front_rect_perceptor = FrontRectPerceptor(
            self.CAR_WIDTH + 0.16, self.LIDAR_2_FRONTCAR_DISTANCE)

        if is_simulator:
            from scipy.spatial.transform import Rotation as R

            self.odom_topic = '/odom_2'
            self.odom_pub = rospy.Subscriber(
                self.odom_topic, Odometry, self.get_odom)

            self._scan_topic = '/scan_2'
            self._scan_subscriber = rospy.Subscriber(
                self._scan_topic, LaserScan, self.scan_simulated)

            self._drive_topic = '/drive_2'
            self._drive_publisher = rospy.Publisher(
                self._drive_topic, AckermannDriveStamped, queue_size=1)

            self._vis_pub = [rospy.Publisher(
                "/visualization_marker", Marker, queue_size=10)]
        else:
            self._scan_topic = '/scan'
            self._scan_subscriber = rospy.Subscriber(
                self._scan_topic, LaserScan, self.scan)

            self._drive_topic = '/tianracer/ackermann_cmd'
            self._drive_publisher = rospy.Publisher(
                self._drive_topic, AckermannDrive, queue_size=1)

    def scan_simulated(self, msg):
        prev = time.time()
        if self.count == 8:
            self.lidar_data = [self.get_range(msg, i)
                               for i in range(90, -91, -1)]
            self.step_whenlidarupdate()
            self.count = 0
            print(time.time() - prev)
        else:
            self.count += 1

    def scan(self, msg):
        prev = time.time()
        self.lidar_data = [self.get_range(msg, i)
                           for i in range(90, -91, -1)]
        self.step_whenlidarupdate()
        self.count = 0
        print(time.time() - prev)

    def get_odom(self, msg):
        self.odom = msg

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

    def step_whenlidarupdate(self):
        if self.lidar_data == []:
            return
        self.steering_angle = 0

        min_angle, self.min_front_dist = self.front_rect_perceptor.percept_nearest_obstacle(
            self.lidar_data, self.steering_angle)

        self.speed_servo()
        self.steering_angle = self.steer_servo()

        drive_msg = AckermannDrive(
            steering_angle=self.steering_angle,  speed=self.speed)
        drive_st_msg = AckermannDriveStamped(drive=drive_msg)
        self._drive_publisher.publish(drive_st_msg)

        # self.displayInConsole()

    def speed_servo(self):
        if self.MAX_REACT_DISTANCE <= self.min_front_dist:
            self.min_front_dist = self.MAX_REACT_DISTANCE

        if self.min_front_dist <= self.LIDAR_2_FRONTCAR_DISTANCE:
            self.speed = 0
        elif self.min_front_dist <= self.BRAKE_DISTANCE:
            self.speed = self.MIN_SPEED / (self.BRAKE_DISTANCE - self.LIDAR_2_FRONTCAR_DISTANCE) * (
                self.min_front_dist - self.LIDAR_2_FRONTCAR_DISTANCE)
        elif self.min_front_dist <= self.SAFETY_DISTANCE:
            self.speed = self.MIN_SPEED
        else:
            normalized_input = (self.min_front_dist - self.SAFETY_DISTANCE) / \
                (self.MAX_REACT_DISTANCE - self.SAFETY_DISTANCE)
            self.speed = easingfunctions.easeOutCubic(
                normalized_input) * (self.MAX_SPEED - self.MIN_SPEED) + self.MIN_SPEED

    def steer_servo(self):
        steering_delta = 0

        angles = [i for i in range(-80, 80, 1)]
        dists = self.lidar_data[0:160]

        midpt_angle, midpt_dist = find_furthest_midpoint(angles, dists)

        # steering_delta += self._steer_narrow_front()
        if self.lidar_data[0] < 0.08 or self.lidar_data[180] < 0.08:
            return 0

        if self.min_front_dist <= self.MIN_ROAD_WIDTH * 1.4:
            if midpt_angle > 0:
                input = midpt_angle / 90
                steering_delta += - easingfunctions.easeInSine(input) * self.MAX_STEERING_ANGLE * 0.45
            else:
                input = midpt_angle / 90
                steering_delta += easingfunctions.easeInSine(input) * self.MAX_STEERING_ANGLE * 0.45

        # if min_front_dist <= self.MIN_ROAD_WIDTH:
        leftcount = 0
        rightcount = 0
        for i in range(15,90,1):
            diff = self.lidar_data[90 - i] - self.lidar_data[91 + i]
            if diff > self.MIN_ROAD_WIDTH / 2.0:
                leftcount += 1
            elif diff < - self.MIN_ROAD_WIDTH / 2.0:
                rightcount += 1
        input = self.min_front_dist / self.MAX_REACT_DISTANCE
        if leftcount > rightcount:
            steering_delta += (1 - easingfunctions.easeOutCubic(input)) * self.MAX_STEERING_ANGLE
        else:
            steering_delta -= (1 - easingfunctions.easeOutCubic(input)) * self.MAX_STEERING_ANGLE
        
        return steering_delta

    # def _steer_narrow_front(self):
    #     leftcount = 0
    #     rightcount = 0
    #     for i in range(6):
    #         leftcount += self.lidar_data[90 - i]
    #         rightcount += self.lidar_data[91 + i]
    #     if leftcount > rightcount:
    #         return 0.025
    #     else:
    #         return - 0.025

    def vision_mark(self, x, y):
        mark = Marker()
        mark.header.frame_id = "/map"
        mark.header.stamp = rospy.Time.now()
        mark.ns = "showen_point"
        mark.id = 1
        mark.type = Marker().POINTS
        mark.action = Marker().ADD
        mark.pose.orientation.w = 1.0
        mark.scale.x = 0.4
        mark.scale.y = 0.2
        mark.scale.z = 0.5
        mark.color.a = 1.0
        mark.color.r = 0.2
        mark.color.g = 1.0
        mark.color.b = 0.3
        # mark.lifetime = rospy.Duration(0.02, 0)
        mark.points.append(Point(x, y, 0))
        self._vis_pub[0].publish(mark)

    def _createQuaternionFromYaw(self, yaw):
        # input: r p y
        r = R.from_euler('zyx', [0, 0, yaw], degrees=False).as_quat()
        # output: w x y z
        return [r[3], r[2], r[1], r[0]]

    def _createYawFromQuaternion(self, quaternion):
        [y, p, r] = R.from_quat([quaternion.x,
                                quaternion.y,
                                quaternion.z,
                                quaternion.w]).as_euler('zyx', degrees=False)
        return y

    def side_perception(self):
        left_side_angles = [i for i in range(-45, -30, 1)]
        left_side_dists = self.lidar_data[30:45]

        right_side_angles = [i for i in range(45, 60, 1)]
        right_side_dists = self.lidar_data[135:150]

        min_left_angle, min_left_dist = self.min_dist_direction(
            left_side_angles, left_side_dists)
        min_right_angle, min_right_dist = self.min_dist_direction(
            right_side_angles, right_side_dists)

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

    def display(self):
        self.display_text_in_rviz()

    def display_text_in_rviz(self):
        if self.odom != None:
            smark = Marker()
            smark.header.frame_id = "/map"
            smark.header.stamp = rospy.Time.now()
            smark.ns = "showen_point"
            smark.id = 2
            smark.type = Marker().TEXT_VIEW_FACING
            smark.action = Marker().ADD
            smark.pose.orientation.w = 1.0
            smark.scale.x = 0.4
            smark.scale.y = 0.2
            smark.scale.z = 0.5
            smark.color.a = 1.0
            smark.color.r = 1.0
            smark.color.g = 1.0
            smark.color.b = 1.0
            # mark.lifetime = rospy.Duration(0.02, 0)
            pose = Pose()
            pose.position.x = self.odom.pose.pose.position.x
            pose.position.y = self.odom.pose.pose.position.y
            pose.position.z = 2.4
            smark.text = "S: " + str("%.2f" % self.steering_angle) + " r      "
            smark.pose = pose

            mark = Marker()
            mark.header.frame_id = "/map"
            mark.header.stamp = rospy.Time.now()
            mark.ns = "showen_point"
            mark.id = 1
            mark.type = Marker().TEXT_VIEW_FACING
            mark.action = Marker().ADD
            mark.pose.orientation.w = 1.0
            mark.scale.x = 0.4
            mark.scale.y = 0.2
            mark.scale.z = 0.5
            mark.color.a = 1.0
            mark.color.r = 1.0
            mark.color.g = 1.0
            mark.color.b = 1.0
            # mark.lifetime = rospy.Duration(0.02, 0)
            pose = Pose()
            pose.position.x = self.odom.pose.pose.position.x
            pose.position.y = self.odom.pose.pose.position.y
            pose.position.z = 1.75
            mark.text = "V: " + str("%.2f" % self.speed) + " m/s    "
            mark.pose = pose

            nmark = Marker()
            nmark.header.frame_id = "/map"
            nmark.header.stamp = rospy.Time.now()
            nmark.ns = "showen_point"
            nmark.id = 3
            nmark.type = Marker().TEXT_VIEW_FACING
            nmark.action = Marker().ADD
            nmark.pose.orientation.w = 1.0
            nmark.scale.x = 0.4
            nmark.scale.y = 0.2
            nmark.scale.z = 0.5
            nmark.color.a = 1.0
            nmark.color.r = 1.0
            nmark.color.g = 1.0
            nmark.color.b = 1.0
            # mark.lifetime = rospy.Duration(0.02, 0)
            pose = Pose()
            pose.position.x = self.odom.pose.pose.position.x
            pose.position.y = self.odom.pose.pose.position.y
            pose.position.z = 1.1
            nmark.text = "MFD: " + str("%.2f" % self.min_front_dist) + " m  "
            nmark.pose = pose

            self._vis_pub[0].publish(mark)
            self._vis_pub[0].publish(smark)
            self._vis_pub[0].publish(nmark)

    def displayInConsole(self):
        print("steering: ", self.steering_angle)
        print("speed: ", self.speed)
        print("min distance: ", self.min_front_dist)


def call_rosspin():
    rospy.spin()


if __name__ == '__main__':
    try:
        print(__file__ + " start!!")
        rospy.init_node('advancer', anonymous=True)
        rospy_thread = rospy.Rate(120)
        f1tenth = F1tenthAdvancer(True, 1.5, 3.0, 8.0)

        spin_thread = threading.Thread(target=call_rosspin).start()

        while not rospy.core.is_shutdown():
            if f1tenth.is_simulator:
                f1tenth.display()
            # f1tenth.step()
            f1tenth.display_text_in_rviz()

            rospy_thread.sleep()

    except rospy.ROSInterruptException:
        pass
