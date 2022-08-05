import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../libs/PathTracking/")

import rospy
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped

import pure_pursuit
from pure_pursuit import State
from pure_pursuit import States

from nav_msgs.msg import Path

class Tracker:
    def __init__(self):
        self._control_rate = 0.5
        self.raw_path = []
        planner_topic = 'f1tenth_planner'
        self._planner_suscriber = rospy.Subscriber(planner_topic, Path, self.raw_path)
        
        drive_topic = 'auto_drive'
        self._drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)

    def step(self):
        pass



