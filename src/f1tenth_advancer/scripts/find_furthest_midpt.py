import math
import threading
import rospy
import numpy as np
import time
import sys, os

import sys, os

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../../")
from libs.FurthestMidPointFinder.furthestmidpointfinder import find_furthest_midpoint

def scan(self, msg):
    prev = time.time()
    self.lidar_data = [self.get_range(msg, i)
                       for i in range(90, -91, -1)]
    
    self.count = 0
    print(time.time() - prev)

scan_topic = '/scan'
scan_sub = rospy.Subscriber(scan_topic, LaserScan, scan)

path_topic = '/local_path'
path_pub = rospy.Publisher(path_topic, Pose, queue_size=10)



def main():
    midpt_angle, midpt_dist = find_furthest_midpoint(angles, dists)

if __name__ == '__main__':
    main()