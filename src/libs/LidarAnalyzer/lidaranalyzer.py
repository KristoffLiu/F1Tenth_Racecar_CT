import sys
import os
import math
import time
import numpy as np

import matplotlib.pyplot as plt
from scipy.ndimage.morphology import binary_erosion

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../FurthestMidPointFinder/")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../Mapping/")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../PathPlanning/")

from VirtualLidarMap.virtuallidarmap import getvirtuallidarmap
from FurthestMidPointFinder.furthestmidpointfinder import find_furthest_midpoint, main
from Mapping.mapping import generate_ray_casting_grid_map
from PathPlanning.HybridAStar.hyprid_a_star import hybrid_a_star_planning

XY_GRID_RESOLUTION = 1.0  # [m]
YAW_GRID_RESOLUTION = np.deg2rad(15.0)  # [rad]

class LidarAnalyzer:
    def __init__(self) -> None:
        pass
        
    def analyze(self, angles, dists):
        self.midpt_ang, self.midpt_dis = find_furthest_midpoint(angles, dists)

        xy_resolution = 0.02  # x-y grid resolution
        ox = np.sin(angles) * dists
        oy = np.cos(angles) * dists
        occupancy_map, min_x, max_x, min_y, max_y, xy_resolution = \
        generate_ray_casting_grid_map(ox, oy, xy_resolution, True)

        k = np.zeros((3,3),dtype=int);
        k = k + 0.5
        k[1] = 0; k[:,1] = 0
        occupancy_map[occupancy_map == 1] = 0.0
        occupancy_map = occupancy_map - binary_erosion(occupancy_map, k)

        n = np.where(occupancy_map[1:occupancy_map.shape[0]-1, 1:occupancy_map.shape[1]-1] == 0.5)
        
        x = n[1:][0]
        y = n[:1][0]

        # # Set Initial parameters
        # start = [0.0, 0.0, np.deg2rad(-90)]
        
        # mx = math.cos(self.midpt_ang) * self.midpt_dis / 0.02 + 50
        # my = math.sin(self.midpt_ang) * self.midpt_dis / 0.02

                # Set Initial parameters
        start = [10, 10, np.deg2rad(-75)]
        
        mx = math.cos(self.midpt_ang) * self.midpt_dis / 0.02
        my = math.sin(self.midpt_ang) * self.midpt_dis / 0.02

        goal = [mx, my, np.deg2rad(-55)]

        path = hybrid_a_star_planning(
        start, goal, [0], [0], XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION)
        # list(x), list(y)
        # [0],[0]
        px = path.x_list
        py = path.y_list
        pyaw = path.yaw_list

        plt.scatter(x, y)
        plt.plot(px, py)
        bottom, top = plt.ylim()  # return the current y-lim
        plt.ylim((top, bottom))  # rescale y axis, to match the grid orientation
        # plt.imshow(occupancy_map, cmap="PiYG_r")
        plt.show()





def main():
    angles, dists = getvirtuallidarmap("lidar03")
    la = LidarAnalyzer()

    lasttime = time.time()
    la.analyze(angles, dists)
    print(time.time() - lasttime)
    # print(la.midpt_ang)
    # print(la.midpt_dis)


if __name__ == '__main__':
    main()
