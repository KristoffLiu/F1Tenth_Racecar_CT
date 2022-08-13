import sys
import os
import math
import time
import numpy as np

import heartrate

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

XY_GRID_RESOLUTION = 2.0  # [m]
YAW_GRID_RESOLUTION = np.deg2rad(15.0)  # [rad]

class LidarAnalyzer:
    def __init__(self) -> None:
        pass
        
    def analyze(self, angles, dists):
        dists = np.concatenate(([1 for _ in range(30)],dists[30:150],[1 for _ in range(210)]),axis = 0)
        # dists[dists > 10] = 10
        # dists = np.concatenate((dists[0:180],[1 for _ in range(180)]),axis = 0)

        self.midpt_ang, self.midpt_dis = find_furthest_midpoint(angles, dists)

        xy_resolution = 0.2  # x-y grid resolution
        
        ox = - np.cos(angles) * dists
        oy = - np.sin(angles) * dists
        occupancy_map, min_x, max_x, min_y, max_y, xy_resolution = \
        generate_ray_casting_grid_map(oy, ox, xy_resolution, False)

        mx = - math.cos(self.midpt_ang) * self.midpt_dis / xy_resolution
        my = - math.sin(self.midpt_ang) * self.midpt_dis / xy_resolution
        
        occupancy_map[int(my- min_x-5):int(my- min_x+5), int(mx- min_y-5):int(mx- min_y+5)] = 1

        # plt.imshow(occupancy_map, cmap="PiYG_r")
        # bottom, top = plt.ylim()  # return the current y-lim
        # plt.ylim((top, bottom))  # rescale y axis, to match the grid orientation
        # plt.show()

        k = np.zeros((3,3),dtype=int);
        k[1] = 1; k[:,1] = 1
        occupancy_map[occupancy_map == 0.5] = 1
        occupancy_map = occupancy_map - binary_erosion(occupancy_map, k)
        n = np.where(occupancy_map[1:occupancy_map.shape[0]-1, 1:occupancy_map.shape[1]-1] == 1)

        x = n[1:][0]
        y = n[:1][0]

        start = [0 - min_y, 0 - min_x, np.deg2rad(-90)]
        goal = [mx - min_y, my - min_x, np.deg2rad(-45)]

        path = hybrid_a_star_planning(
        start, goal, list(x), list(y), XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION)
        
        x = (n[1:][0] + min_y) * xy_resolution
        y = (n[:1][0] + min_x) * xy_resolution

        path.x_list += min_y
        path.y_list += min_x
        path.x_list *= xy_resolution
        path.y_list *= xy_resolution

        # plt.scatter(x, y)
        # plt.plot(path.x_list, path.y_list)
        # bottom, top = plt.ylim()  # return the current y-lim
        # plt.ylim((top, bottom))  # rescale y axis, to match the grid orientation
        # # plt.imshow(occupancy_map, cmap="PiYG_r")
        # plt.show()
        return path.x_list, path.y_list, path.yaw_list





def main():
    # heartrate.trace(browser = True)
    angles, dists = getvirtuallidarmap("data3")
    la = LidarAnalyzer()

    lasttime = time.time()
    la.analyze(angles, dists)
    print(time.time() - lasttime)
    # print(la.midpt_ang)
    # print(la.midpt_dis)


if __name__ == '__main__':
    main()
