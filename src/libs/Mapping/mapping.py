"""
LIDAR to 2D grid map example
author: Erno Horvath, Csaba Hajdu based on Atsushi Sakai's scripts
"""

from cmath import pi
from dis import dis
import math
from collections import deque
from pickle import FALSE
from turtle import left
from typing import Mapping

import matplotlib.pyplot as plt
import numpy as np
from torch import less_equal


import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../VirtualLidarMap/")
from virtuallidarmap import getvirtuallidarmap

EXTEND_AREA = 5.0

def bresenham(start, end):
    """
    Implementation of Bresenham's line drawing algorithm
    See en.wikipedia.org/wiki/Bresenham's_line_algorithm
    Bresenham's Line Algorithm
    Produces a np.array from start and end (original from roguebasin.com)
    >>> points1 = bresenham((4, 4), (6, 10))
    >>> print(points1)
    np.array([[4,4], [4,5], [5,6], [5,7], [5,8], [6,9], [6,10]])
    """
    # setup initial conditions
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1
    is_steep = abs(dy) > abs(dx)  # determine how steep the line is
    if is_steep:  # rotate line
        x1, y1 = y1, x1
        x2, y2 = y2, x2
    # swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
    dx = x2 - x1  # recalculate differentials
    dy = y2 - y1  # recalculate differentials
    error = int(dx / 2.0)  # calculate error
    y_step = 1 if y1 < y2 else -1
    # iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = [y, x] if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += y_step
            error += dx
    if swapped:  # reverse the list if the coordinates were swapped
        points.reverse()
    points = np.array(points)
    return points


def calc_grid_map_config(ox, oy, xy_resolution):
    """
    Calculates the size, and the maximum distances according to the the
    measurement center
    """
    min_x = min(ox) / xy_resolution - EXTEND_AREA / 2.0
    min_y = min(oy) / xy_resolution - EXTEND_AREA / 2.0
    max_x = max(ox) / xy_resolution + EXTEND_AREA / 2.0
    max_y = max(oy) / xy_resolution + EXTEND_AREA / 2.0

    xw = int(max_x - min_x)
    yw = int(max_y - min_y)
    print("The grid map is ", xw, "x", yw, ".")
    return min_x, min_y, max_x, max_y, xw, yw


def atan_zero_to_twopi(y, x):
    angle = math.atan2(y, x)
    if angle < 0.0:
        angle += math.pi * 2.0
    return angle


def init_flood_fill(center_point, obstacle_points, xy_points, min_coord,
                    xy_resolution):
    """
    center_point: center point
    obstacle_points: detected obstacles points (x,y)
    xy_points: (x,y) point pairs
    """
    center_x, center_y = center_point
    prev_ix, prev_iy = center_x - 1, center_y
    ox, oy = obstacle_points
    xw, yw = xy_points
    min_x, min_y = min_coord
    occupancy_map = (np.zeros((xw, yw)))
    for (x, y) in zip(ox, oy):
        # x coordinate of the the occupied area
        ix = int(x / xy_resolution - min_x)
        # y coordinate of the the occupied area
        iy = int(y / xy_resolution - min_y)
        free_area = bresenham((prev_ix, prev_iy), (ix, iy))
        for fa in free_area:
            occupancy_map[fa[0]][fa[1]] = 1  # free area 0.0
        prev_ix = ix
        prev_iy = iy
    return occupancy_map


def flood_fill(center_point, occupancy_map):
    """
    center_point: starting point (x,y) of fill
    occupancy_map: occupancy map generated from Bresenham ray-tracing
    """
    # Fill empty areas with queue method
    sx, sy = occupancy_map.shape
    fringe = deque()
    fringe.appendleft(center_point)
    while fringe:
        n = fringe.pop()
        nx, ny = n
        # West
        if nx > 0:
            if occupancy_map[nx - 1, ny] == 0.0:
                occupancy_map[nx - 1, ny] = 1.0
                fringe.appendleft((nx - 1, ny))
        # East
        if nx < sx - 1:
            if occupancy_map[nx + 1, ny] == 0.0:
                occupancy_map[nx + 1, ny] = 1.0
                fringe.appendleft((nx + 1, ny))
        # North
        if ny > 0:
            if occupancy_map[nx, ny - 1] == 0.0:
                occupancy_map[nx, ny - 1] = 1.0
                fringe.appendleft((nx, ny - 1))
        # South
        if ny < sy - 1:
            if occupancy_map[nx, ny + 1] == 0.0:
                occupancy_map[nx, ny + 1] = 1.0
                fringe.appendleft((nx, ny + 1))


def generate_ray_casting_grid_map(ox, oy, xy_resolution, breshen=True):
    """
    The breshen boolean tells if it's computed with bresenham ray casting
    (True) or with flood fill (False)
    """
    min_x, min_y, max_x, max_y, x_w, y_w = calc_grid_map_config(
        ox, oy, xy_resolution)
    # default 0.5 -- [[0.5 for i in range(y_w)] for i in range(x_w)]
    occupancy_map = np.zeros((x_w, y_w))
    center_x = int(0 - min_x + EXTEND_AREA / 2.0)  # center x coordinate of the grid map
    center_y = int(0 - min_y + EXTEND_AREA / 2.0)  # center y coordinate of the grid map
    # occupancy grid computed with bresenham ray casting
    if breshen:
        for (x, y) in zip(ox, oy):
            # x coordinate of the the occupied area
            ix = int(x / xy_resolution - min_x)
            # y coordinate of the the occupied area
            iy = int(y / xy_resolution - min_y)
            laser_beams = bresenham((center_x, center_y), (
                ix, iy))  # line form the lidar to the occupied point
            for laser_beam in laser_beams:
                occupancy_map[laser_beam[0]][
                    laser_beam[1]] = 1.0  # free area 0.0
            occupancy_map[ix][iy] = 0.5  # occupied area 1.0
            # occupancy_map[ix + 1][iy] = 0.5  # extend the occupied area
            # occupancy_map[ix][iy + 1] = 0.5  # extend the occupied area
            # occupancy_map[ix + 1][iy + 1] = 0.5  # extend the occupied area
    # occupancy grid computed with with flood fill
    else:
        occupancy_map = init_flood_fill((center_x, center_y), (ox, oy),
                                        (x_w, y_w),
                                        (min_x, min_y), xy_resolution)
        
        # return occupancy_map, min_x, max_x, min_y, max_y, xy_resolution

        flood_fill((center_x, center_y), occupancy_map)
        occupancy_map = np.array(occupancy_map, dtype=float)
        for (x, y) in zip(ox, oy):
            ix = int(x / xy_resolution - min_x)
            iy = int(y / xy_resolution - min_y)
            occupancy_map[ix][iy] = 0.5 # occupied area 1.0
            # occupancy_map[ix + 1][iy] = 1.0  # extend the occupied area
            # occupancy_map[ix][iy + 1] = 1.0  # extend the occupied area
            # occupancy_map[ix + 1][iy + 1] = 1.0  # extend the occupied area
        # occupancy_map[occupancy_map == 0] = 0.9
        # occupancy_map[occupancy_map == 1] = 0
        # occupancy_map[occupancy_map == 0.9] = 1
    return occupancy_map, min_x, max_x, min_y, max_y, xy_resolution

def farthestdistance(ang, dists):
    index = 0
    for j in range(len(dists)):
        if dists[j] > dists[index]:
            index = j
    return ang[index], dists[index]

def nearestclosestdistance(ang, dists, index):
    if dists[index - 1] < dists[index + 1]:
        return ang[index - 1], dists[index - 1]
    else:
        return ang[index + 1], dists[index + 1]

def main():
    """
    Example usage
    """
    print(__file__, "start")

    xy_resolution = 0.2  # x-y grid resolution
    ang, dist = getvirtuallidarmap("data4")
    dist = np.concatenate(([0.4 for _ in range(25)],dist[25:155],[0.4 for _ in range(205)]),axis = 0)

    fangle, fdist = farthestdistance(ang, dist)
    _fx = - fdist * math.cos(fangle)
    _fy = - fdist * math.sin(fangle)
    fx = np.array([0, _fx])
    fy = np.array([0, _fy])

    ox = - np.cos(ang) * dist
    oy = - np.sin(ang) * dist
    
    occupancy_map, min_x, max_x, min_y, max_y, xy_resolution = \
        generate_ray_casting_grid_map(oy, ox, xy_resolution, False)
    xy_res = np.array(occupancy_map).shape



    plt.figure(1, figsize=(10, 4))
    plt.subplot(122)
    plt.imshow(occupancy_map, cmap="RdYlGn_r")
    # cmap = "binary" "PiYG_r" "PiYG_r" "bone" "bone_r" "RdYlGn_r"
    plt.clim(0, 1.0)
    plt.gca().set_xticks(np.arange(-.5, xy_res[1], 1), minor=True)
    plt.gca().set_yticks(np.arange(-.5, xy_res[0], 1), minor=True)
    plt.grid(True, which="minor", color="w", linewidth=0.6, alpha=0.5)
    plt.colorbar()
    plt.subplot(121)
    plt.plot([ox, np.zeros(np.size(oy))], [oy, np.zeros(np.size(oy))], "ro-")
    plt.axis("equal")
    plt.plot(0.0, 0.0, "ob")
    plt.plot(fx, fy)
    plt.gca().set_aspect("equal", "box")
    bottom, top = plt.ylim()  # return the current y-lim
    plt.ylim((top, bottom))  # rescale y axis, to match the grid orientation
    plt.grid(True)
    plt.show()


if __name__ == '__main__':
    main()