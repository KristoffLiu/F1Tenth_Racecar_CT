import numpy as np
import math
import matplotlib.pyplot as plt

def furthestdistance_index(angles, dists):
    index = 0
    for j in range(len(dists)):
        if dists[j] > dists[index]:
            index = j
    return index

def nearestclosestdistance_index(ang, dists, index):
    if dists[index - 10] < dists[index + 10]:
        return index - 10
    else:
        return index + 10

def find_furthest_midpoint(angles, dists):
    fd_index = furthestdistance_index(angles, dists)
    ncd_index = nearestclosestdistance_index(angles, dists, fd_index)
    fd = dists[fd_index]
    ncd = dists[ncd_index]
    ratio = fd / ncd
    diff_dis = fd - ncd
    dis_tofd = diff_dis / ratio
    midpt_dis = fd - dis_tofd
    midpt_ang = angles[fd_index]
    return midpt_ang, midpt_dis

def file_read(f):
    """
    Reading LIDAR laser beams (anglells and corresponding distance data)
    """
    with open(f) as data:
        measures = [line.split(",") for line in data]
    angles = []
    distances = []
    for measure in measures:
        angles.append(float(measure[0]))
        distances.append(float(measure[1]))
    angles = np.array(angles)
    distances = np.array(distances)
    return angles, distances

def main():
    angles, distances = file_read("lidar03.csv")
    midpt_ang, midpt_dis = find_furthest_midpoint(angles, distances)
    print(midpt_ang)
    print(midpt_dis)
    
    x = []
    y = []
    for i in range(len(distances)):
        x.append(distances[i] * math.cos(angles[i]))
        y.append(distances[i] * math.sin(angles[i]))

    midx = [0, midpt_dis * math.cos(midpt_ang)]
    midy = [0, midpt_dis * math.sin(midpt_ang)]
    print(midx)
    print(midy)
    plt.plot(x, y, 'oc')

    plt.plot(midx, midy, 'oc', c = "r")
    bottom, top = plt.ylim()  # return the current y-lim
    plt.ylim((top, bottom))  # rescale y axis, to match the grid orientation
    plt.grid(True)
    plt.show()


if __name__ == '__main__':
    main()
