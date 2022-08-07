import os
import numpy as np

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

def getvirtuallidarmap(lidarmap):
    return file_read(os.path.abspath(__file__).replace("virtuallidarmap.py","") + "/" + lidarmap + ".csv")