import math
from turtle import width

class FrontRectPerceptor:
    def __init__(self,
                percept_width,
                start_gap):
        self.width = percept_width
        self.start_gap = start_gap
        self.max_angle = int(math.degrees(math.atan(self.width / self.start_gap)))
        self.max_dists_set =  [self.width / 2 / math.sin(math.radians(i))  for i in range(self.max_angle, 0, -1)]
        self.max_dists_set += [self.width / 2 / math.sin(math.radians(i))  for i in range(1, self.max_angle, 1) ]
        self.starting_angle = 90 - self.max_angle
        self.ending_angle = 90 + self.max_angle

    def percept_all_obstacles(self, angles, dists, steering_angle):
        obstacle_angles = []
        obstacle_dists = []
        for i in range(self.max_angle * 2 - 1):
            _angle = self.starting_angle + i
            _dist  = dists[_angle] + abs(90 - _angle) * math.cos(math.radians(abs(90 - _angle))) * 0.05 * abs(steering_angle)
            if _dist > self.max_dists_set[i]:
                continue
            else:
                obstacle_angles.append(_angle)
                obstacle_dists.append(_dist)
        return obstacle_angles, obstacle_dists

    def percept_nearest_obstacle(self, dists, steering_angle):
        min_angle = -1
        max_dist  = 0
        for j in range(self.max_angle * 2 - 1):
            _angle = self.starting_angle + j
            _dist  = dists[_angle] + abs(90 - _angle) * math.cos(math.radians(abs(90 - _angle))) * 0.05 * abs(steering_angle)
            if _dist > self.max_dists_set[j]:
                continue
            elif min_angle == -1:
                min_angle = _angle
            elif _dist < dists[min_angle]:
                min_angle = _angle
        if min_angle == -1:
            return -1, self.max_dists_set[self.max_angle]
        return min_angle, dists[min_angle]

    def percept_nearest_dist(self, angles, dists):
        min_dist = dists[self.starting_angle]
        for j in range(0, self.max_angle * 2):
            _angle = self.starting_angle + j
            _dist  = dists[_angle]
            if _dist > self.max_dists_set[j]:
                continue
            elif min_dist == -1:
                min_dist = _dist
                continue
            elif _dist < dists[min_angle]:
                min_angle = _angle
        return min_angle, dists[min_angle] if min_angle != -1 else -1, -1       

        
