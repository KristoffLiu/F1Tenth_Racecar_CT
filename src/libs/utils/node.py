class Node:
    def __init__(self, x_ind, y_ind,
                 x_list, y_list, directions, parent_index=None, cost=None):
        self.x_index = x_ind
        self.y_index = y_ind
        self.x_list = x_list
        self.y_list = y_list
        self.parent_index = parent_index
        self.cost = cost

class VectorNode(Node):
    def __init__(self, x_ind, y_ind, yaw_ind, direction,
                 x_list, y_list, yaw_list, directions,
                 steer=0.0, parent_index=None, cost=None):
        Node.__init__(x_ind, y_ind,
                 x_list, y_list, parent_index, cost)
        self.yaw_index = yaw_ind
        self.direction = direction
        self.yaw_list = yaw_list
        self.directions = directions
        self.steer = steer