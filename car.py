# imports
import math
import matplotlib.pyplot as plt
from math import cos, sin, radians
from point import ScaledPoint


# constants
dt = 0.01
D_ACC_WEIGHT = 50
P_ACC_WEIGHT = 5


# --------------------------------- utility functions -------------------------------- #

# returns a unit vector perpendicular to the input vector
def get_orthogonal_unit_vector(vector):   # vector is an instance of scaled point class
    if vector.get_y() == 0:
        return ScaledPoint(0, 1)
    v1 = 5
    # using the fact that dot product of perpendicular vectors is 0
    v2 = -vector.get_x() / vector.get_y() * v1
    magnitude = math.sqrt(v1**2+v2**2)
    v2 /= magnitude
    v1 /= magnitude
    return ScaledPoint(v1, v2)


def print_points(points):
    for x in points:
        x.print()


def get_vector(direction, magnitude):
    angle = radians(direction)  # angle the velocity vector makes with x-axis
    vector_direction = ScaledPoint(cos(angle), sin(angle))  # unit vector in the direction of velocity
    return vector_direction * magnitude


def get_avg_centroid(vehicles):
    avg_centroid = ScaledPoint(0, 0)
    for v_ in vehicles:
        avg_centroid += v_.centroid
    return avg_centroid/len(vehicles)


def point_above_line(point, line_start, line_end):
    return point[1] > line_start[1]


def point_lies_left(point, line_start, line_end):
    dx = line_end[0] - line_start[0]
    dy = line_end[1] - line_start[1]
    dx1 = point[0] - line_start[0]
    dy1 = point[1] - line_start[1]
    cross_product = dx * dy1 - dy * dx1
    return cross_product > 0 or cross_product >= 0


class Vehicle:
    theta = 45

    def __init__(self, length, width, speed_limit, acc_limit, centroid, angle, v, a, error=ScaledPoint(0, 0),
                 current_face=None):
        # parameters
        self.length = length
        self.width = width
        self.speed_limit = 80
        self.acc_limit = 45
        self.theta = -90
        # state variables
        self.centroid = centroid
        self.angle = angle
        self.velocity = v
        self.speed = 0
        self.acc = a
        self.integral_acc = 0
        self.acc_magnitude = 0
        self.max_force = 0.5
        self.error_point = error
        self.error = 0
        self.prev_error = 0
        self.current_face = current_face
        self.prev_face = None
        self.face_mid_point = None

    # uses centroid and angle of vehicle to return its 4 points
    def state_to_corners(self):
        dir_vect = ScaledPoint(cos(self.angle), sin(self.angle))
        dir_vect = dir_vect * (self.length / 2)
        orthogonal_vect = get_orthogonal_unit_vector(dir_vect)
        orthogonal_vect = ScaledPoint(orthogonal_vect.get_x() * self.width / 2, orthogonal_vect.get_y() * self.width / 2)
        return [
            self.centroid + dir_vect + orthogonal_vect,
            self.centroid + dir_vect - orthogonal_vect,
            self.centroid - dir_vect - orthogonal_vect,
            self.centroid - dir_vect + orthogonal_vect,
        ]

    def shortest_distance(self, vector):
        veh1 = self.state_to_corners()
        veh2 = vector.state_to_corners()
        minimum = 99999999
        for i in veh1:
            for j in veh2:
                dist = math.dist([i.x, i.y], [j.x, j.y])
                if dist < minimum:
                    minimum = dist
        return minimum

    def closest_points(self, vector):
        veh1 = self.state_to_corners()
        veh2 = vector.state_to_corners()
        minimum = float('-inf')
        v1 = 0
        v2 = 0
        for i in veh1:
            for j in veh2:
                dist = math.dist([i.x, i.y], [j.x, j.y])
                if dist < minimum:
                    minimum = dist
                    v1 = i
                    v2 = j
        return v1, v2

    def controller(self):
        if self.face_mid_point is None:
            return self.acc

        p_acc_magnitude = self.error
        d_acc_magnitude = self.error - self.prev_error
        self.integral_acc = self.integral_acc + p_acc_magnitude
        self.acc_magnitude = (P_ACC_WEIGHT*p_acc_magnitude) + (D_ACC_WEIGHT * d_acc_magnitude)
        self.acc = get_vector(self.theta + (self.angle * 180 / math.pi), self.acc_magnitude)
        return self.acc

    def update_state_vars(self):
        self.controller()
        if self.acc.norm() > self.acc_limit:
            self.acc /= self.acc.norm()
            self.acc *= self.acc_limit
        self.velocity += (self.acc * dt)
        self.speed = self.velocity.norm()
        if self.velocity.norm() > self.speed_limit:
            self.velocity /= self.velocity.norm()
            self.velocity *= self.speed_limit
        self.centroid += self.velocity * dt
        self.angle = math.atan2(self.velocity.get_y(), self.velocity.get_x())

    def get_xy_lists(self):
        p = self.state_to_corners()
        x, y = [p[i].get_x() for i in range(4)], [p[i].get_y() for i in range(4)]
        x.append(p[0].get_x())
        y.append(p[0].get_y())
        return x, y

    def draw_vehicle(self):
        p = self.get_xy_lists()
        x, y = p[0], p[1]
        plt.plot(x, y)

    def get_car_mid_point(self):
        segment = self.get_car_perpendicular_line()
        vehicle_x = (segment[0][0] + segment[1][0]) / 2
        vehicle_y = (segment[0][1] + segment[1][1]) / 2
        return vehicle_x, vehicle_y

    def get_car_perpendicular_line(self):
        x, y = self.get_xy_lists()
        x1, x2 = x[0], x[1]
        y1, y2 = y[0], y[1]

        return [(x1, y1), (x2, y2)]

