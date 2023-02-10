import math
import matplotlib.pyplot as plt
from math import cos, sin, radians
import pygame
dt = 0.01
theta = 15


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, p):
        return Point(self.x + p.x, self.y + p.y)

    def __sub__(self, p):
        return Point(self.x - p.x, self.y - p.y)

    def __mul__(self, s):
        return Point(self.x * s, self.y * s)

    def __truediv__(self, s):
        return Point(self.x / s, self.y / s) if s != 0 else self

    def __repr__(self):
        return f"({str(self.x)},{str(self.y)})"

    def norm(self):
        return math.sqrt(self.x**2+self.y**2)


# utility functions
# returns a unit vector perpendicular to the input vector V
def get_orth_unit_vector(vector):   # V is an instance of point class
    if vector.y == 0:
        return Point(0, 1)
    v1 = 5
    # using the fact that dot product of perpendicular vectors is 0
    v2 = -vector.x / vector.y * v1
    magnitude = math.sqrt(v1**2+v2**2)
    v2 = v2/magnitude
    v1 = v1/magnitude
    return Point(v1, v2)


def print_points(points):
    for x in points:
        x.print()


def get_vector(direction, magnitude):
    vel_angle = radians(direction)  # angle the velocity vector makes with x axis
    vel_dir = Point(cos(vel_angle), sin(vel_angle))  # unit vector in the direction of velocity
    curr_vel = vel_dir * magnitude
    return curr_vel


def get_avg_centroid(vehicles):
    avg_centroid = Point(0, 0)
    for v_ in vehicles:
        avg_centroid += v_.centroid
    return avg_centroid/len(vehicles)


class Vehicle:
    def __init__(self, length, width, speed_limit, acc_limit, centroid, angle, v, a, error=Point(0, 0)):
        # parameters
        self.length = length
        self.width = width
        self.speed_limit = speed_limit
        self.acc_limit = acc_limit
        # state variables
        self.centroid = centroid
        self.angle = angle
        self.velocity = v
        self.acc = a
        self.max_force = 0.5
        self.error_point = error
        self.error = 0

    # uses centroid and angle of vehicle to return its 4 points
    def state_to_corners(self):
        dir_vect = Point(cos(self.angle), sin(self.angle))
        dir_vect = dir_vect * (self.length / 2)
        orth_vect = get_orth_unit_vector(dir_vect)
        orth_vect = Point(orth_vect.x * self.width / 2, orth_vect.y * self.width / 2)
        return [
            self.centroid + dir_vect + orth_vect,
            self.centroid + dir_vect - orth_vect,
            self.centroid - dir_vect - orth_vect,
            self.centroid - dir_vect + orth_vect,
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
        self.acc = get_vector((theta + (self.angle * 180 / math.pi)), self.error)
        return self.acc
        # return Point(0, 0)

    def update_state_vars(self):
        self.velocity += (self.controller() * dt)
        self.centroid += self.velocity * dt
        self.angle = math.atan2(self.velocity.y, self.velocity.x)
        # print("hi")

    def get_xy_lists(self):
        p = self.state_to_corners()
        x, y = [p[i].x for i in range(4)], [p[i].y for i in range(4)]
        x.append(p[0].x)
        y.append(p[0].y)
        return x, y
