import math
import matplotlib.pyplot as plt
from math import cos, sin, radians
import numpy as np
dt = 0.01


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
def get_orthogonal_unit_vector(vector):   # V is an instance of point class
    if vector.y == 0:
        return Point(0, 1)
    v1 = 5
    # using the fact that dot product of perpendicular vectors is 0
    v2 = -vector.x / vector.y * v1
    magnitude = math.sqrt(v1**2+v2**2)
    v2 /= magnitude
    v1 /= magnitude
    return Point(v1, v2)


def print_points(points):
    for x in points:
        x.print()


def get_vector(direction, magnitude):
    angle = radians(direction)  # angle the velocity vector makes with x-axis
    vector_direction = Point(cos(angle), sin(angle))  # unit vector in the direction of velocity
    return vector_direction * magnitude


def get_avg_centroid(vehicles):
    avg_centroid = Point(0, 0)
    for v_ in vehicles:
        avg_centroid += v_.centroid
    return avg_centroid/len(vehicles)


def point_above_line(point, line_start, line_end):
    # Calculate the vector from the line start to the point
    p1 = np.array(line_start)
    p2 = np.array(point)
    p3 = np.array(line_end)
    v1 = p2 - p1

    # Calculate the vector from the line start to the line end
    v2 = p3 - p1

    # Calculate the cross product
    cross_product = np.cross(v1, v2)
    # If the cross product is positive, the point is above the line
    return cross_product > 0


class Vehicle:
    theta = 45

    def __init__(self, length, width, speed_limit, acc_limit, centroid, angle, v, a, error=Point(0, 0),
                 current_face=None):
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
        self.current_face = current_face
        self.face_mid_point = None

    # uses centroid and angle of vehicle to return its 4 points
    def state_to_corners(self):
        dir_vect = Point(cos(self.angle), sin(self.angle))
        dir_vect = dir_vect * (self.length / 2)
        orthogonal_vect = get_orthogonal_unit_vector(dir_vect)
        orthogonal_vect = Point(orthogonal_vect.x * self.width / 2, orthogonal_vect.y * self.width / 2)
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
        vehicle_x, vehicle_y = self.get_car_mid_point()
        line_start = (vehicle_x, vehicle_y)
        line_end = (self.velocity.x + vehicle_x, self.velocity.y + vehicle_y)
        if self.face_mid_point is None:
            return self.acc
        if point_above_line(self.face_mid_point, line_start, line_end): # agar mid_point hai oper
            self.theta = abs(self.theta)  # angle positive
        else:  # agar mid_point nechay hai
            self.theta = -1*abs(self.theta) # angle negative
        self.acc = get_vector(-1 * (self.theta + (self.angle * 180 / math.pi)), self.error)
        return self.acc

    def update_state_vars(self):
        self.velocity += (self.controller() * dt)
        self.centroid += self.velocity * dt
        self.angle = math.atan2(self.velocity.y, self.velocity.x)

    def get_xy_lists(self):
        p = self.state_to_corners()
        x, y = [p[i].x for i in range(4)], [p[i].y for i in range(4)]
        x.append(p[0].x)
        y.append(p[0].y)
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
