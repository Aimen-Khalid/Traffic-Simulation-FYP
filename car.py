# imports
import math
import matplotlib.pyplot as plt
from math import cos, sin, radians
from point import ScaledPoint
from shapely import Point, LineString

# constants
dt = 0.01
D_ACC_WEIGHT = 50
P_ACC_WEIGHT = 10


# --------------------------------- utility functions -------------------------------- #

# returns a unit vector perpendicular to the input vector
def get_orthogonal_unit_vector(vector):   # vector is an instance of scaled point class
    if vector.get_y() == 0:
        return ScaledPoint(0, 1)
    # choose an arbitrary value for one of the coordinates of the orthogonal vector
    x1 = 5
    # using the fact that dot product of perpendicular vectors is 0 ie x1*x2 + y1*y2 = 0
    y1 = -vector.get_x() / vector.get_y() * x1
    magnitude = math.sqrt(x1**2+y1**2)
    # normalizing the vector
    y1 /= magnitude
    x1 /= magnitude
    return ScaledPoint(x1, y1)


def get_vector(angle, magnitude):
    angle = radians(angle)
    vector_direction = ScaledPoint(cos(angle), sin(angle))  # unit vector with the given angle
    return vector_direction * magnitude

# ------------------------------------------------------------------------------------------------


class Vehicle:

    def __init__(self, length, width, centroid, angle, velocity, acceleration):

        # parameters
        self.length = length
        self.width = width
        self.speed_limit = 80
        self.acc_limit = 45
        self.theta = -90
        self.reference_track = LineString([(300, 55), (635, 55), (770, 220), (770, 500), (620, 620), (300, 620),
                                           (135, 500), (135, 230), (300, 55)])
        # self.reference_track = LineString([(100, 100), (100, 250), (200, 300), (230, 220), (250, 250), (300, 300),
        #                                    (360, 300), (440, 230), (440, 110), (500, 150), (610, 115)])

        # state variables
        self.centroid = centroid
        self.angle = angle
        self.velocity = velocity
        self.acc = acceleration
        self.integral_acc = 0
        self.acc_magnitude = 0
        self.error = 0
        self.prev_error = 0
        self.current_face = None
        self.prev_face = None
        self.face_mid_point = None

    # uses centroid and angle of vehicle to return its 4 points
    def state_to_corners(self):
        # dir_vector starts from vehicle centroid and ends at the mid-point of the front line of vehicle
        dir_vect = ScaledPoint(cos(self.angle), sin(self.angle))
        dir_vect = dir_vect * (self.length / 2)

        # orthogonal_vector starts from vehicle centroid and ends at the mid-point of the side line of vehicle
        orthogonal_vector = get_orthogonal_unit_vector(dir_vect)
        orthogonal_vector = ScaledPoint(orthogonal_vector.get_x() * self.width / 2, orthogonal_vector.get_y() * self.width / 2)

        return [
            self.centroid + dir_vect + orthogonal_vector,
            self.centroid + dir_vect - orthogonal_vector,
            self.centroid - dir_vect - orthogonal_vector,
            self.centroid - dir_vect + orthogonal_vector,
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
        # self.speed = self.velocity.norm()
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

