import math
import matplotlib.pyplot as plt
from math import cos, sin, radians
from point import ScaledPoint
dt = 0.01

road_threshold = 10


# utility functions
# returns a unit vector perpendicular to the input vector V
def get_orthogonal_unit_vector(vector):   # V is an instance of point class
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


def point_line_side(point, line_start, line_end):
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
        self.acc_limit = 40
        # state variables
        self.centroid = centroid
        self.angle = angle
        self.velocity = v
        self.acc = a
        self.integral_acc = ScaledPoint(0, 0)
        self.max_force = 0.5
        self.error_point = error
        self.error = 0
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

    def controller(self, dist, dist2):
        self.theta = 45
        if self.face_mid_point is None:
            return self.acc
        vehicle_x, vehicle_y = self.get_car_mid_point()
        line_start = (vehicle_x, vehicle_y)
        line_end = (self.velocity.get_x() + vehicle_x, self.velocity.get_y() + vehicle_y)
        if point_line_side(self.face_mid_point, line_start, line_end):
            self.theta = abs(self.theta)
        else:
            self.theta = -1*abs(self.theta)

        theta_weight = 1
        if dist < road_threshold or dist2 < road_threshold:
            # theta_weight = 1.5
            theta_weight *= self.error
            self.theta *= theta_weight
            if self.theta > 90:
                self.theta = 90
            elif self.theta < -90:
                self.theta = -90

        self.acc = get_vector((self.theta + (self.angle * 180 / math.pi)), self.error)
        self.integral_acc = self.integral_acc + self.acc
        self.acc = self.acc  # + self.integral_acc
        return self.acc

    def update_state_vars(self, dist, dist2):
        self.controller(dist, dist2)
        if self.acc.norm() > self.acc_limit:
            self.acc /= self.acc.norm()
            self.acc *= self.acc_limit
        self.velocity += (self.acc * dt)
        # self.velocity *= 2
        if self.velocity.norm() > self.speed_limit:
            self.velocity /= self.velocity.norm()
            self.velocity *= self.speed_limit
        self.centroid += self.velocity * dt
        self.angle = math.atan2(self.velocity.get_y(), self.velocity.get_x())
        # self.velocity = get_vector(math.atan2(self.velocity.get_y(), self.velocity.get_x()), 20)

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

