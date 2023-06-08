# imports
from math import radians
import math
import matplotlib.pyplot as plt
import geometry
from math import cos, sin, radians
from point import ScaledPoint
from shapely import Point, LineString
from shapely.ops import nearest_points


# constants
params = {
    "dt": 0.01,
    "D_PER_ACC_WEIGHT": 20,
    "P_PER_ACC_WEIGHT": 10,
    "P_PARALLEL_DEC_WEIGHT": 10,
    "P_PARALLEL_ACC_WEIGHT": 0.2,
    "angle_threshold": 5
}


class Vehicle:

    def __init__(self, length, width, centroid, angle, velocity, acceleration, reference_track=None):

        # parameters
        self.lookahead_distance = 10
        self.angle_threshold = params["angle_threshold"]
        self.initial_speed = velocity.norm()
        self.length = length
        self.width = width
        self.speed_limit = 10
        self.acc_limit = 2.65
        self.goal_speed = 8
        self.theta = -90
        self.reference_track = reference_track

        # state variables
        self.centroid = centroid
        self.front_mid_point = None
        self.lookahead_point = None
        self.closest_point = None
        self.angle = angle
        self.velocity = velocity
        self.perpendicular_acc = acceleration
        self.parallel_acc = geometry.get_vector(0, 0)
        self.parallel_acc_sign = 1
        self.perpendicular_acc_sign = 1
        self.lookahead_angle = 0
        self.error = 0
        self.prev_error = 0
        self.current_face = None
        self.face_mid_point = None

    def state_to_corners(self):
        """Returns car's four points using centroid and vehicle orientation"""

        # dir_vector starts from vehicle centroid and ends at the mid-point of the front line of vehicle
        dir_vect = ScaledPoint(cos(self.angle), sin(self.angle))
        dir_vect = dir_vect * (self.length / 2)

        # orthogonal_vector starts from vehicle centroid and ends at the mid-point of the side line of vehicle
        orthogonal_vector = geometry.get_orthogonal_unit_vector(dir_vect)
        orthogonal_vector = ScaledPoint(orthogonal_vector.get_x() * self.width / 2,
                                        orthogonal_vector.get_y() * self.width / 2)

        return [
            self.centroid + dir_vect + orthogonal_vector,
            self.centroid + dir_vect - orthogonal_vector,
            self.centroid - dir_vect - orthogonal_vector,
            self.centroid - dir_vect + orthogonal_vector,
        ]

    def set_perpendicular_acc(self):
        proportional_error = self.error
        derivative_error = self.error - self.prev_error

        perpendicular_acc_magnitude = (params["P_PER_ACC_WEIGHT"] * proportional_error) + \
                                    (params["D_PER_ACC_WEIGHT"] * derivative_error)
        self.perpendicular_acc_sign = -1 if perpendicular_acc_magnitude < 0 else 1

        self.perpendicular_acc = geometry.get_vector(self.theta + (self.angle * 180 / math.pi), perpendicular_acc_magnitude)
        self.perpendicular_acc = geometry.keep_in_limit(self.perpendicular_acc, self.acc_limit)

        if self.velocity.norm() >= self.goal_speed:
            self.perpendicular_acc = geometry.get_vector(0, 0)

    def set_parallel_acc(self):
        if abs(self.lookahead_angle) > radians(params["angle_threshold"]):
            dec_magnitude = (self.lookahead_angle - radians(params["angle_threshold"])) * (-params["P_PARALLEL_DEC_WEIGHT"])
            self.parallel_acc_sign = -1 if dec_magnitude * (-params["P_PARALLEL_DEC_WEIGHT"]) < 0 else 1
            self.parallel_acc = (self.velocity / self.velocity.norm()) * dec_magnitude
        else:
            acc_magnitude = params["P_PARALLEL_ACC_WEIGHT"] * (self.goal_speed - self.velocity.norm())
            self.parallel_acc = (self.velocity / self.velocity.norm()) * acc_magnitude
            self.parallel_acc_sign = -1 if acc_magnitude < 0 else 1

        self.parallel_acc = geometry.keep_in_limit(self.parallel_acc, self.acc_limit)

    def controller(self):
        self.set_perpendicular_acc()
        self.set_parallel_acc()
        resultant_acc = self.perpendicular_acc + self.parallel_acc
        return geometry.keep_in_limit(resultant_acc, self.acc_limit)

    def set_velocity(self):
        acc = self.controller()
        self.velocity += (acc * params["dt"])
        self.velocity = geometry.keep_in_limit(self.velocity, self.speed_limit)

    def set_centroid(self):
        self.centroid += self.velocity * params["dt"]

    def set_closest_point(self):
        projected_point_distance = self.reference_track.project(self.front_mid_point)
        self.closest_point = self.reference_track.interpolate(projected_point_distance)

    def set_lookahead_point(self):
        """ Creates a circle with center at car centroid and radius equal to lookahead distance.
            Finds the intersection points of the circle and track.
            Returns the intersection point closest to the car's velocity vector
        """
        circle_center = Point(self.centroid.get_x(), self.centroid.get_y())
        circle_radius = self.lookahead_distance
        vel_vector = Point(self.velocity.get_x(), self.velocity.get_y())
        circle = circle_center.buffer(circle_radius)  # Create a circle geometry
        boundary = circle.boundary  # Get the boundary of the circle
        intersection = boundary.intersection(self.reference_track)
        if intersection.is_empty:
            return None
        # Translate the velocity vector to car centroid
        vel_vector = Point(vel_vector.x + circle_center.x, vel_vector.y + circle_center.y)
        # If multiple intersection points exist, find the one closest to the velocity vector
        if intersection.geom_type == 'MultiPoint':
            intersection = nearest_points(vel_vector, intersection)[1]
        self.lookahead_point = intersection

    def set_orientation(self):
        self.angle = math.atan2(self.velocity.get_y(), self.velocity.get_x())

    def set_prev_error(self):
        self.prev_error = self.error

    def update_state_vars(self):
        self.set_vehicle_front_mid_point()
        self.set_closest_point()
        self.set_lookahead_point()
        self.set_prev_error()
        self.set_error()
        self.set_velocity()
        self.set_centroid()
        self.set_orientation()

    def get_xy_lists(self):
        p = self.state_to_corners()
        x, y = [p[i].get_x() for i in range(4)], [p[i].get_y() for i in range(4)]
        x.append(p[0].get_x())
        y.append(p[0].get_y())
        return x, y

    def get_car_perpendicular_line(self):
        x, y = self.get_xy_lists()
        x1, x2 = x[0], x[1]
        y1, y2 = y[0], y[1]

        return [(x1, y1), (x2, y2)]

    def set_vehicle_front_mid_point(self):
        """
        :return: A Point object representing the mid-point of the front line of the vehicle.
        """
        segment = self.get_car_perpendicular_line()
        self.front_mid_point = Point((segment[0][0] + segment[1][0]) / 2, (segment[0][1] + segment[1][1]) / 2)

    def get_lookahead_angle_magnitude(self):
        """Returns lookahead angle magnitude in radians"""
        headed_direction = (self.front_mid_point, Point(self.velocity.x + self.front_mid_point.x, self.velocity.y + self.front_mid_point.y))
        track_direction = (self.front_mid_point, self.lookahead_point)

        vector1 = Point(headed_direction[1].x - headed_direction[0].x, headed_direction[1].y - headed_direction[0].y)
        vector2 = Point(track_direction[1].x - track_direction[0].x, track_direction[1].y - track_direction[0].y)

        dot_product = vector1.x * vector2.x + vector1.y * vector2.y

        magnitude1 = math.sqrt(vector1.x ** 2 + vector1.y ** 2)
        magnitude2 = math.sqrt(vector2.x ** 2 + vector2.y ** 2)

        return math.acos(dot_product / (magnitude1 * magnitude2))

    def get_error_sign(self):
        segment_point_1 = Point(self.front_mid_point.x, self.front_mid_point.y)
        segment_point_2 = Point(self.velocity.get_x() + self.front_mid_point.x, self.velocity.get_y() + self.front_mid_point.y)

        # translating the points to origin
        lookahead_point = Point(self.lookahead_point.x - segment_point_1.x, self.lookahead_point.y - segment_point_1.y)
        segment_point_2 = Point(segment_point_2.x - segment_point_1.x, segment_point_2.y - segment_point_1.y)

        cross_product = segment_point_2.x * lookahead_point.y - segment_point_2.y * lookahead_point.x

        return -1 if cross_product > 0 else 1

    def set_error(self):
        self.lookahead_angle = self.get_lookahead_angle_magnitude()
        self.error = self.lookahead_angle * self.get_error_sign()

    def set_current_face(self, road_network):
        vehicle_x, vehicle_y = self.front_mid_point.x, self.front_mid_point.y
        self.current_face = road_network.get_face_for_point((vehicle_x, vehicle_y))

    def get_vectors(self):
        vehicle_point = (self.front_mid_point.x, self.front_mid_point.y)
        headed_direction = (
            vehicle_point, (self.velocity.get_x() + vehicle_point[0], self.velocity.get_y() + vehicle_point[1]))
        lookahead_point = (self.front_mid_point.x, self.front_mid_point.y)
        track_direction = (vehicle_point, lookahead_point)
        return headed_direction, track_direction
