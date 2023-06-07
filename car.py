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
# dt = 0.01
# D_PER_ACC_WEIGHT = 20
# P_PER_ACC_WEIGHT = 10
# P_PARALLEL_DEC_WEIGHT = 10
# P_PARALLEL_ACC_WEIGHT = 0.2
# angle_threshold = 5


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
        self.angle = angle
        self.velocity = velocity
        self.perpendicular_acc = acceleration
        self.parallel_acc = geometry.get_vector(0, 0)
        self.lookahead_angle = 0
        self.error = 0
        self.prev_error = 0
        self.current_face = None
        self.prev_face = None
        self.face_mid_point = None

    def state_to_corners(self):
        """uses centroid and angle of vehicle to return its 4 points"""

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

        perp_acc_magnitude = (params["P_PER_ACC_WEIGHT"] * proportional_error) + (params["D_PER_ACC_WEIGHT"] * derivative_error)

        self.perpendicular_acc = geometry.get_vector(self.theta + (self.angle * 180 / math.pi),
                                                               perp_acc_magnitude)
        self.perpendicular_acc = geometry.keep_in_limit(self.perpendicular_acc, self.acc_limit)

        if self.velocity.norm() >= self.goal_speed:
            self.perpendicular_acc = geometry.get_vector(0, 0)

    def set_parallel_acc(self):
        if abs(self.lookahead_angle) > radians(params["angle_threshold"]):
            dec_magnitude = self.lookahead_angle - radians(params["angle_threshold"])
            self.parallel_acc = (self.velocity / self.velocity.norm()) * dec_magnitude * (-params["P_PARALLEL_DEC_WEIGHT"])
        else:
            self.parallel_acc = (self.velocity / self.velocity.norm()) * (
                    params["P_PARALLEL_ACC_WEIGHT"] * (self.goal_speed - self.velocity.norm()))
        self.parallel_acc = geometry.keep_in_limit(self.parallel_acc, self.acc_limit)

    def controller(self):
        self.set_perpendicular_acc()
        self.set_parallel_acc()
        resultant_acc = self.perpendicular_acc + self.parallel_acc
        return geometry.keep_in_limit(resultant_acc, self.acc_limit)

    def update_state_vars(self):
        self.error = self.get_error_ang_disp()
        acc = self.controller()
        self.velocity += (acc * params["dt"])
        self.velocity = geometry.keep_in_limit(self.velocity, self.speed_limit)
        self.centroid += self.velocity * params["dt"]
        self.angle = math.atan2(self.velocity.get_y(), self.velocity.get_x())

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

    def get_vehicle_front_mid_point(self):
        """
        :return: A Point object representing the mid-point of the front line of the vehicle.
        """
        segment = self.get_car_perpendicular_line()
        return Point((segment[0][0] + segment[1][0]) / 2, (segment[0][1] + segment[1][1]) / 2)

    def get_projected_points(self):
        vehicle_point = self.get_vehicle_front_mid_point()
        projected_point_distance = self.reference_track.project(vehicle_point)
        closest_point = self.reference_track.interpolate(projected_point_distance)
        lookahead_point = self.reference_track.interpolate(projected_point_distance + self.lookahead_distance)
        # return {"closest_point": closest_point, "lookahead_point": lookahead_point}
        return {"closest_point": closest_point, "lookahead_point": self.get_lookahead_point()}

    def get_lookahead_point(self):
        circle_center = Point(self.centroid.get_x(), self.centroid.get_y())
        circle_radius = self.lookahead_distance
        vel_vector = Point(self.velocity.get_x(), self.velocity.get_y())
        circle = Point(circle_center).buffer(circle_radius)  # Create a circle geometry
        boundary = circle.boundary  # Get the boundary of the circle
        intersection = boundary.intersection(self.reference_track)  # Find the intersection points
        if intersection.is_empty:
            return None  # No intersection point
        vel_vector = Point(vel_vector.x + circle_center.x, vel_vector.y + circle_center.y)
        # If multiple intersection points exist, find the one closest to the circle center
        if intersection.geom_type == 'MultiPoint':
            intersection = nearest_points(vel_vector, intersection)[1]

        return intersection

    def get_track_segment(self):
        """
        :return: A tuple of two Point objects representing the start and end points of the track segment
                 on which the look-ahead point lies
        """
        lookahead_point = self.get_projected_points()["lookahead_point"]

        # Iterate over all the line segments of the reference track and find the segment closest to the projected point
        coordinates = self.reference_track.coords
        for i in range(len(coordinates) - 1):
            line_segment = LineString([coordinates[i], coordinates[i + 1]])
            if line_segment.distance(lookahead_point) < 1:
                # If the projected point is within 1 unit of distance from the line segment, return the start and end points
                return Point(coordinates[i][0], coordinates[i][1]), Point(coordinates[i + 1][0], coordinates[i + 1][1])

    def get_error_from_track(self):
        """
        :return: The signed error, positive if the vehicle is to the right of the track and negative if it is to the left.
        """
        vehicle_point = self.get_vehicle_front_mid_point()
        segment_point_1, segment_point_2 = self.get_track_segment()

        error = vehicle_point.distance(self.reference_track)

        # translating the points to origin
        vehicle_point = Point(vehicle_point.x - segment_point_1.x, vehicle_point.y - segment_point_1.y)
        segment_point_2 = Point(segment_point_2.x - segment_point_1.x, segment_point_2.y - segment_point_1.y)

        cross_product = segment_point_2.x * vehicle_point.y - segment_point_2.y * vehicle_point.x

        return error * (-1 if cross_product < 0 else 1)

    def get_error_ang_disp(self):
        vehicle_point = (self.get_vehicle_front_mid_point().x, self.get_vehicle_front_mid_point().y)
        headed_direction = (
            vehicle_point, (self.velocity.get_x() + vehicle_point[0], self.velocity.get_y() + vehicle_point[1]))
        # print(headed_direction)
        lookahead_point = (self.get_projected_points()["lookahead_point"].x,
                           self.get_projected_points()["lookahead_point"].y)
        track_direction = (vehicle_point, lookahead_point)
        # print(track_direction)
        # Calculate the vectors of the two line segments
        vector1 = (headed_direction[1][0] - headed_direction[0][0], headed_direction[1][1] - headed_direction[0][1])
        vector2 = (track_direction[1][0] - track_direction[0][0], track_direction[1][1] - track_direction[0][1])

        # Calculate the dot product of the two vectors
        dot_product = vector1[0] * vector2[0] + vector1[1] * vector2[1]

        # Calculate the magnitudes of the vectors
        magnitude1 = math.sqrt(vector1[0] ** 2 + vector1[1] ** 2)
        magnitude2 = math.sqrt(vector2[0] ** 2 + vector2[1] ** 2)

        # Calculate the angle in radians
        self.lookahead_angle = math.acos(dot_product / (magnitude1 * magnitude2))

        lookahead_point = Point(lookahead_point[0], lookahead_point[1])
        segment_point_1, segment_point_2 = Point(vehicle_point[0], vehicle_point[1]), \
            Point(self.velocity.get_x() + vehicle_point[0], self.velocity.get_y() + vehicle_point[1])

        # translating the points to origin
        lookahead_point = Point(lookahead_point.x - segment_point_1.x, lookahead_point.y - segment_point_1.y)
        segment_point_2 = Point(segment_point_2.x - segment_point_1.x, segment_point_2.y - segment_point_1.y)

        cross_product = segment_point_2.x * lookahead_point.y - segment_point_2.y * lookahead_point.x

        return self.lookahead_angle * (-1 if cross_product > 0 else 1)

    def get_vectors(self):
        vehicle_point = (self.get_vehicle_front_mid_point().x, self.get_vehicle_front_mid_point().y)
        headed_direction = (
            vehicle_point, (self.velocity.get_x() + vehicle_point[0], self.velocity.get_y() + vehicle_point[1]))
        lookahead_point = (
            self.get_projected_points()["lookahead_point"].x, self.get_projected_points()["lookahead_point"].y)
        track_direction = (vehicle_point, lookahead_point)
        return headed_direction, track_direction

    def set_current_face(self, road_network):
        """
        A utility function for compute_parameters function

        :param road_network: The road network DCEL object
        :param self: The vehicle whose current face needs to be set.
        """
        vehicle_x, vehicle_y = self.get_vehicle_front_mid_point().x, self.get_vehicle_front_mid_point().y
        self.current_face = road_network.get_face_for_point((vehicle_x, vehicle_y))

        if self.current_face is None:
            self.current_face = self.prev_face
        if self.current_face is not None:
            self.prev_face = self.current_face
