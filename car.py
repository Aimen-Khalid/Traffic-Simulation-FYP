# imports
from math import radians
import math
import matplotlib.pyplot as plt
import geometry_functions
from math import cos, sin, radians
from point import ScaledPoint
from shapely import Point, LineString

# constants
dt = 0.01
D_PER_ACC_WEIGHT = 20
P_PER_ACC_WEIGHT = 10
P_PARALLEL_DEC_WEIGHT = 10
P_PARALLEL_ACC_WEIGHT = 0.2
angle_threshold = 5

# --------------------------------- utility functions -------------------------------- #


# returns a unit vector perpendicular to the input vector
def get_orthogonal_unit_vector(vector):  # vector is an instance of scaled point class
    if vector.get_y() == 0:
        return ScaledPoint(0, 1)
    # choose an arbitrary value for one of the coordinates of the orthogonal vector
    x1 = 5
    # using the fact that dot product of perpendicular vectors is 0 ie x1*x2 + y1*y2 = 0
    y1 = -vector.get_x() / vector.get_y() * x1
    magnitude = math.sqrt(x1 ** 2 + y1 ** 2)
    # normalizing the vector
    y1 /= magnitude
    x1 /= magnitude
    return ScaledPoint(x1, y1)


def get_vector(angle, magnitude):
    angle = radians(angle)
    vector_direction = ScaledPoint(cos(angle), sin(angle))  # unit vector with the given angle
    return vector_direction * magnitude


def keep_in_limit(vector, limit):
    if vector.norm() > limit:
        vector /= vector.norm()
        vector *= limit
    return vector


# ------------------------------------------------------------------------------------------------


class Vehicle:

    def __init__(self, length, width, centroid, angle, velocity, acceleration, reference_track=None):

        # parameters
        self.lookahead_distance = 10
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
        self.parallel_acc = get_vector(0, 0)
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
        orthogonal_vector = get_orthogonal_unit_vector(dir_vect)
        orthogonal_vector = ScaledPoint(orthogonal_vector.get_x() * self.width / 2,
                                        orthogonal_vector.get_y() * self.width / 2)

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

    def set_perpendicular_acc(self):
        proportional_error = self.error
        derivative_error = self.error - self.prev_error

        perp_acc_magnitude = (P_PER_ACC_WEIGHT * proportional_error) + (D_PER_ACC_WEIGHT * derivative_error)

        self.perpendicular_acc = get_vector(self.theta + (self.angle * 180 / math.pi), perp_acc_magnitude)
        self.perpendicular_acc = keep_in_limit(self.perpendicular_acc, self.acc_limit)

        if self.velocity.norm() >= self.goal_speed:
            self.perpendicular_acc = get_vector(0, 0)

    def set_parallel_acc(self):
        if abs(self.lookahead_angle) > radians(angle_threshold):
            dec_magnitude = self.lookahead_angle - math.radians(angle_threshold)
            self.parallel_acc = (self.velocity / self.velocity.norm()) * dec_magnitude * (-P_PARALLEL_DEC_WEIGHT)
        else:
            self.parallel_acc = (self.velocity / self.velocity.norm()) * (
                    P_PARALLEL_ACC_WEIGHT * (self.goal_speed - self.velocity.norm()))
        self.parallel_acc = keep_in_limit(self.parallel_acc, self.acc_limit)

    def controller(self):
        self.set_perpendicular_acc()
        self.set_parallel_acc()
        return self.perpendicular_acc + self.parallel_acc

    def update_state_vars(self):
        acc = self.controller()
        self.velocity += (acc * dt)
        self.velocity = keep_in_limit(self.velocity, self.speed_limit)
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

    def current_face_intersection_points(self):
        """Finds the intersection points of the line representing the front of the vehicle and the current face of
        the vehicle.

        :param self: A Vehicle object.

        :return: A list of intersection points as tuples.
        """
        intersection_points = []

        # Get half-edges of the current face of the vehicle
        half_edges = self.current_face.get_face_hedges()

        # Check each half-edge for intersection with the line representing the front of the vehicle
        for half_edge in half_edges:
            # Get the coordinates of the half-edge as a line segment
            segment1 = [(half_edge.origin.x, half_edge.origin.y), (half_edge.destination.x, half_edge.destination.y)]

            # Get the coordinates of the line representing the front of the vehicle as a line segment
            segment2 = self.get_car_perpendicular_line()

            # Get the intersection point of the two line segments
            intersection_point = geometry_functions.get_intersection_point(segment1, segment2)

            # Check if the intersection point lies within the bounds of the half-edge
            if (intersection_point[0] is not None
                    and geometry_functions.x_lies_between(x1=half_edge.origin.x, x2=half_edge.destination.x,
                                                          x=intersection_point[0])
                    and geometry_functions.x_lies_between(x1=half_edge.origin.y, x2=half_edge.destination.y,
                                                          x=intersection_point[1])):
                intersection_points.append((intersection_point[0], intersection_point[1]))

        return intersection_points

    def get_closest_intersection_points(self):
        """
        Returns the two closest intersection points between the line representing the front of the vehicle and
        the current face of the vehicle's DCEL road network.

        :param self: Vehicle object
        :return: List of two tuples representing the two closest intersection points
        """
        # Initialize the closest points to (0, 0)
        closest_point_1 = (0, 0)
        closest_point_2 = (0, 0)

        # Get the coordinates of the mid_point of the vehicle front line
        vehicle_x, vehicle_y = self.get_car_mid_point()
        # Get coordinates of the line parallel to the body of the vehicle
        line_start = (vehicle_x, vehicle_y)
        line_end = (self.velocity.get_x() + vehicle_x, self.velocity.get_y() + vehicle_y)

        # Initialize minimum distances to a high value
        min_distance_1 = 999999999
        min_distance_2 = 999999999

        # Get intersection points between the line and the current face of the vehicle
        intersection_points = self.current_face_intersection_points()

        # Find the two closest intersection points
        for point in intersection_points:
            # Calculate distance from vehicle's front line mid point to intersection point
            distance = math.sqrt((vehicle_x - point[0]) ** 2 + (vehicle_y - point[1]) ** 2)

            # Determine if intersection point is on the left or right of the line
            if geometry_functions.point_lies_left(point, line_start, line_end):
                # If on the left and closer than current closest point 1, update the closest point 1 and distance 1
                if distance < min_distance_1:
                    min_distance_1 = distance
                    closest_point_1 = point
            # If on the right and closer than current closest point 2, update the closest point 2 and distance 2
            elif distance < min_distance_2:
                min_distance_2 = distance
                closest_point_2 = point

        # Return the two closest intersection points
        return [closest_point_1, closest_point_2]

    def get_error_midpoint_approach(self, intersection_points):
        """
            Calculates the signed distance between the midpoint of the line joining the intersection points
            of the vehicle's front line with its current face and the midpoint of the vehicle's front line.
            A positive value indicates the vehicle is to the left of the line, while a negative value indicates
            the vehicle is to the right of the line.

            :param self: Vehicle object representing the vehicle.
            :param intersection_points: A list containing two tuples representing the intersection points
                                        of the vehicle's front line with its current face.
            :return: The signed error of the vehicle
            """

        vehicle_point = self.get_vehicle_front_mid_point()

        a = geometry_functions.get_euclidean_distance((intersection_points[0][0], intersection_points[0][1]),
                                                      (vehicle_point.x, vehicle_point.y))
        b = geometry_functions.get_euclidean_distance((intersection_points[1][0], intersection_points[1][1]),
                                                      (vehicle_point.x, vehicle_point.y))

        return (b - a) / 2

    def get_vehicle_front_mid_point(self):
        """
            Compute the mid-point of the front line of the given vehicle.

        :param self: The vehicle object.
        :return: A Point object representing the mid-point of the front line of the vehicle.

        """
        segment = self.get_car_perpendicular_line()
        return Point((segment[0][0] + segment[1][0]) / 2, (segment[0][1] + segment[1][1]) / 2)

    def get_projected_points(self):
        """
            Returns the projected point of the vehicle's front mid-point onto the vehicle's reference track.

            :param self: The vehicle whose projected point on the reference track is to be found.
            :param distance_ahead: The distance between the projected point and the point on reference track from where the error will be measured.
            :return: The projected point of the vehicle's front mid-point onto the vehicle's reference track.
            """

        vehicle_point = self.get_vehicle_front_mid_point()
        projected_point_distance = self.reference_track.project(vehicle_point)
        return (self.reference_track.interpolate(projected_point_distance),
                self.reference_track.interpolate(projected_point_distance + self.lookahead_distance))

    def get_track_segment(self):
        """
        Finds the track segment of the entire reference track on which the vehicle is currently projected.

        :param self: Vehicle object representing the vehicle
        :return: A tuple of two Point objects representing the start and end points of the track segment
                 on which the vehicle is currently projected
        """
        # Get the projected point of the vehicle on the reference track
        projected_point = self.get_projected_points()[1]

        # Iterate over all the line segments of the reference track and find the segment closest to the projected point
        coordinates = self.reference_track.coords
        for i in range(len(coordinates) - 1):
            line_segment = LineString([coordinates[i], coordinates[i + 1]])
            if line_segment.distance(projected_point) < 1:
                # If the projected point is within 1 unit of distance from the line segment, return the start and end points
                return Point(coordinates[i][0], coordinates[i][1]), Point(coordinates[i + 1][0], coordinates[i + 1][1])

    def get_error_from_track(self):
        """
        Calculate the signed error between the vehicle's front mid-point and the projected point on the reference track.

        :param self: The vehicle object to calculate the error for.
        :return: The signed error, positive if the vehicle is to the right of the track and negative if it is to the left.
        """
        vehicle_point = self.get_vehicle_front_mid_point()
        segment_point_1, segment_point_2 = self.get_track_segment()

        error = vehicle_point.distance(self.get_projected_points()[1])
        # error = vehicle_point.distance(self.reference_track)

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
        lookahead_point = (self.get_projected_points()[1].x, self.get_projected_points()[1].y)
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
        lookahead_point = (self.get_projected_points()[1].x, self.get_projected_points()[1].y)
        track_direction = (vehicle_point, lookahead_point)
        return headed_direction, track_direction

    def set_current_face(self, road_network):
        """
        A utility function for compute_parameters function

        :param road_network: The road network DCEL object
        :param self: The vehicle whose current face needs to be set.
        """
        vehicle_x, vehicle_y = self.get_car_mid_point()
        self.current_face = road_network.get_face_for_point((vehicle_x, vehicle_y))

        if self.current_face is None:
            self.current_face = self.prev_face
        if self.current_face is not None:
            self.prev_face = self.current_face
