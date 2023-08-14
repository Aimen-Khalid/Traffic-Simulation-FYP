import math
from RUSTIC.Utility import geometry
from math import cos, sin, radians
from RUSTIC.Utility.point import ScaledPoint
from shapely import Point, LineString, Polygon
from shapely.ops import nearest_points
from RUSTIC.ObstacleAvoidance import obstacle_avoidance

# constants
params = {
    "dt": 0.01,
    "D_PER_ACC_WEIGHT": 20,
    "P_PER_ACC_WEIGHT": 10,
    "P_PARALLEL_DEC_WEIGHT": 20,
    "P_PARALLEL_ACC_WEIGHT":0.5,
    "angle_threshold": 2.5,
    "lookahead_time": 1.5
}


class Lookahead:
    def __init__(self, time=None, distance=None, point=None, angle=None, error=None):
        self.time = time
        self.distance = distance
        self.point = point
        self.angle = angle
        self.error = error


class Vehicle:

    def __init__(self, length, width, centroid, angle, velocity, acceleration, goal_speed=8, trail_color="grey",
                 track_color="green", reference_track=None):

        # parameters
        self.angle_threshold = params["angle_threshold"]
        self.initial_speed = velocity.norm()
        self.length = length
        self.width = width
        self.speed_limit = 10
        self.acc_limit = 2.65
        self.goal_speed = goal_speed
        self.theta = -90
        self.reference_track = reference_track
        self.next_lane_curve = None
        self.trail_color = trail_color
        self.track_color = track_color
        self.in_breaking_distance = False

        # state variables
        self.centroid = centroid
        self.front_mid_point = None
        self.speed_lookahead = Lookahead(time=params['lookahead_time'])
        self.turning_lookahead = Lookahead(time=params['lookahead_time'],
                                           distance=self.goal_speed * self.speed_lookahead.time - 5)
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
        self.prev_face = None
        self.face_mid_point = None
        self.stopped = False

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
        proportional_error = self.turning_lookahead.error
        derivative_error = self.turning_lookahead.error - self.prev_error

        perpendicular_acc_magnitude = (params["P_PER_ACC_WEIGHT"] * proportional_error) + \
                                      (params["D_PER_ACC_WEIGHT"] * derivative_error)
        self.perpendicular_acc_sign = -1 if perpendicular_acc_magnitude < 0 else 1

        self.perpendicular_acc = geometry.get_vector(self.theta + (self.angle * 180 / math.pi),
                                                     perpendicular_acc_magnitude)
        self.perpendicular_acc = geometry.keep_in_limit(self.perpendicular_acc, self.acc_limit)

        if self.velocity.norm() >= self.goal_speed:
            self.perpendicular_acc = geometry.get_vector(0, 0)

    def set_parallel_acc(self):
        breaking_distance = self.velocity.norm()
        destination_point = Point(self.reference_track.coords[-1])
        if destination_point.distance(self.front_mid_point) < 2 * breaking_distance:
            self.in_breaking_distance = True
        if self.in_breaking_distance:
            dec_magnitude = -self.velocity.norm()
            self.parallel_acc_sign = -1
            self.parallel_acc = (self.velocity / self.velocity.norm()) * dec_magnitude
        elif abs(self.speed_lookahead.angle) > radians(params["angle_threshold"]):
            dec_magnitude = (self.speed_lookahead.angle - radians(params["angle_threshold"])) * (
                -params["P_PARALLEL_DEC_WEIGHT"])
            self.parallel_acc_sign = -1 if dec_magnitude * (-params["P_PARALLEL_DEC_WEIGHT"]) < 0 else 1
            self.parallel_acc = (self.velocity / self.velocity.norm()) * dec_magnitude
            # print('turning')
        else:
            acc_magnitude = params["P_PARALLEL_ACC_WEIGHT"] * (self.goal_speed - self.velocity.norm())
            self.parallel_acc = (self.velocity / self.velocity.norm()) * acc_magnitude
            self.parallel_acc_sign = -1 if acc_magnitude < 0 else 1
            # print('accelerating')

        self.parallel_acc = geometry.keep_in_limit(self.parallel_acc, self.acc_limit)

    def controller(self):
        self.set_perpendicular_acc()
        self.set_parallel_acc()
        resultant_acc = self.parallel_acc + self.perpendicular_acc
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

    def get_circle_intersection_point(self, circle_radius):
        circle_center = self.front_mid_point
        vel_vector = Point(self.velocity.get_x(), self.velocity.get_y())
        circle = circle_center.buffer(circle_radius)
        boundary = circle.boundary
        intersection = boundary.intersection(self.reference_track)
        if intersection.is_empty:
            closest_point, _ = nearest_points(self.reference_track, self.front_mid_point)
            return closest_point
        vel_vector = Point(vel_vector.x + circle_center.x, vel_vector.y + circle_center.y)
        # If multiple intersection points exist, find the one closest to the velocity vector
        if intersection.geom_type == 'MultiPoint':
            intersection = nearest_points(vel_vector, intersection)[1]
        return intersection

    def set_lookahead_points(self, ):
        """ Creates a circle with center at car centroid and radius equal to lookahead distance.
            Finds the intersection points of the circle and track.
            Returns the intersection point closest to the car's velocity vector
        """
        self.speed_lookahead.point = self.get_circle_intersection_point(self.speed_lookahead.distance)
        self.turning_lookahead.point = self.get_circle_intersection_point(self.turning_lookahead.distance)

    def set_orientation(self):
        self.angle = math.atan2(self.velocity.get_y(), self.velocity.get_x())

    def set_prev_error(self):
        self.prev_error = self.error

    def set_lookahead_distance(self):
        self.turning_lookahead.distance = self.velocity.norm() * self.turning_lookahead.time
        self.speed_lookahead.distance = self.velocity.norm() * self.speed_lookahead.time

    def set_prev_face(self):
        self.prev_face = self.current_face

    def modify_track_around_obstacles(self, road_network):
        obstacles = obstacle_avoidance.obstacles + obstacle_avoidance.static_cars
        for obstacle in obstacles:
            if obstacle.centroid.distance(Point(self.centroid.x, self.centroid.y)) < 50:
                # print('modifying path')
                self.reference_track = obstacle_avoidance.modify_reference_track_for_obstacles(
                    [obstacle], self.reference_track, road_network)

    def add_to_obstacles(self):
        x, y = self.get_xy_lists()
        x = [round(i, 2) for i in x]
        y = [round(i, 2) for i in y]
        obstacle = Polygon(list(zip(x, y)))
        obstacle_avoidance.static_cars.append(obstacle)

    def update_state_vars(self, road_network):
        if self.stopped:
            return
        if round(self.velocity.norm(), 2) <= 0.5:
            self.stopped = True
            self.add_to_obstacles()
            return
        self.set_vehicle_front_mid_point()
        self.set_closest_point()
        self.set_lookahead_distance()
        self.set_lookahead_points()
        self.set_prev_error()
        self.set_per_acc_error()
        self.set_parallel_acc_error()
        self.set_velocity()
        self.set_centroid()
        self.set_orientation()
        self.modify_track_around_obstacles(road_network)

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

    def get_lookahead_angle_magnitude(self, angle_type):
        """Returns lookahead angle magnitude in radians"""
        headed_direction = (
        self.front_mid_point, Point(self.velocity.x + self.front_mid_point.x, self.velocity.y + self.front_mid_point.y))
        if angle_type == "turning":
            track_direction = (self.front_mid_point, self.turning_lookahead.point)
        else:
            track_direction = (self.front_mid_point, self.speed_lookahead.point)
        vector1 = Point(headed_direction[1].x - headed_direction[0].x, headed_direction[1].y - headed_direction[0].y)
        vector2 = Point(track_direction[1].x - track_direction[0].x, track_direction[1].y - track_direction[0].y)

        dot_product = vector1.x * vector2.x + vector1.y * vector2.y

        magnitude1 = math.sqrt(vector1.x ** 2 + vector1.y ** 2)
        magnitude2 = math.sqrt(vector2.x ** 2 + vector2.y ** 2)

        cos_theta = dot_product / (magnitude1 * magnitude2)
        cos_theta = min(cos_theta, 1)
        return math.acos(cos_theta)

    def get_error_sign(self, error_type):
        # returns sign of perpendicular acceleration error term
        segment_point_1 = Point(self.front_mid_point.x, self.front_mid_point.y)
        segment_point_2 = Point(self.velocity.get_x() + self.front_mid_point.x,
                                self.velocity.get_y() + self.front_mid_point.y)

        # translating the points to origin
        if error_type == "turning":
            lookahead_point = Point(self.turning_lookahead.point.x - segment_point_1.x,
                                    self.turning_lookahead.point.y - segment_point_1.y)
        else:
            lookahead_point = Point(self.speed_lookahead.point.x - segment_point_1.x,
                                    self.speed_lookahead.point.y - segment_point_1.y)
        segment_point_2 = Point(segment_point_2.x - segment_point_1.x, segment_point_2.y - segment_point_1.y)

        cross_product = segment_point_2.x * lookahead_point.y - segment_point_2.y * lookahead_point.x

        return -1 if cross_product > 0 else 1

    def set_per_acc_error(self):
        self.turning_lookahead.angle = self.get_lookahead_angle_magnitude("turning")
        self.turning_lookahead.error = self.turning_lookahead.angle * self.get_error_sign("turning")

    def set_parallel_acc_error(self):
        self.speed_lookahead.angle = self.get_lookahead_angle_magnitude("speed")
        self.speed_lookahead.error = self.lookahead_angle * self.get_error_sign("speed")

    def set_current_face(self, road_network):
        vehicle_x, vehicle_y = self.front_mid_point.x, self.front_mid_point.y
        self.current_face = road_network.get_face_for_point((vehicle_x, vehicle_y))

    def set_reference_curve(self, road_network):
        starting_face = self.current_face
        connected_curves = self.current_face.get_connected_curves(self.reference_track)
        while len(connected_curves) != 0:
            if len(connected_curves) == 1:
                next_curve = connected_curves[0]
            else:
                next_curve = connected_curves[2]
            self.reference_track = geometry.merge_line_strings(self.reference_track, LineString(next_curve))
            mid_point = ((next_curve[0][0] + next_curve[1][0]) / 2, (next_curve[0][1] + next_curve[1][1]) / 2)
            self.current_face = road_network.get_face_for_point(mid_point)
            if self.current_face == starting_face:
                return
            last_curve = LineString(list(self.reference_track.coords)[-2:])
            connected_curves = self.current_face.get_connected_curves(last_curve)
        self.current_face = starting_face
