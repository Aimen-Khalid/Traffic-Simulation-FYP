import math
import dcel
import car
from point import ScaledPoint
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import time
from tqdm import tqdm
import winsound
from shapely import Point, LineString
import networkx as nx

ROAD_EDGE_COLOR = 'black'
ROAD_COLOR = 'lightgrey'
NON_ROAD_COLOR = 'white'
TRAIL_COLOR = 'grey'

"""
Head over to this link to understand the terms used in this project:
https://drive.google.com/file/d/1s4K-4sUjk51QQ3viBoLcKz9DzYK-8FNi/view?usp=sharing
"""


def write_vertices_to_file(vertices, file_name):
    with open(file_name, "w") as file:
        for vertex in vertices:
            for coordinate in vertex:
                file.write(str(coordinate) + "\t")
            file.write("\n")


def load_vertices_from_file(file_name):
    vertices = []
    with open(file_name, "r") as file:
        for line in file:
            vertex = tuple(float(x) for x in line.strip().split('\t'))
            vertices.append(vertex)
    return vertices


def write_segments_to_file(segments, file_name):
    with open(file_name, "w") as file:
        for segment in segments:
            for vertices in segment:
                for vertex in vertices:
                    file.write(str(vertex) + "\t")
            file.write("\n")


def load_segments_from_file(file_name):
    segments = []
    with open(file_name, "r") as file:
        for line in file:
            vertices = [float(x) for x in line.strip().split("\t")]
            segment = [(vertices[0], vertices[1]), (vertices[2], vertices[3])]
            segments.append(segment)
    return segments


def point_lies_left(point, line_start, line_end):
    """
    :param point: a tuple in the form (x, y)
    :param line_start: a tuple in the form (x, y) representing starting point of the line
    :param line_end: a tuple in the form (x, y) representing ending point of the line
    :returns: true if point lies to the left of the line, false otherwise
    """
    dx = line_end[0] - line_start[0]
    dy = line_end[1] - line_start[1]
    dx1 = point[0] - line_start[0]
    dy1 = point[1] - line_start[1]
    cross_product = dx * dy1 - dy * dx1
    return cross_product >= 0


def get_vertices_and_segments():
    """
        Allows the user to interactively plot a graph by clicking on a grid of points.

        To draw the graph in continuity, simply click on each vertex you want to add.

        To start a new, disconnected component of the graph, click on the last vertex you
        created. This will allow you to draw a new vertex without having a line segment
        between this vertex and the previous vertex.

        To finish creating the graph, click twice on the last vertex you plotted.

        vertices - a list of tuples representing the coordinates of each point clicked.

        segments - a list of lists, where each inner list contains two tuples representing the start and end points of
        a line segment in the graph.

        :returns: vertices and segments
    """

    # create a figure and axes object
    fig, ax = plt.subplots(figsize=(100, 100))
    # set the aspect ratio of the plot to be equal
    ax.set_aspect('equal')
    # set the axis limits and ticks
    ax.axis([0, 50, 0, 50])
    ax.xaxis.set_ticks(range(0, 900, 30))
    ax.yaxis.set_ticks(range(0, 700, 30))

    # add a grid to the plot
    ax.grid(color='green', linestyle='-', linewidth=0.5)

    # initialize variables to keep track of vertices and line segments
    prev_x = 0
    prev_y = 0
    vertices = []
    segments = []
    first = True
    grid_spacing = 30

    # continue drawing vertices and line segments until the function is interrupted
    while True:
        # get a single point from user input
        point = fig.ginput(n=1, show_clicks=True, mouse_add=1)
        # if there is no point, continue waiting for input
        if len(point) == 0:
            continue
        # round the point to the nearest grid point
        x = round(point[0][0] / grid_spacing) * grid_spacing
        y = round(point[0][1] / grid_spacing) * grid_spacing
        # add the point to the list of vertices if it hasn't already been added
        if (x, y) not in vertices:
            vertices.append((x, y))
        # if this isn't the first point, plot a line segment between this point and the previous point
        if not first:
            plt.plot([x, prev_x], [y, prev_y], 'o-')
            segment = [(prev_x, prev_y), (x, y)]
            # if the previous point is different from this point, add the segment to the list of segments
            if prev_x != x or prev_y != y:
                segments.append(segment)
        else:
            # if this is the first point, just plot it
            plt.plot(x, y, 'o-')
        # draw the plot
        plt.draw()
        # if this is the first point of a connected component, and it is the same as the previous point, we're done
        # drawing
        if first and prev_x == x and prev_y == y:
            return vertices, segments
        # update the first variable and previous point coordinates
        first = prev_x == x and prev_y == y
        prev_x = x
        prev_y = y


def draw_and_save_road_network_graph(vertices_file_name, segments_file_name):
    vertices, segments = get_vertices_and_segments()
    write_vertices_to_file(vertices, vertices_file_name)
    write_segments_to_file(segments, segments_file_name)


def get_slope_and_y_intercept(segment):
    """
    :param segment: a list of two tuples in the form (x, y) representing starting and ending coordinates of the segment.
    :returns: slope and y-intercept of the segment
    """
    x1, y1 = segment[0]
    x2, y2 = segment[1]

    if x2 - x1 == 0:
        # slope of vertical segment is infinity
        slope = "inf"
        y_intercept = 0
    else:
        slope = (y2 - y1) / (x2 - x1)
        y_intercept = y1 - slope * x1
    return slope, y_intercept


def get_y_at_x(origin, destination, x):
    """
    :param origin: tuple in the form (x, y) representing starting point of a segment
    :param destination: tuple in the form (x, y) representing ending point of a segment
    :param x: x coordinate of a point
    :return: y coordinate corresponding to x coordinate of the point on the segment
    """
    m, b = get_slope_and_y_intercept([origin, destination])
    return m * x + b


def get_intersection_point(segment1, segment2):
    """
    Calculates the intersection point of two line segments.

    :param segment1: A list of two tuples representing the starting and ending points of the first line segment.
    :param segment2: A list of two tuples representing the starting and ending points of the second line segment.

    :returns: A list containing coordinates of the intersection point, or [None] if the segments do not intersect.
    """

    # Calculate the slope and y-intercept of each segment
    m1, b1 = get_slope_and_y_intercept(segment1)
    m2, b2 = get_slope_and_y_intercept(segment2)

    # Check if the segments are parallel
    if m1 != "inf" and m2 != "inf" and m1 - m2 == 0:
        return [None]

    # Check if either segment is vertical
    if m1 == "inf":
        # Calculate x coordinate of intersection point for vertical segment1
        x = segment1[0][0]
        y = get_y_at_x((segment2[0][0], segment2[0][1]), (segment2[1][0], segment2[1][1]), x)
    elif m2 == "inf":
        # Calculate x coordinate of intersection point for vertical segment2
        x = segment2[0][0]
        y = get_y_at_x((segment1[0][0], segment1[0][1]), (segment1[1][0], segment1[1][1]), x)

    # Check if either segment is horizontal
    elif m1 == 0:
        # Calculate y coordinate of intersection point for horizontal segment1
        y = segment1[0][1]
        x = get_y_at_x((segment2[0][1], segment2[0][0]), (segment2[1][1], segment2[1][0]), y)
    elif m2 == 0:
        # Calculate y coordinate of intersection point for horizontal segment2
        y = segment2[0][1]
        x = get_y_at_x((segment1[0][1], segment1[0][0]), (segment1[1][1], segment1[1][0]), y)

    # Otherwise, calculate the intersection point normally
    else:
        x = (b2 - b1) / (m1 - m2)
        y = m1 * x + b1

    return [x, y]


def x_lies_between(x1, x2, x):
    """Checks whether a given x value lies between two other x values.

        :param x1: The x value corresponding to the start of the range.
        :param x2: The x value corresponding to the end of the range.
        :param x: The x value to be checked.
        :return: True if x lies between x1 and x2 (inclusive), False otherwise.
        """
    x_min = min(x1, x2)
    x_max = max(x1, x2)
    return x_min <= x <= x_max


def get_mid_point(segment):
    """
        Calculate the midpoint of a line segment.

        :param segment: A list of two tuples, each representing the starting and ending point of the line segment.
        :return: A tuple representing the coordinates of the midpoint of the line segment.
        """
    return (segment[0][0] + segment[1][0]) / 2, (segment[0][1] + segment[1][1]) / 2


def get_euclidean_distance(point1, point2):
    """
        Calculates the Euclidean distance between two points.

        :param point1: A tuple representing the x and y coordinates of the first point.
        :param point2: A tuple representing the x and y coordinates of the second point.
        :return: A float representing the Euclidean distance between the two points.
        """
    return math.sqrt((point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2)


def current_face_intersection_points(vehicle):
    """Finds the intersection points of the line representing the front of the vehicle and the current face of
    the vehicle.

    :param vehicle: A Vehicle object.

    :return: A list of intersection points as tuples.
    """
    intersection_points = []

    # Get half-edges of the current face of the vehicle
    half_edges = vehicle.current_face.get_face_hedges()

    # Check each half-edge for intersection with the line representing the front of the vehicle
    for half_edge in half_edges:
        # Get the coordinates of the half-edge as a line segment
        segment1 = [(half_edge.origin.x, half_edge.origin.y), (half_edge.destination.x, half_edge.destination.y)]

        # Get the coordinates of the line representing the front of the vehicle as a line segment
        segment2 = vehicle.get_car_perpendicular_line()

        # Get the intersection point of the two line segments
        intersection_point = get_intersection_point(segment1, segment2)

        # Check if the intersection point lies within the bounds of the half-edge
        if (intersection_point[0] is not None
                and x_lies_between(x1=half_edge.origin.x, x2=half_edge.destination.x, x=intersection_point[0])
                and x_lies_between(x1=half_edge.origin.y, x2=half_edge.destination.y, x=intersection_point[1])):
            intersection_points.append((intersection_point[0], intersection_point[1]))

    return intersection_points


def get_closest_intersection_points(vehicle):
    """
    Returns the two closest intersection points between the line representing the front of the vehicle and
    the current face of the vehicle's DCEL road network.

    :param vehicle: Vehicle object
    :return: List of two tuples representing the two closest intersection points
    """
    # Initialize the closest points to (0, 0)
    closest_point_1 = (0, 0)
    closest_point_2 = (0, 0)

    # Get the coordinates of the mid_point of the vehicle front line
    vehicle_x, vehicle_y = vehicle.get_car_mid_point()
    # Get coordinates of the line parallel to the body of the vehicle
    line_start = (vehicle_x, vehicle_y)
    line_end = (vehicle.velocity.get_x() + vehicle_x, vehicle.velocity.get_y() + vehicle_y)

    # Initialize minimum distances to a high value
    min_distance_1 = 999999999
    min_distance_2 = 999999999

    # Get intersection points between the line and the current face of the vehicle
    intersection_points = current_face_intersection_points(vehicle)

    # Find the two closest intersection points
    for point in intersection_points:
        # Calculate distance from vehicle's front line mid point to intersection point
        distance = math.sqrt((vehicle_x - point[0]) ** 2 + (vehicle_y - point[1]) ** 2)

        # Determine if intersection point is on the left or right of the line
        if point_lies_left(point, line_start, line_end):
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


def get_error(vehicle, intersection_points):
    """
        Calculates the signed distance between the midpoint of the line joining the intersection points
        of the vehicle's front line with its current face and the midpoint of the vehicle's front line.
        A positive value indicates the vehicle is to the left of the line, while a negative value indicates
        the vehicle is to the right of the line.

        :param vehicle: Vehicle object representing the vehicle.
        :param intersection_points: A list containing two tuples representing the intersection points
                                    of the vehicle's front line with its current face.
        :return: The signed error of the vehicle
        """

    vehicle_point = get_vehicle_front_mid_point(vehicle)

    a = get_euclidean_distance((intersection_points[0][0], intersection_points[0][1]),
                               (vehicle_point.x, vehicle_point.y))
    b = get_euclidean_distance((intersection_points[1][0], intersection_points[1][1]),
                               (vehicle_point.x, vehicle_point.y))

    return (b - a) / 2


def get_vehicle_front_mid_point(vehicle):
    """
        Compute the mid-point of the front line of the given vehicle.

    :param vehicle: The vehicle object.
    :return: A Point object representing the mid-point of the front line of the vehicle.

    """
    segment = vehicle.get_car_perpendicular_line()
    return Point((segment[0][0] + segment[1][0]) / 2, (segment[0][1] + segment[1][1]) / 2)


def get_projected_point(vehicle):
    """
        Returns the projected point of the vehicle's front mid-point onto the vehicle's reference track.

        :param vehicle: The vehicle whose projected point on the reference track is to be found.
        :return: The projected point of the vehicle's front mid-point onto the vehicle's reference track.
        """
    vehicle_point = get_vehicle_front_mid_point(vehicle)
    projected_point_distance = vehicle.reference_track.project(vehicle_point)
    return vehicle.reference_track.interpolate(projected_point_distance)


def get_track_segment(vehicle):
    """
    Finds the track segment of the entire reference track on which the vehicle is currently projected.

    :param vehicle: Vehicle object representing the vehicle
    :return: A tuple of two Point objects representing the start and end points of the track segment
             on which the vehicle is currently projected
    """
    # Get the projected point of the vehicle on the reference track
    projected_point = get_projected_point(vehicle)

    # Iterate over all the line segments of the reference track and find the segment closest to the projected point
    coordinates = vehicle.reference_track.coords
    for i in range(len(coordinates) - 1):
        line_segment = LineString([coordinates[i], coordinates[i + 1]])
        if line_segment.distance(projected_point) < 1:
            # If the projected point is within 1 unit of distance from the line segment, return the start and end points
            return Point(coordinates[i][0], coordinates[i][1]), Point(coordinates[i + 1][0], coordinates[i + 1][1])


def get_error_from_track(vehicle):
    """
    Calculate the signed error between the vehicle's front mid-point and the projected point on the reference track.

    :param vehicle: The vehicle object to calculate the error for.
    :return: The signed error, positive if the vehicle is to the right of the track and negative if it is to the left.
    """
    vehicle_point = get_vehicle_front_mid_point(vehicle)
    segment_point_1, segment_point_2 = get_track_segment(vehicle)

    error = vehicle_point.distance(vehicle.reference_track)

    # translating the points to origin
    vehicle_point = Point(vehicle_point.x - segment_point_1.x, vehicle_point.y - segment_point_1.y)
    segment_point_2 = Point(segment_point_2.x - segment_point_1.x, segment_point_2.y - segment_point_1.y)

    cross_product = segment_point_2.x * vehicle_point.y - segment_point_2.y * vehicle_point.x

    return error * (1 if cross_product > 0 else -1)


def build_dcel_from_file(vertices_file_name, segments_file_name):
    vertices = load_vertices_from_file(vertices_file_name)
    segments = load_segments_from_file(segments_file_name)
    dcel_object = dcel.Dcel()
    dcel_object.build_dcel(vertices, segments)
    return dcel_object


def initialize_parameters_dict():
    # utility function for compute_parameters function
    return {
        'vehicle': [],
        'velocity': [],
        'speed': [],
        'acc': [],
        'acc_magnitude': [],
        'error': [],
        'intersection_points': [],
        'text': [],
        'trail_x': [],
        'trail_y': []
    }


def set_vehicle_current_face(road_network, vehicle):
    """
    A utility function for compute_parameters function

    :param road_network: The road network DCEL object
    :param vehicle: The vehicle whose current face needs to be set.
    """
    vehicle_x, vehicle_y = vehicle.get_car_mid_point()
    vehicle.current_face = road_network.get_face_for_point((vehicle_x, vehicle_y))

    if vehicle.current_face is None:
        vehicle.current_face = vehicle.prev_face
    if vehicle.current_face is not None:
        vehicle.prev_face = vehicle.current_face


def update_parameters_list(vehicle, parameters, vehicle_x, vehicle_y, intersection_points_list):
    """
    utility function for compute_parameters function
    """
    text = (f'acc: {str(vehicle.acc.norm())}'
            + '\ntheta: ' + str(vehicle.theta)
            + '\nerror: ' + str(vehicle.error)
            )
    parameters['text'].append(text)

    x, y = vehicle.get_xy_lists()
    parameters['vehicle'].append((x, y))

    x = [vehicle_x, vehicle.velocity.get_x() + vehicle_x]
    y = [vehicle_y, vehicle.velocity.get_y() + vehicle_y]
    parameters['velocity'].append((x, y))

    parameters['trail_x'].append(vehicle_x)
    parameters['trail_y'].append(vehicle_y)
    x = [vehicle_x, vehicle.acc.get_x() + vehicle_x]
    y = [vehicle_y, vehicle.acc.get_y() + vehicle_y]
    parameters['acc'].append((x, y))
    parameters['acc_magnitude'].append(vehicle.acc_magnitude)
    parameters['speed'].append(vehicle.velocity.norm())
    parameters['error'].append(vehicle.error)

    intersection_points_list.append((vehicle.face_mid_point[0], vehicle.face_mid_point[1]))
    parameters['intersection_points'].append(intersection_points_list)

    return parameters


def compute_parameters1(road_network, vehicle, frames):  # face mid-point approach
    """Compute various parameters of a vehicle as it moves along a road network.

    :param road_network: A DCEL object representing the environment in which the vehicle is moving.
    :param vehicle: A vehicle object representing the vehicle whose parameters we want to compute.
    :param frames: The number of frames for which to compute the vehicle parameters.

    :returns: A dictionary containing various vehicle parameters computed over the specified number of frames.
    """
    print("Computation in progress...")
    start = time.time()
    parameters = initialize_parameters_dict()
    # Loop over the specified number of frames
    with tqdm(total=frames) as pbar:
        for _ in range(frames):
            # Get the current position of the vehicle
            vehicle_x, vehicle_y = vehicle.get_car_mid_point()
            set_vehicle_current_face(road_network, vehicle)
            # If the vehicle is not in a valid face, skip to the next iteration
            if vehicle.current_face is None:
                continue
            # Find the two closest intersection points of the vehicle's current face
            intersection_points_list = get_closest_intersection_points(vehicle)
            # Compute the midpoint of the two intersection points as the current face midpoint
            vehicle.face_mid_point = get_mid_point([(intersection_points_list[0][0], intersection_points_list[0][1]),
                                                    (intersection_points_list[1][0], intersection_points_list[1][1])])
            vehicle.prev_error = vehicle.error
            # Compute the vehicle's current error
            vehicle.error = get_error(vehicle, intersection_points_list)
            vehicle.update_state_vars()
            # Update the parameters dictionary with the computed values for this iteration
            parameters = update_parameters_list(parameters, vehicle_x, vehicle_y, intersection_points_list)
            # Update the progress bar
            pbar.update(1)

    end = time.time()
    print(f"{int(end - start)} Seconds")

    return parameters


def compute_parameters(vehicle, frames):  # reference track approach
    """
    Computes vehicle parameters for each frame of a simulation using the reference track approach.

    :param vehicle: A Vehicle object representing the vehicle being simulated.
    :type vehicle: Vehicle
    :param frames: The number of simulation frames to compute parameters for.
    :type frames: int
    :return: A dictionary containing vehicle parameters for each frame of the simulation.
    :rtype: dict
    """
    print("Computation in progress...")
    start = time.time()

    # Initialize an empty dictionary to store the computed parameters for each frame of the simulation
    parameters = {
        'vehicle': [],  # The x and y coordinates of the vehicle
        'velocity': [],  # The x and y components of the vehicle's velocity vector
        'speed': [],  # The magnitude of the vehicle's velocity vector
        'acc': [],  # The x and y components of the vehicle's acceleration vector
        'acc_magnitude': [],  # The magnitude of the vehicle's acceleration vector
        'error': [],  # The vehicle's lateral error from the reference track
        'text': [],  # Additional text information to display on the simulation plot
        'trail_x': [],  # The x coordinates of the vehicle's trail (for visualization purposes)
        'trail_y': [],  # The y coordinates of the vehicle's trail (for visualization purposes)
        'projected_point': []  # The projected point of the vehicle onto the reference track
    }

    # Use tqdm to display a progress bar while computing the parameters for each frame
    with tqdm(total=frames) as pbar:
        for _ in range(frames):
            # Get the current x and y coordinates of the vehicle's midpoint
            vehicle_x, vehicle_y = vehicle.get_car_mid_point()

            # Add the current x and y coordinates to the trail list for visualization purposes
            parameters['trail_x'].append(vehicle_x)
            parameters['trail_y'].append(vehicle_y)

            # Compute the vehicle's lateral error from the reference track and update the vehicle's state variables
            vehicle.prev_error = vehicle.error
            vehicle.error = get_error_from_track(vehicle)
            projected_point = get_projected_point(vehicle)
            vehicle.update_state_vars()

            # Add additional text information to the plot for this frame
            text = (f'acc: {str(vehicle.acc.norm())}'
                    + '\nframes: ' + str(_)
                    + '\nerror: ' + str(vehicle.error)
                    )
            parameters['text'].append(text)

            # Add the vehicle's x and y coordinates to the dictionary
            x, y = vehicle.get_xy_lists()
            parameters['vehicle'].append((x, y))

            # Add the x and y components of the vehicle's velocity vector to the dictionary
            x = [vehicle_x, vehicle.velocity.get_x() + vehicle_x]
            y = [vehicle_y, vehicle.velocity.get_y() + vehicle_y]
            parameters['velocity'].append((x, y))

            # Add the x and y components of the vehicle's acceleration vector to the dictionary
            x = [vehicle_x, vehicle.acc.get_x() + vehicle_x]
            y = [vehicle_y, vehicle.acc.get_y() + vehicle_y]
            parameters['acc'].append((x, y))

            # Add the magnitude of the vehicle's acceleration vector to the dictionary
            parameters['acc_magnitude'].append(vehicle.acc_magnitude)

            # Add the magnitude of the vehicle's velocity vector to the dictionary
            parameters['speed'].append(vehicle.velocity.norm())

            # Add the magnitude of the vehicle's error to the dictionary
            parameters['error'].append(vehicle.error)

            # Add the vehicle's projected point onto the reference track to the dictionary
            parameters['projected_point'].append(projected_point)

            pbar.update(1)

    end = time.time()
    print(f"{int(end - start)} Seconds")

    return parameters


def draw_roads(road_network, ax):
    f = road_network.outer_face

    x = [f.upper_left.x, f.upper_right.x, f.bottom_right.x, f.bottom_left.x]
    y = [f.upper_left.y, f.upper_right.y, f.bottom_right.y, f.bottom_left.y]
    ax.fill(x, y, ec=ROAD_EDGE_COLOR, fc=NON_ROAD_COLOR)
    for face in road_network.faces:
        vertices = face.get_face_vertices()
        x = [x for x, y in vertices]
        x.append(vertices[0][0])
        y = [y for x, y in vertices]
        y.append(vertices[0][1])
        if face.tag == dcel.ROAD:
            ax.fill(x, y, ec=ROAD_EDGE_COLOR, fc=ROAD_COLOR)
        else:
            ax.fill(x, y, ec=ROAD_EDGE_COLOR, fc=NON_ROAD_COLOR)


def plot_parameters(vehicle, start, end):
    acc = np.loadtxt("acc.txt")
    speed = np.loadtxt("speed.txt")
    error = np.loadtxt("error.txt")

    acc = acc[start: end]
    speed = speed[start: end]
    error = error[start: end]

    fig, (ax1, ax2, ax3) = plt.subplots(nrows=3, ncols=1, figsize=(8, 10))
    frames = np.linspace(0, len(acc), len(acc))

    ax1.plot(frames, acc)

    ax1.axhline(y=vehicle.acc_limit, color='blue', label="acc limit")
    ax1.axhline(y=-1 * vehicle.acc_limit, color='blue')
    max_acc = np.max(acc[10:])
    min_acc = np.min(acc[10:])
    ax1.text(0.95, 0.95, f"max acc={max_acc:.2f}\nmin acc={min_acc:.2f}", transform=ax1.transAxes, ha='right', va='top',
             bbox=dict(facecolor='white', edgecolor='none', alpha=0.5))

    ax1.legend(loc='upper left')
    ax1.set_ylabel('Acceleration')
    # ax1.set_xlim([400, 450])

    ax2.plot(frames, error)
    ax2.set_ylabel('Error')
    ax2.set_ylim([-20, 20])
    # ax2.set_xlim([400, 450])

    ax3.plot(frames, speed)
    ax3.set_ylabel('Speed')
    ax3.set_xlabel('Frames')

    fig.suptitle(f"kp={car.P_ACC_WEIGHT} kd={car.D_ACC_WEIGHT}")

    # Adjust the spacing between subplots
    fig.tight_layout()

    # plt.show()

    fig.savefig(f"p{car.P_ACC_WEIGHT} d {car.D_ACC_WEIGHT} theta {vehicle.theta}plot.png")


def simulate1(road_network, vehicle, frames, parameters, file_name):  # face mid-point
    fig = plt.figure()
    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box')
    x, y = vehicle.reference_track.xy
    ax.plot(x, y)
    draw_roads(road_network, ax)

    # road_network.show_road_network(ax)

    vehicle_line, = ax.plot([], [])
    trail_line, = ax.plot([], [], color=TRAIL_COLOR)
    ideal_trail_line, = ax.plot([], [], color='green')

    ideal_trail_line.set_data(
        [parameters['intersection_points'][j][2][0] for j in range(5, len(parameters['intersection_points']))],
        [parameters['intersection_points'][j][2][1] for j in range(5, len(parameters['intersection_points']))])
    acc_line, = ax.plot([], [])
    velocity_line, = ax.plot([], [])
    intersection_points = ax.scatter([], [], color='red', s=5)
    text = ax.text(80, 450, vehicle.error)

    def init():
        return vehicle_line, acc_line, velocity_line, intersection_points, text, ideal_trail_line

    def animate(i):
        vehicle_line.set_data(parameters['vehicle'][i])
        velocity_line.set_data(parameters['velocity'][i])
        trail_line.set_data(parameters['trail_x'][:i], parameters['trail_y'][:i])

        acc_line.set_data(parameters['acc'][i])
        intersection_points.set_offsets(parameters['intersection_points'][i])
        text.set_text(parameters['text'][i])
        return vehicle_line, velocity_line, acc_line, intersection_points, text, trail_line

    print("Animation in progress...")
    start = time.time()
    anim = FuncAnimation(fig, animate, init_func=init, frames=frames, blit=True)
    anim.save(file_name, writer='ffmpeg', fps=100)
    end = time.time()
    print("Animation COMPLETED....")
    print(f"{int(end - start)} Seconds")


def simulate(road_network, vehicle, frames, parameters, file_name):  # reference track
    fig = plt.figure()
    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box')

    draw_roads(road_network, ax)

    vehicle_line, = ax.plot([], [])
    trail_line, = ax.plot([], [], color=TRAIL_COLOR)

    acc_line, = ax.plot([], [])
    velocity_line, = ax.plot([], [])
    projected_point = ax.scatter([], [], color='red', s=5)
    text = ax.text(0, 80, vehicle.error)

    def init():
        return vehicle_line, acc_line, velocity_line, projected_point, text

    def animate(i):
        vehicle_line.set_data(parameters['vehicle'][i])
        velocity_line.set_data(parameters['velocity'][i])
        trail_line.set_data(parameters['trail_x'][:i], parameters['trail_y'][:i])

        acc_line.set_data(parameters['acc'][i])
        projected_point.set_offsets((parameters['projected_point'][i].x, parameters['projected_point'][i].y))

        text.set_text(parameters['text'][i])
        return vehicle_line, velocity_line, acc_line, text, trail_line

    print("Animation in progress...")
    start = time.time()
    anim = FuncAnimation(fig, animate, init_func=init, frames=frames, blit=True)
    anim.save(file_name, writer='ffmpeg', fps=150)
    end = time.time()
    print("Animation COMPLETED....")
    print(f"{int(end - start)} Seconds")


def write_to_file(arrays):
    with open("acc.txt", "w") as file:
        for a in arrays['acc_magnitude']:
            file.write(f"{a}\n")
    with open("speed.txt", "w") as file:
        for s in arrays['speed']:
            file.write(f"{s}\n")
    with open("error.txt", "w") as file:
        for e in arrays['error']:
            file.write(f"{e}\n")


def create_simulation_video(vehicle, road_network, frames):
    parameters = compute_parameters1(road_network, vehicle, frames)
    write_to_file(parameters)

    # simulation will start after closing the plot figure
    plot_parameters(vehicle, start=5, end=frames)

    simulate1(road_network, vehicle, frames=frames, parameters=parameters,
              file_name=f"p{car.P_ACC_WEIGHT} d {car.D_ACC_WEIGHT} theta {vehicle.theta}.mp4")


def angle_between_segments(seg1, seg2):
    # Compute the vectors for the two segments
    seg1_point_1 = seg1[0]
    seg1_point_2 = seg1[1]
    seg2_point1 = seg2[0]
    seg2_point2 = seg2[1]
    v1 = (seg1_point_2.x - seg1_point_1.x, seg1_point_2.y - seg1_point_1.y)
    v2 = (seg2_point2.x - seg2_point1.x, seg2_point2.y - seg2_point1.y)

    # Compute the angle between the two vectors
    dot_product = v1[0] * v2[0] + v1[1] * v2[1]
    mag_v1 = math.sqrt(v1[0] ** 2 + v1[1] ** 2)
    mag_v2 = math.sqrt(v2[0] ** 2 + v2[1] ** 2)
    cos_angle = dot_product / (mag_v1 * mag_v2)
    angle = math.acos(cos_angle)

    # Determine the sign of the angle based on the direction of the turn
    cross_product = v1[0] * v2[1] - v1[1] * v2[0]
    if cross_product > 0:
        angle = 2 * math.pi - angle

    # Convert the angle to degrees and return it
    return math.degrees(angle)


def rotate_line(start_point, end_point, theta, anticlockwise=False):
    # Convert the angle from degrees to radians
    if not anticlockwise:
        theta = -1 * theta
    theta_rad = math.radians(theta)

    # Translate the line so that the starting point is at the origin
    translated_end_point = (end_point.x - start_point.x, end_point.y - start_point.y)

    # Rotate the translated end point by the angle theta
    rotated_end_point = (translated_end_point[0] * math.cos(theta_rad) - translated_end_point[1] * math.sin(theta_rad),
                         translated_end_point[0] * math.sin(theta_rad) + translated_end_point[1] * math.cos(theta_rad))

    # Translate the rotated end point back to the original coordinates
    new_end_point = (rotated_end_point[0] + start_point.x, rotated_end_point[1] + start_point.y)

    return new_end_point


def translate_segment(segment, length, anticlockwise=False):
    if anticlockwise:
        length = -1 * length
    # Extract the starting and ending points of the segment
    start, end = segment

    start = Point(start[0], start[1])
    end = Point(end[0], end[1])
    # Compute the vector representing the segment
    dx = end.x - start.x
    dy = end.y - start.y

    # Compute the length of the vector
    vector_length = ((dx ** 2) + (dy ** 2)) ** 0.5
    if vector_length == 0:
        vector_length = 1
    # Compute the cosine and sine of the angle between the vector and the x-axis
    cos_theta = dx / vector_length
    sin_theta = dy / vector_length

    # Translate the starting and ending points of the segment
    new_start_x = start.x + length * sin_theta
    new_start_y = start.y - length * cos_theta
    new_end_x = end.x + length * sin_theta
    new_end_y = end.y - length * cos_theta

    # Return the translated segment as a list of two tuples
    return [(new_start_x, new_start_y), (new_end_x, new_end_y)]


def get_angle(edge):
    """Calculate the angle the edge makes with x-axis, in radians.

    :param edge: a tuple containing the coordinates of the edge's endpoints

    :return: the angle of the edge, in radians
    """
    origin, destination = edge[0], edge[1]
    dx = destination[0] - origin[0]
    dy = destination[1] - origin[1]
    length = math.sqrt(dx * dx + dy * dy)

    if length == 0:
        return 0
    if dy > 0:
        return math.acos(dx / length)
    else:
        return 2 * math.pi - math.acos(dx / length)


def get_edges_cw(graph, node):
    outgoing_edges = list(graph.edges(node))
    outgoing_edges.sort(key=lambda e: get_angle(e), reverse=True)
    return outgoing_edges


def create_graph(vertices_file_name, segment_file_name):
    graph = nx.DiGraph()
    # Add vertices to the graph
    vertices_ = load_vertices_from_file(vertices_file_name)
    segments_ = load_segments_from_file(segment_file_name)
    # Add nodes to the graph with their specific positions
    for vertex in vertices_:
        graph.add_node(vertex, pos=vertex)
    # Add edges to the graph
    for segment in segments_:
        graph.add_edge(segment[0], segment[1], visited=False, cw_start=None, cw_end=None, acw_start=None, acw_end=None)
        graph.add_edge(segment[1], segment[0], visited=False, cw_start=None, cw_end=None, acw_start=None, acw_end=None)

    return graph


def show_graph(graph):
    # Get the node positions from their attributes
    node_positions = nx.get_node_attributes(graph, 'pos')

    # Draw the graph with node positions
    nx.draw(graph, pos=node_positions, node_size=20, font_size=8, with_labels=True)
    plt.show()


def graph_to_lanes(graph):
    vertices = []
    segments = []
    partitions = []

    lane_width = 1
    for node in graph.nodes:
        if graph.degree(node) == 2:
            outgoing_edge = list(graph.edges(node))[0]
            if not graph.get_edge_data(outgoing_edge[0], outgoing_edge[1])['cw_start']:
                cw_start = translate_segment(outgoing_edge, lane_width)[0]
                acw_start = translate_segment(outgoing_edge, lane_width, anticlockwise=True)[0]
                if graph.get_edge_data(outgoing_edge[0], outgoing_edge[1])['cw_start'] is None:
                    nx.set_edge_attributes(graph, {
                        (outgoing_edge[0], outgoing_edge[1]): {'cw_start': cw_start, 'acw_start': acw_start}})
                    nx.set_edge_attributes(graph, {
                        (outgoing_edge[1], outgoing_edge[0]): {'acw_end': cw_start, 'cw_end': acw_start}})
            continue
        translated_segments_cw = []
        translated_segments_acw = []

        edges = get_edges_cw(graph, node)
        for edge in edges:
            translated_segments_cw.append(translate_segment(edge, lane_width))
            translated_segments_acw.append(translate_segment(edge, lane_width, anticlockwise=True))

        for i, edge in enumerate(edges):
            if i + 1 == len(edges):
                segment1 = translated_segments_acw[0]
            else:
                segment1 = translated_segments_acw[i + 1]
            segment2 = translated_segments_cw[i]
            segment3 = translated_segments_acw[i]
            if i == 0:
                segment4 = translated_segments_cw[len(edges) - 1]
            else:
                segment4 = translated_segments_cw[i - 1]
            cw_start = tuple(get_intersection_point(segment1, segment2))
            acw_start = tuple(get_intersection_point(segment3, segment4))
            if graph.get_edge_data(edge[0], edge[1])['cw_start'] is None:
                nx.set_edge_attributes(graph, {(edge[0], edge[1]): {'cw_start': cw_start, 'acw_start': acw_start}})
                nx.set_edge_attributes(graph, {(edge[1], edge[0]): {'acw_end': cw_start, 'cw_end': acw_start}})
            # vertices.extend((cw_start, acw_start))
    for node in graph.nodes:
        edges = get_edges_cw(graph, node)
        for edge in edges:
            if graph.get_edge_data(edge[0], edge[1])['cw_end'][0] is not None:
                continue
            neighbor_edges = get_edges_cw(graph, edge[1])
            if len(neighbor_edges) == 1:
                cw_end = translate_segment(edge, lane_width)[1]
                acw_end = translate_segment(edge, lane_width, anticlockwise=True)[1]
                nx.set_edge_attributes(graph, {(edge[0], edge[1]): {'cw_end': cw_end, 'acw_end': acw_end}})
                nx.set_edge_attributes(graph, {(edge[1], edge[0]): {'acw_start': cw_end, 'cw_start': acw_end}})
                continue
            index = neighbor_edges.index(edge)
            prev = len(neighbor_edges) - 1 if index == 0 else index - 1
            cw_end = graph.get_edge_data(neighbor_edges[prev][0], neighbor_edges[prev][1])['cw_start']
            next_ = 0 if index == len(neighbor_edges) - 1 else index + 1
            acw_end = graph.get_edge_data(neighbor_edges[next_][0], neighbor_edges[next_][1])['acw_start']
            if graph.get_edge_data(edge[0], edge[1])['cw_end'] is None:
                nx.set_edge_attributes(graph, {(edge[0], edge[1]): {'cw_end': cw_end, 'acw_end': acw_end}})
                nx.set_edge_attributes(graph, {(edge[1], edge[0]): {'acw_start': cw_end, 'cw_start': acw_end}})
            # vertices.extend((cw_end, acw_end))

    for node in graph.nodes:
        edges = get_edges_cw(graph, node)
        for edge in edges:
            if graph.get_edge_data(edge[0], edge[1])['visited']:
                continue
            nx.set_edge_attributes(graph, {(edge[0], edge[1]): {'visited': True}})
            nx.set_edge_attributes(graph, {(edge[1], edge[0]): {'visited': True}})
            segments.extend(
                (
                    [
                        (graph.get_edge_data(edge[0], edge[1])['cw_start']),
                        (graph.get_edge_data(edge[0], edge[1])['cw_end']),
                    ],
                    # [
                    #     (edge[0]),
                    #     (edge[1])
                    # ],
                    [
                        (graph.get_edge_data(edge[0], edge[1])['acw_start']),
                        (graph.get_edge_data(edge[0], edge[1])['acw_end']),
                    ],
                )
            )
            vertices.extend(
                (
                    (graph.get_edge_data(edge[0], edge[1])['cw_start']),
                    (graph.get_edge_data(edge[0], edge[1])['cw_end']),
                    (graph.get_edge_data(edge[0], edge[1])['acw_start']),
                    (graph.get_edge_data(edge[0], edge[1])['acw_end']),
                    # (edge[0]),
                    # (edge[1])
                )
            )
            partitions.extend(
                (
                    [
                        (graph.get_edge_data(edge[0], edge[1])['cw_start']),
                        (graph.get_edge_data(edge[0], edge[1])['acw_start']),
                    ],
                    [
                        (graph.get_edge_data(edge[0], edge[1])['cw_end']),
                        (graph.get_edge_data(edge[0], edge[1])['acw_end']),
                    ],)
            )
            # partitions.extend(
            #     (
            #         [
            #             (graph.get_edge_data(edge[0], edge[1])['cw_start']),
            #             (edge[0])
            #         ],
            #         [
            #             (edge[0]),
            #             (graph.get_edge_data(edge[0], edge[1])['acw_start'])
            #         ],
            #         [
            #             (graph.get_edge_data(edge[0], edge[1])['cw_end']),
            #             (edge[1])
            #         ],
            #         [
            #             (edge[1]),
            #             (graph.get_edge_data(edge[0], edge[1])['acw_end'])
            #         ]
            #     )
            #)

    unique_partitions = {frozenset(segment) for segment in partitions}
    unique_partitions = [list(segment) for segment in unique_partitions]
    return vertices, segments, unique_partitions


def show_lanes_from_graph(graph, segments_file_name):
    vertices, segments, partitions = graph_to_lanes(graph)
    fig, ax = plt.subplots()

    for segment in segments:
        start, end = segment
        x_values = [start[0], end[0]]
        y_values = [start[1], end[1]]
        ax.text(start[0], start[1], f"{round(start[0])},{str(round(start[1]))}", fontsize=6, color='green')
        ax.text(end[0], end[1], f"{round(end[0])},{str(round(end[1]))}", fontsize=6, color='green')
        ax.plot(x_values, y_values, color='green')

    segments_ = load_segments_from_file(segments_file_name)
    for segment in segments_:
        start, end = segment
        x_values = [start[0], end[0]]
        y_values = [start[1], end[1]]
        # ax.text(start[0], start[1], f"{round(start[0])},{str(round(start[1]))}", fontsize=6, color='black')
        # ax.text(end[0], end[1], f"{round(end[0])},{str(round(end[1]))}", fontsize=6, color='black')
        ax.plot(x_values, y_values, color='black')

    for segment in partitions:
        start, end = segment
        x_values = [start[0], end[0]]
        y_values = [start[1], end[1]]
        ax.plot(x_values, y_values, color='grey')

    ax.set_aspect('equal')
    plt.show()


def main():
    option = input("Enter desired option number:\n"
                   "1. Create simulation video on new road network\n"
                   "2. Create simulation video on road network from file\n"
                   "3. Draw road network graph, create its lane and convert to DCEL. ")
    vehicle = car.Vehicle(length=20, width=10, centroid=ScaledPoint(250, 100),
                          angle=90, velocity=ScaledPoint(50, 5), acceleration=ScaledPoint(0, 0))
    frames = 1000

    # if option == 1:
    #     draw_and_save_road_network_graph("vertices1.txt", "segments1.txt")
    #     road_network = build_dcel_from_file("vertices.txt", "segments.txt")
    #     create_simulation_video(vehicle, road_network, frames)
    #     winsound.Beep(frequency=2500, duration=1000)

    initial_speed = vehicle.velocity.norm()
    vertices, segments = get_vertices_and_segments()
    vertices_file_name = "vertices1.txt"
    segments_file_name = "segments1.txt"
    write_vertices_to_file(vertices, vertices_file_name)
    write_segments_to_file(segments, segments_file_name)
    graph = create_graph(vertices_file_name, segments_file_name)
    translated_vertices, translated_segments, partitions = graph_to_lanes(graph)
    show_graph(graph)
    show_lanes_from_graph(graph, segments_file_name)
    translated_segments.extend(partitions)
    dcel_obj = dcel.Dcel(graph)
    dcel_obj.build_dcel(translated_vertices, translated_segments)
    dcel_obj.show_road_network()


# main()


















# draw_and_save_road_network_graph("vertices2.txt", "segments2.txt")
G = nx.DiGraph()
# Add vertices to the graph
vertices_ = load_vertices_from_file("vertices1.txt")
segments_ = load_segments_from_file("segments1.txt")
# Add nodes to the graph with their specific positions
for vertex in vertices_:
    G.add_node(vertex, pos=vertex)
# Add edges to the graph
for segment in segments_:
    G.add_edge(segment[0], segment[1], visited=False, cw_start=None, cw_end=None, acw_start=None, acw_end=None)
    G.add_edge(segment[1], segment[0], visited=False, cw_start=None, cw_end=None, acw_start=None, acw_end=None)


# Get the node positions from their attributes
node_positions = nx.get_node_attributes(G, 'pos')

# Draw the graph with node positions
nx.draw(G, pos=node_positions, node_size=20, font_size=8, with_labels=False)
plt.show()



vertices, segments, partitions = graph_to_lanes(G)
# print(segments)

fig, ax = plt.subplots()

for segment in segments:
    start, end = segment
    x_values = [start[0], end[0]]
    y_values = [start[1], end[1]]
    # ax.text(start[0], start[1], f"{round(start[0])},   {str(round(start[1]))}", fontsize=6, color='green')
    # ax.text(end[0], end[1], f"{round(end[0])},   {str(round(end[1]))}", fontsize=6, color='green')
    ax.plot(x_values, y_values, color='green')

for segment in segments_:
    start, end = segment
    x_values = [start[0], end[0]]
    y_values = [start[1], end[1]]
    # ax.text(start[0], start[1], f"{round(start[0])},{str(round(start[1]))}", fontsize=6, color='black')
    # ax.text(end[0], end[1], f"{round(end[0])},{str(round(end[1]))}", fontsize=6, color='black')
    ax.plot(x_values, y_values, color='black')

unique_partitions = {frozenset(segment) for segment in partitions}
unique_partitions = [list(segment) for segment in unique_partitions]
segments.extend(unique_partitions)

for segment in partitions:
    start, end = segment
    x_values = [start[0], end[0]]
    y_values = [start[1], end[1]]
    ax.plot(x_values, y_values, color='grey')


ax.set_aspect('equal')
plt.show()

dcel_obj = dcel.Dcel(G)
dcel_obj.build_dcel(vertices, segments)


# dcel_obj.show_dcel()
dcel_obj.show_road_network()

