import os
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, os.pardir))
sys.path.append(parent_dir)

import math
from math import cos, sin, radians
from shapely import LineString, Point
import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from .point import ScaledPoint
lane_width = 20


def get_y_at_x(origin, destination, x):
    """
    :param origin: tuple in the form (x, y) representing starting point of a segment
    :param destination: tuple in the form (x, y) representing ending point of a segment
    :param x: x coordinate of a point
    :return: y coordinate corresponding to x coordinate of the point on the segment
    """
    m, b = get_slope_and_y_intercept([origin, destination])
    return m * x + b


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


def get_common_point(segment1, segment2):
    points = [segment1[0], segment1[1], segment2[0], segment2[1]]
    length = 4
    for i in range(length):
        for j in range(i+1, length):
            if points[i] == points[j]:
                return points[i]
    return [None]


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
    if m1 == m2:
        return get_common_point(segment1, segment2)

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


def rearrange_line_strings(line1, line2):
    # Calculate the distances between the second point of line1 and the first point of line2
    p1 = Point(line1.coords[0])
    p2 = Point(line2.coords[0])
    min_dis = float('inf')
    distance1 = Point(line1.coords[0]).distance(Point(line2.coords[0]))
    if min_dis > distance1:
        p1 = Point(line1.coords[0])
        p2 = Point(line2.coords[0])
        min_dis = distance1
    distance2 = Point(line1.coords[0]).distance(Point(line2.coords[1]))
    if min_dis > distance2:
        p1 = Point(line1.coords[0])
        p2 = Point(line2.coords[1])
        min_dis = distance2
    distance3 = Point(line1.coords[1]).distance(Point(line2.coords[0]))
    if min_dis > distance3:
        p1 = Point(line1.coords[1])
        p2 = Point(line2.coords[0])
        min_dis = distance3
    distance4 = Point(line1.coords[1]).distance(Point(line2.coords[1]))
    if min_dis > distance4:
        p1 = Point(line1.coords[1])
        p2 = Point(line2.coords[1])
        min_dis = distance4

    if (Point(line1.coords[1]).x, Point(line1.coords[1]).y) != (p1.x, p1.y):
        line1 = LineString([line1.coords[1], line1.coords[0]])

    if (Point(line2.coords[0]).x, Point(line2.coords[0]).y) != (p2.x, p2.y):
        line2 = LineString([line2.coords[1], line2.coords[0]])

    return line1, line2


def get_interpolated_curve(line_string1, line_string2):
    # Extract x and y coordinates from the first line string
    line_string1 = LineString(line_string1)
    line_string2 = LineString(line_string2)

    line_string1, line_string2 = rearrange_line_strings(line_string1, line_string2)
    x_data1, y_data1 = line_string1.xy

    # Extract x and y coordinates from the second line string
    x_data2, y_data2 = line_string2.xy

    # Combine the x and y coordinates into a single line string
    line_string_combined = list(line_string1.coords) + list(line_string2.coords)

    # Extract x and y coordinates from the combined line string
    x_data_combined, y_data_combined = zip(*line_string_combined)
    # Cubic spline interpolation
    y_f = interp1d(x_data_combined, y_data_combined, 'cubic')
    x = []
    for i in range(len(x_data_combined) - 1):
        x_values = np.linspace(x_data_combined[i], x_data_combined[i + 1], 5)[:-1]  # Limit to 4 points
        x.extend(x_values)
    y = y_f(x)
    filtered_x = []
    filtered_y = []
    for i in range(len(x)):
        if not (line_string1.bounds[0] <= x[i] <= line_string1.bounds[2] or line_string2.bounds[0] <= x[i] <= line_string2.bounds[2]):
            filtered_x.append(x[i])
            filtered_y.append(y[i])
    filtered_x.insert(0, line_string1.coords[-1][0])
    filtered_x.append(line_string2.coords[0][0])
    filtered_y.insert(0, line_string1.coords[-1][1])
    filtered_y.append(line_string2.coords[0][1])

    # Create a LineString object from the filtered points
    filtered_line_string = LineString(zip(filtered_x, filtered_y))

    return list(filtered_line_string.coords)


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


def merge_line_strings(l1, l2):
    coords = list(l1.coords)[:-1]
    coords.extend(list(l2.coords))
    return LineString(coords)

