from shapely.geometry import box
import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from shapely import Point, LineString
from shapely.geometry import Polygon


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


def path_interpolation(l1, l2, num_points=1000):
    # Combine the points from l1 and l2
    points = np.array(list(l1.coords) + list(l2.coords))

    # Separate the x and y coordinates
    x = points[:, 0]
    y = points[:, 1]

    # Compute the cumulative distance along the path
    distance = np.concatenate(
        [[0], np.cumsum(np.sqrt(np.diff(x) ** 2 + np.diff(y) ** 2))])

    # Fit cubic splines to x and y as functions of distance
    cs_x = CubicSpline(distance, x)
    cs_y = CubicSpline(distance, y)

    # Create new distance values, skip the start and end points by a tiny amount
    distance_new = np.linspace(
        distance.min() + 1e-10, distance.max() - 1e-10, num_points)

    # Compute interpolated x and y values
    x_new = cs_x(distance_new)
    y_new = cs_y(distance_new)

    # Create a bounding box encompassing both l1 and l2
    min_x = min(min(point[0] for point in l1.coords),
                min(point[0] for point in l2.coords))
    min_y = min(min(point[1] for point in l1.coords),
                min(point[1] for point in l2.coords))
    max_x = max(max(point[0] for point in l1.coords),
                max(point[0] for point in l2.coords))
    max_y = max(max(point[1] for point in l1.coords),
                max(point[1] for point in l2.coords))
    bounding_box = box(min_x, min_y, max_x, max_y)

    # Filter out the interpolated points that are inside the bounding box
    points = np.array(list(zip(x_new, y_new)))
    points_outside_box = [
        point for point in points if not bounding_box.contains(Point(point))]

    # Separate the x and y coordinates of the points outside the box
    x_new, y_new = zip(*points_outside_box)

    return x_new, y_new


def plot_interpolation(l1, l2):
    # Call the path_interpolation function
    x_new, y_new = path_interpolation(l1, l2)

    # Extract coordinates from l1 and l2 for plotting
    l1_points = list(l1.coords)
    l2_points = list(l2.coords)

    # Extract x and y coordinates for LineString 1 and 2
    l1_x = [p[0] for p in l1_points]
    l1_y = [p[1] for p in l1_points]
    l2_x = [p[0] for p in l2_points]
    l2_y = [p[1] for p in l2_points]

    # Plot the original points and lines
    plt.plot(l1_x, l1_y, 'go-', label='Line 1')
    plt.plot(l2_x, l2_y, 'bo-', label='Line 2')

    # Adding labels to the points
    for p in l1_points+l2_points:
        plt.text(p[0], p[1], str(p))

    # Plot the interpolated data
    plt.plot(x_new, y_new, 'r-', label='Interpolated')

    # Plot the interpolated points
    plt.plot(x_new[::10], y_new[::10], 'gx', label='Interpolated Points')

    # Setting the legend
    plt.legend()

    plt.show()


# Defining your LineStrings
l1 = [[35, 24], [88, 75]]
l2 = [[87, 108], [38, 153]]
# Plotting

l1 = LineString(l1)
l2 = LineString(l2)

l1, l2 = rearrange_line_strings(l1, l2)

plot_interpolation(l1, l2)
