from shapely import Point, LineString
from scipy.interpolate import interp1d
from matplotlib import pyplot as plt
import numpy as np


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


def get_interpolated_curve(line_string1, line_string2, limit):
    # Extract x and y coordinates from the first line string
    line_string1 = LineString(line_string1)
    line_string2 = LineString(line_string2)

    line_string1, line_string2 = rearrange_line_strings(
        line_string1, line_string2)
    x_data1, y_data1 = line_string1.xy

    # Extract x and y coordinates from the second line string
    x_data2, y_data2 = line_string2.xy

    # Combine the x and y coordinates into a single line string
    line_string_combined = list(line_string1.coords) + \
        list(line_string2.coords)

    # Extract x and y coordinates from the combined line string
    x_data_combined, y_data_combined = zip(*line_string_combined)
    # Cubic spline interpolation
    y_f = interp1d(x_data_combined, y_data_combined, 'cubic')
    x = []
    for i in range(len(x_data_combined) - 1):
        x_values = np.linspace(
            x_data_combined[i], x_data_combined[i + 1], limit)[:-1]
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

#
# l1 = [[21, 38], [72, 89]]
# l2 = [[96, 115], [38, 146]]
# x = [l1[0][0], l1[1][0]]
# y = [l1[0][1], l1[1][1]]
# plt.plot(x, y)
#
# x = [l2[0][0], l2[1][0]]
# y = [l2[0][1], l2[1][1]]
# plt.plot(x, y)
#
# limit = 10
# curve = get_interpolated_curve(l2, l1, limit)
# x = [curve[i][0] for i in range(len(curve))]
# y = [curve[i][1] for i in range(len(curve))]
#
# plt.plot(x, y, 'b')
# plt.scatter(x, y, color='r')
# plt.show()
