import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Point, LineString
from scipy.interpolate import interp1d, CubicSpline


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


# Define rotation function
def rotate(points, angle):
    theta = np.radians(angle)
    cos, sin = np.cos(theta), np.sin(theta)
    rotation_matrix = np.array([[cos, -sin], [sin, cos]])

    return np.dot(points, rotation_matrix)


def get_interpolated_curve(line_string1, line_string2, angle):
    # Rotate the line strings
    line_string1 = rotate(line_string1, angle)
    line_string2 = rotate(line_string2, angle)

    # Convert them back to LineString objects
    line_string1 = LineString(line_string1)
    line_string2 = LineString(line_string2)

    line_string1, line_string2 = rearrange_line_strings(
        line_string1, line_string2)
    # Combine the x and y coordinates into a single line string
    line_string_combined = list(line_string1.coords) + \
        list(line_string2.coords)

    # Extract x and y coordinates from the combined line string
    x_data_combined, y_data_combined = zip(*line_string_combined)
    y_f = interp1d(x_data_combined, y_data_combined, 'cubic')
    x = []
    for i in range(len(x_data_combined) - 1):
        x_values = np.linspace(
            x_data_combined[i], x_data_combined[i + 1], 50)[:-1]  # Limit to 4 points
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

    # Rotate back to the original coordinates
    filtered_coords = rotate(list(zip(filtered_x, filtered_y)), -angle)

    # Create a LineString object from the filtered points
    filtered_line_string = LineString(filtered_coords)

    return list(filtered_line_string.coords)


# 90
l1 = [[96.25, 80.0], [96.25, 25.0]]    # [(460, -2), (338, 71)]
l2 = [[170.0, 83.75], [100.0, 83.75]]  # [(317, 71), (226, 26)]
# normal case
# l1 = [[460, -2], [338, 71]]  # [[460, -2], [338, 71]]
# l2 = [[317, 71], [226, 26]]  # [[17, 71], [226, 26]]
#
# l1 = [[35, 24], [88, 75]]
# l2 = [[87, 108], [38, 153]]
x = [l1[0][0], l1[1][0]]  # 96.25, 96.25
y = [l1[0][1], l1[1][1]]  # 80.0, 25.0

plt.plot(x, y)

for i in range(len(x)):
    plt.text(x[i], y[i], f'({x[i]}, {y[i]})', ha='center', va='bottom')

x = [l2[0][0], l2[1][0]]
y = [l2[0][1], l2[1][1]]
plt.plot(x, y)

# Plot the points
for i in range(len(x)):
    plt.text(x[i], y[i], f'({x[i]}, {y[i]})', ha='center', va='bottom')


curve = get_interpolated_curve(l2, l1, 20)
x = [curve[i][0] for i in range(len(curve))]
y = [curve[i][1] for i in range(len(curve))]

plt.plot(x, y)

plt.show()

print()
