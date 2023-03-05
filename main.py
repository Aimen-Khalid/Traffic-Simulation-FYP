import math
from math import sqrt
import dcel
import car
from point import ScaledPoint
import matplotlib.pyplot as plt
from car import point_line_side
from matplotlib.animation import FuncAnimation
import numpy as np

# Set the figure size and create the axis
fig, ax = plt.subplots(figsize=(100, 100))
ax.set_aspect('equal')

# Set the axis limits and add the grid lines
ax.axis([0, 50, 0, 50])
ax.xaxis.set_ticks(range(0, 900, 30))
ax.yaxis.set_ticks(range(0, 700, 30))
ax.grid(color='green', linestyle='-', linewidth=0.5)

vehicle = car.Vehicle(length=20, width=10, speed_limit=0, acc_limit=0, centroid=ScaledPoint(420, 30),
                      angle=5, v=ScaledPoint(50, 5), a=ScaledPoint(0, 0))


def draw_rectangle(p):
    x, y = p[0], p[1]
    plt.plot(x, y)


def get_vertices_list(vertices):
    return [(vertex.x, vertex.y) for vertex in vertices]


def write_vertices_to_file(vertices):
    with open("vertices.txt", "w") as file:
        for vertex in vertices:
            for coordinate in vertex:
                file.write(str(coordinate) + "\t")
            file.write("\n")


def load_vertices_from_file():
    vertices = []
    with open("vertices.txt", "r") as file:
        for line in file:
            vertex = tuple(int(x) for x in line.strip().split("\t"))
            vertices.append(vertex)
    return vertices


def write_segments_to_file(segments):
    with open("segments.txt", "w") as file:
        for segment in segments:
            for vertices in segment:
                for vertex in vertices:
                    file.write(str(vertex) + "\t")
            file.write("\n")


def load_segments_from_file():
    segments = []
    with open("segments.txt", "r") as file:
        for line in file:
            vertices = [int(x) for x in line.strip().split("\t")]
            segment = [(vertices[0], vertices[1]), (vertices[2], vertices[3])]
            segments.append(segment)
    return segments


def get_vertices_and_segments():
    prev_x = 0
    prev_y = 0

    vertices = []
    segments = []
    first = True
    grid_spacing = 30
    while True:
        # if keyboard.is_pressed('d'):
        #     return vertices, segments

        point = fig.ginput(n=1, show_clicks=True, mouse_add=1)
        if len(point) == 0:
            continue
        x = round(point[0][0] / grid_spacing) * grid_spacing
        y = round(point[0][1] / grid_spacing) * grid_spacing
        if (x, y) not in vertices:
            vertices.append((x, y))
        if not first:
            plt.plot([x, prev_x], [y, prev_y], 'o-')
            segment = [(prev_x, prev_y), (x, y)]
            if prev_x != x or prev_y != y:
                segments.append(segment)
        else:
            plt.plot(x, y, 'o-')
        plt.draw()
        if first and prev_x == x and prev_y == y:
            return vertices, segments
        first = prev_x == x and prev_y == y
        prev_x = x
        prev_y = y


def main():
    vertices, segments = get_vertices_and_segments()
    dcel_obj = dcel.Dcel()
    dcel_obj.build_dcel(vertices, segments)
    dcel_obj.show_dcel()
    write_vertices_to_file(vertices)
    write_segments_to_file(segments)


def get_m_and_b(segment):
    x1, y1 = segment[0]
    x2, y2 = segment[1]

    if x2 - x1 == 0:
        m = "inf"
        b = 0
    else:
        m = (y2 - y1) / (x2 - x1)
        b = y1 - m * x1
    return m, b


def get_y_at_x(origin, destination, x):
    m, b = get_m_and_b([origin, destination])
    return m * x + b


def get_intersection_point(segment1, segment2):
    m1, b1 = get_m_and_b(segment1)
    m2, b2 = get_m_and_b(segment2)
    if m1 != "inf" and m2 != "inf" and m1 - m2 == 0:
        return [None]
    if m1 == "inf":  # vertical line
        x = segment1[0][0]
        y = get_y_at_x((segment2[0][0], segment2[0][1]), (segment2[1][0], segment2[1][1]), x)
    elif m2 == "inf":  # vertical line
        x = segment2[0][0]
        # get x at y
        y = get_y_at_x((segment1[0][0], segment1[0][1]), (segment1[1][0], segment1[1][1]), x)
    elif m1 == 0:  # horizontal line y = c
        y = segment1[0][1]
        x = get_y_at_x((segment2[0][1], segment2[0][0]), (segment2[1][1], segment2[1][0]), y)
    elif m2 == 0:
        y = segment2[0][1]
        x = get_y_at_x((segment1[0][1], segment1[0][0]), (segment1[1][1], segment1[1][0]), y)
    else:
        x = (b2 - b1) / (m1 - m2)
        y = m1 * x + b1
    return [x, y]


def x_lies_between(x1, x2, x):
    x_min = min(x1, x2)
    x_max = max(x1, x2)
    return x_min <= x <= x_max


def get_mid_point(segment):
    return (segment[0][0] + segment[1][0]) / 2, (segment[0][1] + segment[1][1]) / 2


def get_point_distance(point1, point2):
    return math.sqrt((point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2)


# def get_face_intersection_points(vehicle):
#     faces = vehicle.current_face.inner_components
#     faces.append(vehicle.current_face)
#     intersection_points = []
#     for face in faces:
#         half_edge = face.outer_component
#         first_iteration = True
#         while first_iteration or half_edge is not face.outer_component:
#             first_iteration = False
#             segment1 = [(half_edge.origin.x, half_edge.origin.y), (half_edge.destination.x, half_edge.destination.y)]
#             segment2 = vehicle.get_car_perpendicular_line()
#             intersection_point = get_intersection_point(segment1, segment2)
#             if (
#                     intersection_point[0] is not None
#                     and x_lies_between(
#                         x1=half_edge.origin.x,
#                         x2=half_edge.destination.x,
#                         x=intersection_point[0],
#                         )
#                     and x_lies_between(
#                         x1=half_edge.origin.y,
#                         x2=half_edge.destination.y,
#                         x=intersection_point[1],
#                         )
#             ):
#                 intersection_points.append((intersection_point[0], intersection_point[1]))
#
#             half_edge = half_edge.next
#
#     return intersection_points


def get_face_intersection_points(vehicle):
    intersection_points = []

    half_edges = my_dcel.hedges_map.get_all_hedges()
    for half_edge in half_edges:
        segment1 = [(half_edge.origin.x, half_edge.origin.y), (half_edge.destination.x, half_edge.destination.y)]
        segment2 = vehicle.get_car_perpendicular_line()
        intersection_point = get_intersection_point(segment1, segment2)
        if (
                intersection_point[0] is not None
                and x_lies_between(
                    x1=half_edge.origin.x,
                    x2=half_edge.destination.x,
                    x=intersection_point[0],
                    )
                and x_lies_between(
                    x1=half_edge.origin.y,
                    x2=half_edge.destination.y,
                    x=intersection_point[1],
                    )
        ):
            intersection_points.append((intersection_point[0], intersection_point[1]))

    return intersection_points


def get_closest_intersection_points(vehicle):
    vehicle_x, vehicle_y = vehicle.get_car_mid_point()
    line_start = (vehicle_x, vehicle_y)
    line_end = (vehicle.velocity.get_x() + vehicle_x, vehicle.velocity.get_y() + vehicle_y)
    min_distance_1 = 999999999
    min_distance_2 = 999999999
    intersection_points = get_face_intersection_points(vehicle)
    for point in intersection_points:
        distance = sqrt((vehicle_x - point[0]) ** 2 + (vehicle_y - point[1]) ** 2)
        if point_line_side(point, line_start, line_end):
            if distance < min_distance_1:
                min_distance_1 = distance
                closest_point_1 = point
        elif distance < min_distance_2:
            min_distance_2 = distance
            closest_point_2 = point
    return [closest_point_1, closest_point_2]


def get_error(vehicle):
    intersection_points = get_closest_intersection_points(vehicle)
    if len(intersection_points) < 2:
        return (0, 0), vehicle.error * 10
    mid_point_x, mid_point_y = get_mid_point([(intersection_points[0][0], intersection_points[0][1]),
                                            (intersection_points[1][0], intersection_points[1][1])])
    segment = vehicle.get_car_perpendicular_line()
    vehicle_x = (segment[0][0] + segment[1][0]) / 2
    vehicle_y = (segment[0][1] + segment[1][1]) / 2
    distance = get_point_distance((mid_point_x, mid_point_y), (vehicle_x, vehicle_y))

    if distance < 1:
        distance *= 1000
    else:
        distance *= 10

    return (mid_point_x, mid_point_y), distance


def build_dcel_from_file():
    vertices = load_vertices_from_file()
    segments = load_segments_from_file()
    my_dcel = dcel.Dcel()
    my_dcel.build_dcel(vertices, segments)
    # show_dcel(my_dcel)
    return my_dcel


def compute(my_dcel, vehicle, frames):
    list_of_lines = []
    for _ in range(frames):
        lines = []

        for face in my_dcel.faces:
            vertices = face.get_face_vertices()
            x = [x for x, y in vertices]
            x.append(vertices[0][0])
            y = [y for x, y in vertices]
            y.append(vertices[0][1])
            line, = ax.plot(x, y)
            lines.append(line)
        list_of_lines.append(lines)


def compute_arrays(my_dcel, vehicle, frames):
    arrays = {
        'vehicle': [],
        'velocity': [],
        'acc': [],
        'intersection_points': [],
        'text': [],
        'trail': []
    }

    for _ in range(frames):
        vehicle_x, vehicle_y = vehicle.get_car_mid_point()
        arrays['trail'].append((vehicle_x, vehicle_y))
        vehicle.current_face = my_dcel.get_face_for_point((vehicle_x, vehicle_y))
        if vehicle.current_face is None:
            vehicle.current_face = vehicle.prev_face
        if vehicle.current_face is not None:
            vehicle.prev_face = vehicle.current_face
            intersection_points_list = get_closest_intersection_points(vehicle)
            vehicle.face_mid_point, vehicle.error = get_error(vehicle)

            dist = 0
            dist2 = 0
            if len(intersection_points_list) > 2:
                dist = math.sqrt((vehicle_x - intersection_points_list[0][0])**2 +
                                 (vehicle_y - intersection_points_list[0][1])**2)
                dist2 = math.sqrt((vehicle_x - intersection_points_list[1][0])**2 +
                                  (vehicle_y - intersection_points_list[1][1])**2)

            vehicle.update_state_vars(dist, dist2)

            text = (f'acc: {str(vehicle.acc.norm())}'
                            + '\nTHETA: ' + str(vehicle.theta)
                            + '\nVelocity: ' + str(vehicle.velocity.norm())
                            + '\nDist: ' + str(dist)
                            + '\nDist2: ' + str(dist2))
            arrays['text'].append(text)

            x, y = vehicle.get_xy_lists()
            arrays['vehicle'].append((x, y))

            x = [vehicle_x, vehicle.velocity.get_x() + vehicle_x]
            y = [vehicle_y, vehicle.velocity.get_y() + vehicle_y]
            arrays['velocity'].append((x, y))

            x = [vehicle_x, vehicle.acc.get_x() + vehicle_x]
            y = [vehicle_y, vehicle.acc.get_y() + vehicle_y]
            arrays['acc'].append((x, y))

            intersection_points_list.append((vehicle.face_mid_point[0], vehicle.face_mid_point[1]))
            arrays['intersection_points'].append(intersection_points_list)

    return arrays


def simulate(my_dcel, vehicle, frames, arrays, fn):
    fig = plt.figure()

    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box')
    lines = []
    for count, face in enumerate(my_dcel.faces):
        vertices = face.get_face_vertices()
        x = [x for x, y in vertices]
        x.append(vertices[0][0])
        y = [y for x, y in vertices]
        y.append(vertices[0][1])
        line, = ax.plot(x, y)
        lines.append(line)

    vehicle_line, = ax.plot([], [])
    acc_line, = ax.plot([], [])
    velocity_line, = ax.plot([], [])
    intersection_points = ax.scatter([], [], color='red', s=5)
    text = ax.text(0, 80, vehicle.error)

    def init():
        return lines + [vehicle_line, acc_line, velocity_line, intersection_points, text]

    def animate(i):
        vehicle_line.set_data(arrays['vehicle'][i])
        velocity_line.set_data(arrays['velocity'][i])
        acc_line.set_data(arrays['acc'][i])
        intersection_points.set_offsets(arrays['intersection_points'][i])

        text.set_text(arrays['text'][i])

        if i % 30 == 0:
            trail = ax.scatter([], [], color='grey', s=0.3)
            trail.set_offsets(arrays['trail'][i])
            return vehicle_line, velocity_line, acc_line, intersection_points, trail, text
        return vehicle_line, velocity_line, acc_line, intersection_points, text

    print("Simulation is being created...")
    anim = FuncAnimation(fig, animate, init_func=init, frames=frames, blit=True)
    anim.save(fn, writer='ffmpeg', fps=60)
    print("SIMULATION COMPLETE")


my_dcel = build_dcel_from_file()
# my_dcel.show_dcel()
frames = 5000
arrays = compute_arrays(my_dcel, vehicle, frames)
simulate(my_dcel, vehicle, frames=frames, arrays=arrays, fn="simulation26.mp4")
