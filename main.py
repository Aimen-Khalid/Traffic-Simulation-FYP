import math
from math import sqrt
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
ROAD_COLOR = 'white'
NON_ROAD_COLOR = 'lightgrey'
TRAIL_COLOR = 'grey'

vehicle = car.Vehicle(length=10, width=5, centroid=ScaledPoint(200, 200),
                      angle=90, velocity=ScaledPoint(12, -12), acceleration=ScaledPoint(0, 0))
# (50, 5), -90
initial_speed = vehicle.velocity.norm()


def write_vertices_to_file(vertices):
    with open("vertices1.txt", "w") as file:
        for vertex in vertices:
            for coordinate in vertex:
                file.write(str(coordinate) + "\t")
            file.write("\n")


def load_vertices_from_file():
    vertices = []
    with open("vertices1.txt", "r") as file:
        for line in file:
            vertex = tuple(int(x) for x in line.strip().split("\t"))
            vertices.append(vertex)
    return vertices


def write_segments_to_file(segments):
    with open("segments1.txt", "w") as file:
        for segment in segments:
            for vertices in segment:
                for vertex in vertices:
                    file.write(str(vertex) + "\t")
            file.write("\n")


def load_segments_from_file():
    segments = []
    with open("segments1.txt", "r") as file:
        for line in file:
            vertices = [int(x) for x in line.strip().split("\t")]
            segment = [(vertices[0], vertices[1]), (vertices[2], vertices[3])]
            segments.append(segment)
    return segments


def point_lies_left(point, line_start, line_end):
    dx = line_end[0] - line_start[0]
    dy = line_end[1] - line_start[1]
    dx1 = point[0] - line_start[0]
    dy1 = point[1] - line_start[1]
    cross_product = dx * dy1 - dy * dx1
    return cross_product > 0 or cross_product >= 0


def get_vertices_and_segments():
    fig, ax = plt.subplots(figsize=(100, 100))
    ax.set_aspect('equal')

    ax.axis([0, 50, 0, 50])
    ax.xaxis.set_ticks(range(0, 900, 30))
    ax.yaxis.set_ticks(range(0, 700, 30))
    ax.grid(color='green', linestyle='-', linewidth=0.5)

    prev_x = 0
    prev_y = 0

    vertices = []
    segments = []
    first = True
    grid_spacing = 30
    while True:
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


def draw_road_network():
    vertices, segments = get_vertices_and_segments()
    dcel_obj = dcel.Dcel()
    dcel_obj.build_dcel(vertices, segments)
    dcel_obj.show_dcel()
    write_vertices_to_file(vertices)
    write_segments_to_file(segments)


def draw_road_network_primary():
    vertices, segments = get_vertices_and_segments()
    dcel_obj = dcel.Dcel()
    dcel_obj.build_dcel_primary(vertices, segments)
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


def get_euclidean_distance(point1, point2):
    return math.sqrt((point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2)


def current_face_intersection_points(vehicle):
    intersection_points = []

    half_edges = vehicle.current_face.get_face_hedges()
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
    closest_point_1 = (0, 0)
    closest_point_2 = (0, 0)

    vehicle_x, vehicle_y = vehicle.get_car_mid_point()
    line_start = (vehicle_x, vehicle_y)
    line_end = (vehicle.velocity.get_x() + vehicle_x, vehicle.velocity.get_y() + vehicle_y)

    min_distance_1 = 999999999
    min_distance_2 = 999999999

    intersection_points = current_face_intersection_points(vehicle)

    for point in intersection_points:
        distance = sqrt((vehicle_x - point[0]) ** 2 + (vehicle_y - point[1]) ** 2)
        if point_lies_left(point, line_start, line_end):
            if distance < min_distance_1:
                min_distance_1 = distance
                closest_point_1 = point
        elif distance < min_distance_2:
            min_distance_2 = distance
            closest_point_2 = point
    return [closest_point_1, closest_point_2]


def get_error(vehicle, intersection_points):
    face_mid_point_x, face_mid_point_y = vehicle.face_mid_point

    segment = vehicle.get_car_perpendicular_line()

    vehicle_x = (segment[0][0] + segment[1][0]) / 2
    vehicle_y = (segment[0][1] + segment[1][1]) / 2

    a = get_euclidean_distance((intersection_points[0][0], intersection_points[0][1]), (vehicle_x, vehicle_y))
    b = get_euclidean_distance((intersection_points[1][0], intersection_points[1][1]), (vehicle_x, vehicle_y))

    return (b - a) / 2


def get_vehicle_front_point(vehicle):
    segment = vehicle.get_car_perpendicular_line()
    return Point((segment[0][0] + segment[1][0]) / 2, (segment[0][1] + segment[1][1]) / 2)


def get_projected_point(vehicle):
    vehicle_point = get_vehicle_front_point(vehicle)
    projected_point_distance = vehicle.reference_track.project(vehicle_point)
    return vehicle.reference_track.interpolate(projected_point_distance)


def get_track_segment(vehicle):
    projected_point = get_projected_point(vehicle)
    coords = vehicle.reference_track.coords
    for i in range(len(coords) - 1):
        if LineString([coords[i], coords[i + 1]]).distance(projected_point) < 1:
            return Point(coords[i][0], coords[i][1]), Point(coords[i + 1][0], coords[i + 1][1])


def get_error_from_track(vehicle):
    vehicle_point = get_vehicle_front_point(vehicle)
    segment_point_1, segment_point_2 = get_track_segment(vehicle)

    # translating the points to origin
    error = vehicle_point.distance(vehicle.reference_track)
    vehicle_point = Point(vehicle_point.x - segment_point_1.x, vehicle_point.y - segment_point_1.y)
    segment_point_2 = Point(segment_point_2.x - segment_point_1.x, segment_point_2.y - segment_point_1.y)

    cross_product = segment_point_2.x * vehicle_point.y - segment_point_2.y * vehicle_point.x

    return error * (1 if cross_product > 0 else -1)


def build_dcel_from_file():
    vertices = load_vertices_from_file()
    segments = load_segments_from_file()
    road_network = dcel.Dcel()
    road_network.build_dcel(vertices, segments)
    return road_network


def build_dcel_from_file_primary():
    vertices = load_vertices_from_file()
    segments = load_segments_from_file()
    road_network = dcel.Dcel()
    road_network.build_dcel_primary(vertices, segments)
    return road_network

def compute_parameters1(road_network, vehicle, frames): # face mid point
    print("Computation in progress...")
    start = time.time()

    parameters = {
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
    with tqdm(total=frames) as pbar:
        for _ in range(frames):
            vehicle_x, vehicle_y = vehicle.get_car_mid_point()

            parameters['trail_x'].append(vehicle_x)
            parameters['trail_y'].append(vehicle_y)

            vehicle.current_face = road_network.get_face_for_point((vehicle_x, vehicle_y))

            if vehicle.current_face is None:
                vehicle.current_face = vehicle.prev_face
            if vehicle.current_face is not None:

                vehicle.prev_face = vehicle.current_face

                intersection_points_list = get_closest_intersection_points(vehicle)

                vehicle.face_mid_point = get_mid_point([(intersection_points_list[0][0], intersection_points_list[0][1]),
                                                        (intersection_points_list[1][0], intersection_points_list[1][1])])

                vehicle.prev_error = vehicle.error
                vehicle.error = get_error(vehicle, intersection_points_list)
                # vehicle.error = get_error_from_track(vehicle)
                vehicle.update_state_vars()

                text = (f'acc: {str(vehicle.acc.norm())}'
                                + '\ntheta: ' + str(vehicle.theta)
                                # + '\nVelocity: ' + str(vehicle.velocity.norm())
                                + '\nerror: ' + str(vehicle.error)
                                # + '\nframe: ' + str(_)
                        )
                parameters['text'].append(text)

                x, y = vehicle.get_xy_lists()
                parameters['vehicle'].append((x, y))

                x = [vehicle_x, vehicle.velocity.get_x() + vehicle_x]
                y = [vehicle_y, vehicle.velocity.get_y() + vehicle_y]
                parameters['velocity'].append((x, y))

                x = [vehicle_x, vehicle.acc.get_x() + vehicle_x]
                y = [vehicle_y, vehicle.acc.get_y() + vehicle_y]
                parameters['acc'].append((x, y))
                parameters['acc_magnitude'].append(vehicle.acc_magnitude)
                parameters['speed'].append(vehicle.velocity.norm())
                parameters['error'].append(vehicle.error)

                intersection_points_list.append((vehicle.face_mid_point[0], vehicle.face_mid_point[1]))
                parameters['intersection_points'].append(intersection_points_list)
            pbar.update(1)

    end = time.time()
    print(f"{int(end - start)} Seconds")

    return parameters


def compute_parameters(road_network, vehicle, frames): # reference track
    print("Computation in progress...")
    start = time.time()

    parameters = {
        'vehicle': [],
        'velocity': [],
        'speed': [],
        'acc': [],
        'acc_magnitude': [],
        'error': [],
        'text': [],
        'trail_x': [],
        'trail_y': [],
        'projected_point':[]
    }
    with tqdm(total=frames) as pbar:
        for _ in range(frames):
            vehicle_x, vehicle_y = vehicle.get_car_mid_point()

            parameters['trail_x'].append(vehicle_x)
            parameters['trail_y'].append(vehicle_y)

            vehicle.prev_error = vehicle.error
            vehicle.error = get_error_from_track(vehicle)
            projected_point = get_projected_point(vehicle)
            vehicle.update_state_vars()

            text = (f'acc: {str(vehicle.acc.norm())}'
                            # + '\nvehicle: ' + str(get_vehicle_front_point(vehicle))
                            # + '\nprojected_point: ' + str(get_projected_point(vehicle))
                            + '\nframes: ' + str(_)
                            + '\nerror: ' + str(vehicle.error)
                    )
            parameters['text'].append(text)

            x, y = vehicle.get_xy_lists()
            parameters['vehicle'].append((x, y))

            x = [vehicle_x, vehicle.velocity.get_x() + vehicle_x]
            y = [vehicle_y, vehicle.velocity.get_y() + vehicle_y]
            parameters['velocity'].append((x, y))

            x = [vehicle_x, vehicle.acc.get_x() + vehicle_x]
            y = [vehicle_y, vehicle.acc.get_y() + vehicle_y]
            parameters['acc'].append((x, y))
            parameters['acc_magnitude'].append(vehicle.acc_magnitude)
            parameters['speed'].append(vehicle.velocity.norm())
            parameters['error'].append(vehicle.error)
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
        if face.name == 'f2':
            ax.fill(x, y, ec=ROAD_EDGE_COLOR, fc=ROAD_COLOR)
        else:
            ax.fill(x, y, ec=ROAD_EDGE_COLOR, fc=NON_ROAD_COLOR)
    x, y = vehicle.reference_track.xy
    ax.plot(x, y)


def plot_parameters(start, end):
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
    ax1.axhline(y=-1*vehicle.acc_limit, color='blue')
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

    fig.savefig(f"p{car.P_ACC_WEIGHT} d {car.D_ACC_WEIGHT} speed {initial_speed} theta {vehicle.theta}plot.png")


def simulate1(road_network, vehicle, frames, parameters, file_name): # face mid point
    fig = plt.figure()
    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box')

    draw_roads(road_network, ax)

    vehicle_line, = ax.plot([], [])
    trail_line, = ax.plot([], [], color=TRAIL_COLOR)
    ideal_trail_line, = ax.plot([], [], color='green')

    ideal_trail_line.set_data([parameters['intersection_points'][j][2][0] for j in range(5, len(parameters['intersection_points']))],
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


def run():
    draw_road_network()
    road_network = build_dcel_from_file()
    road_network.show_dcel()
    frames = 4000
    # parameters = compute_parameters(road_network, vehicle, frames)
    # write_to_file(parameters)
    #
    # # simulation will start after closing the plot figure
    # plot_parameters(start=5, end=4000)
    #
    # simulate(road_network, vehicle, frames=frames, parameters=parameters,
    #         file_name=f"p{car.P_ACC_WEIGHT} d {car.D_ACC_WEIGHT} speed {initial_speed} theta {vehicle.theta}.mp4")


# run()
# winsound.Beep(frequency=2500, duration=1000)


def angle_between_segments(seg1, seg2):
    # Compute the vectors for the two segments
    A = seg1[0]
    B = seg1[1]
    C = seg2[0]
    D = seg2[1]
    v1 = (B.x-A.x, B.y-A.y)
    v2 = (D.x-C.x, D.y-C.y)

    # Compute the angle between the two vectors
    dot_product = v1[0]*v2[0] + v1[1]*v2[1]
    mag_v1 = math.sqrt(v1[0]**2 + v1[1]**2)
    mag_v2 = math.sqrt(v2[0]**2 + v2[1]**2)
    cos_angle = dot_product / (mag_v1 * mag_v2)
    angle = math.acos(cos_angle)

    # Determine the sign of the angle based on the direction of the turn
    cross_product = v1[0]*v2[1] - v1[1]*v2[0]
    if cross_product > 0:
        angle = 2*math.pi-angle

    # Convert the angle to degrees and return it
    return math.degrees(angle)


def rotate_line(start_point, end_point, theta):  # clockwise
    # Convert the angle from degrees to radians
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


def map_overlay():
    lane_width = 30
    # draw_road_network_primary()
    road_network = build_dcel_from_file_primary()
    vertices = []
    segments = []
    for vertex in road_network.vertices_map.values():
        hedges = road_network.hedges_map.get_outgoing_hedges_clockwise(vertex)
        if len(hedges) < 1:
            continue
        if len(hedges) == 1:
            segments.append(translate_segment((hedges[0].origin, hedges[0].destination), lane_width))
            segments.append(translate_segment((hedges[0].origin, hedges[0].destination), lane_width, anticlockwise=True))
            vertices.append((segments[0][0][0], segments[0][0][1]))
            vertices.append((segments[0][1][0], segments[0][1][1]))

            vertices.append((segments[1][0][0], segments[1][0][1]))
            vertices.append((segments[1][1][0], segments[1][1][1]))
            continue

        translated_segments_cw = []
        translated_segments_acw = []

        for i, hedge in enumerate(hedges):
            if i + 1 == len(hedges):
                i = -1
            if i == -1:
                i = len(hedges) - 1
            translated_segments_cw.append(translate_segment((hedge.origin, hedge.destination), 30))
            translated_segments_acw.append(translate_segment((hedge.origin, hedge.destination), 30, anticlockwise=True))

        for i, hedge in enumerate(hedges):
            print(i)
            if i + 1 == len(hedges):
                segment1 = translated_segments_acw[0]
            else:
                segment1 = translated_segments_acw[i + 1]
            segment2 = translated_segments_cw[i]

            segment2[0] = tuple(get_intersection_point(segment1, segment2))
            segment1[0] = tuple(get_intersection_point(segment1, segment2))
            vertices.append((segment2[0][0], segment2[0][1]))
            vertices.append((segment2[1][0], segment2[1][1]))

            vertices.append((segment1[1][0], segment1[1][1]))

            segments.append(segment2)
            segments.append(segment1)

    fig, ax = plt.subplots()
    for segment in segments:
        start, end = segment
        ax.plot([start[0], end[0]], [start[1], end[1]], 'b-')
    plt.show()

    dcel_obj = dcel.Dcel()
    dcel_obj.build_dcel(vertices, segments)
    dcel_obj.show_dcel()

# map_overlay()


def get_angle(edge):
    origin, destination = edge[0], edge[1]
    dx = destination[0] - origin[0]
    dy = destination[1] - origin[1]
    length = math.sqrt(dx * dx + dy * dy)
    if dy > 0:
        return math.acos(dx / length)
    else:
        return 2 * math.pi - math.acos(dx / length)


def get_edges_cw(graph, node):
    outgoing_edges = list(graph.edges(node))
    outgoing_edges.sort(key=lambda e: get_angle(e), reverse=True)
    return outgoing_edges


# draw_road_network_primary()
G = nx.DiGraph()
# Add vertices to the graph
vertices_ = load_vertices_from_file()
segments_ = load_segments_from_file()
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
nx.draw(G, pos=node_positions, node_size=20, font_size=8, with_labels=True)
plt.show()

# Print the outgoing edges
# print(outgoing_edges)


def graph_to_lanes(G):
    vertices = []
    segments = []
    partitions = []

    lane_width = 30
    for node in G.nodes:
        if G.degree(node) == 2:
            outgoing_edge = list(G.edges(node))[0]
            if not G.get_edge_data(outgoing_edge[0], outgoing_edge[1])['cw_start']:
                cw_start = translate_segment(outgoing_edge, lane_width)[0]
                acw_start = translate_segment(outgoing_edge, lane_width, anticlockwise=True)[0]
                if G.get_edge_data(outgoing_edge[0], outgoing_edge[1])['cw_start'] is None:
                    nx.set_edge_attributes(G, {
                        (outgoing_edge[0], outgoing_edge[1]): {'cw_start': cw_start, 'acw_start': acw_start}})
                    nx.set_edge_attributes(G, {
                        (outgoing_edge[1], outgoing_edge[0]): {'acw_end': cw_start, 'cw_end': acw_start}})
            continue
        translated_segments_cw = []
        translated_segments_acw = []

        edges = get_edges_cw(G, node)
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
                segment4 = translated_segments_cw[len(edges)-1]
            else:
                segment4 = translated_segments_cw[i - 1]
            cw_start = tuple(get_intersection_point(segment1, segment2))
            acw_start = tuple(get_intersection_point(segment3, segment4))
            if G.get_edge_data(edge[0], edge[1])['cw_start'] is None:
                nx.set_edge_attributes(G, {(edge[0], edge[1]): {'cw_start': cw_start, 'acw_start': acw_start}})
                nx.set_edge_attributes(G, {(edge[1], edge[0]): {'acw_end': cw_start, 'cw_end': acw_start}})
            # vertices.extend((cw_start, acw_start))
    for node in G.nodes:
        edges = get_edges_cw(G, node)
        for edge in edges:
            if G.get_edge_data(edge[0], edge[1])['cw_end'][0] is not None:
                continue
            neighbor_edges = get_edges_cw(G, edge[1])
            if len(neighbor_edges) == 1:
                cw_end = translate_segment(edge, lane_width)[1]
                acw_end = translate_segment(edge, lane_width, anticlockwise=True)[1]
                nx.set_edge_attributes(G, {(edge[0], edge[1]): {'cw_end': cw_end, 'acw_end': acw_end}})
                nx.set_edge_attributes(G, {(edge[1], edge[0]): {'acw_start': cw_end, 'cw_start': acw_end}})
                continue
            # u, v = edge[0], edge[1]
            # edge = (v, u)
            index = neighbor_edges.index(edge)
            prev = len(neighbor_edges) - 1 if index == 0 else index - 1
            cw_end = G.get_edge_data(neighbor_edges[prev][0], neighbor_edges[prev][1])['cw_start']
            next_ = 0 if index == len(neighbor_edges) - 1 else index + 1
            acw_end = G.get_edge_data(neighbor_edges[next_][0], neighbor_edges[next_][1])['acw_start']
            if G.get_edge_data(edge[0], edge[1])['cw_end'] is None:
                nx.set_edge_attributes(G, {(edge[0], edge[1]): {'cw_end': cw_end, 'acw_end': acw_end}})
                nx.set_edge_attributes(G, {(edge[1], edge[0]): {'acw_start': cw_end, 'cw_start': acw_end}})
            # vertices.extend((cw_end, acw_end))

    for node in G.nodes:
        edges = get_edges_cw(G, node)
        for edge in edges:
            if G.get_edge_data(edge[0], edge[1])['visited']:
                continue
            nx.set_edge_attributes(G, {(edge[0], edge[1]): {'visited': True}})
            nx.set_edge_attributes(G, {(edge[1], edge[0]): {'visited': True}})
            vertices.extend([edge[0], edge[1]])
            segments.extend(
                (
                    [
                        (G.get_edge_data(edge[0], edge[1])['cw_start']),
                        (G.get_edge_data(edge[0], edge[1])['cw_end']),
                    ],
                    [
                        (G.get_edge_data(edge[0], edge[1])['acw_start']),
                        (G.get_edge_data(edge[0], edge[1])['acw_end']),
                    ],
                )
            )
            partitions.extend(
                (
                    [
                        (G.get_edge_data(edge[0], edge[1])['cw_start']),
                        (G.get_edge_data(edge[0], edge[1])['acw_start']),
                    ],
                    [
                        (G.get_edge_data(edge[0], edge[1])['cw_end']),
                        (G.get_edge_data(edge[0], edge[1])['acw_end']),
                    ],)
            )
    return vertices, segments, partitions


vertices, segments, partitions = graph_to_lanes(G)
# print(segments)

fig, ax = plt.subplots()

for segment in segments:
    start, end = segment
    x_values = [start[0], end[0]]
    print(segment)
    y_values = [start[1], end[1]]
    # ax.text(start[0], start[1], f"{round(start[0])},{str(round(start[1]))}", fontsize=6, color='green')
    # ax.text(end[0], end[1], f"{round(end[0])},{str(round(end[1]))}", fontsize=6, color='green')
    ax.plot(x_values, y_values, color='green')

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



