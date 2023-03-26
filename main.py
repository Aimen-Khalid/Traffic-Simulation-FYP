import math
from math import sqrt
import dcel
import car
from point import ScaledPoint
import matplotlib.pyplot as plt
from car import point_lies_left
from matplotlib.animation import FuncAnimation
import numpy as np
import time
from tqdm import tqdm

ROAD_EDGE_COLOR = 'black'
ROAD_COLOR = 'white'
NON_ROAD_COLOR = 'lightgrey'
TRAIL_COLOR = 'grey'

vehicle = car.Vehicle(length=20, width=10, speed_limit=0, acc_limit=0, centroid=ScaledPoint(420, 50),
                      angle=5, v=ScaledPoint(50, 5), a=ScaledPoint(0, 0))


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


def get_error(vehicle):
    mid_point_x, mid_point_y = vehicle.face_mid_point

    segment = vehicle.get_car_perpendicular_line()

    vehicle_x = (segment[0][0] + segment[1][0]) / 2
    vehicle_y = (segment[0][1] + segment[1][1]) / 2

    distance = get_euclidean_distance((mid_point_x, mid_point_y), (vehicle_x, vehicle_y))

    if point_lies_left((mid_point_x, mid_point_y), segment[0], segment[1]):
        distance *= -1

    return distance


def build_dcel_from_file():
    vertices = load_vertices_from_file()
    segments = load_segments_from_file()
    road_network = dcel.Dcel()
    road_network.build_dcel(vertices, segments)
    return road_network


def compute_parameters(road_network, vehicle, frames):
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
                vehicle.error = get_error(vehicle)

                dist = 0
                dist2 = 0

                if len(intersection_points_list) > 2:
                    dist = math.sqrt((vehicle_x - intersection_points_list[0][0])**2 +
                                    (vehicle_y - intersection_points_list[0][1])**2)
                    dist2 = math.sqrt((vehicle_x - intersection_points_list[1][0])**2 +
                                    (vehicle_y - intersection_points_list[1][1])**2)

                vehicle.update_state_vars(dist, dist2)

                text = (f'acc: {str(vehicle.acc.norm())}'
                                + '\ntheta: ' + str(vehicle.theta)
                                + '\nVelocity: ' + str(vehicle.velocity.norm())
                                + '\nDist: ' + str(dist)
                                + '\nDist2: ' + str(dist2))
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
                parameters['speed'].append(vehicle.speed)
                parameters['error'].append(vehicle.error)

                intersection_points_list.append((vehicle.face_mid_point[0], vehicle.face_mid_point[1]))
                parameters['intersection_points'].append(intersection_points_list)
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
    # ax1.set_ylim([-250, 250])

    ax2.plot(frames, error)
    ax2.set_ylabel('Error')
    ax2.set_ylim([-20, 20])

    ax3.plot(frames, speed)
    ax3.set_ylabel('Speed')
    ax3.set_xlabel('Frames')

    fig.suptitle(f"kp={car.P_ACC_WEIGHT} kd={car.D_ACC_WEIGHT}")

    # Adjust the spacing between subplots
    fig.tight_layout()

    plt.show()

    fig.savefig(f"p{car.P_ACC_WEIGHT} d {car.D_ACC_WEIGHT} plot.png")


def simulate(road_network, vehicle, frames, parameters, file_name):
    fig = plt.figure()
    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box')

    draw_roads(road_network, ax)

    vehicle_line, = ax.plot([], [])
    trail_line, = ax.plot([], [], color=TRAIL_COLOR)
    ideal_trail_line, = ax.plot([], [], color='green')

    ideal_trail_line.set_data([parameters['intersection_points'][j][2][0] for j in range(len(parameters['intersection_points']))],
                              [parameters['intersection_points'][j][2][1] for j in range(len(parameters['intersection_points']))])
    acc_line, = ax.plot([], [])
    velocity_line, = ax.plot([], [])
    intersection_points = ax.scatter([], [], color='red', s=5)
    text = ax.text(0, 80, vehicle.error)

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
    road_network = build_dcel_from_file()

    frames = 4000
    parameters = compute_parameters(road_network, vehicle, frames)
    write_to_file(parameters)

    # simulation will start after closing the plot figure
    plot_parameters(start=5, end=1000)

    simulate(road_network, vehicle, frames=frames, parameters=parameters, file_name="simulation.mp4")  #fn=f"p{car.P_ACC_WEIGHT} d {car.D_ACC_WEIGHT}.mp4")


run()
