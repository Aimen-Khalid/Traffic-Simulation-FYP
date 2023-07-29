from CarModel import car
import time
from tqdm import tqdm
import matplotlib.pyplot as plt
import numpy as np
from Utility import geometry
from matplotlib.animation import FuncAnimation
import matplotlib
import math
import winsound
from matplotlib.patches import Circle
from shapely import LineString, Point
from MapToRoadMapping import dcel

ROAD_EDGE_COLOR = 'black'
ROAD_COLOR = 'lightgrey'
NON_ROAD_COLOR = 'white'
TRAIL_COLOR = 'grey'

"""For testing this module, please see simulation_main in main.py of Main module"""


def initialize_parameters_dict():
    return {
        'vehicle': [],  # The x and y coordinates of four points of vehicle
        'centroid': [],
        'velocity': [],
        'speed': [],
        'perpendicular_acc': [],
        'parallel_acc': [],
        'perpendicular_acc_magnitude': [],
        'parallel_acc_magnitude': [],
        'resultant_acc_magnitude': [],
        'turning_error': [],
        'speed_error': [],
        'text': [],
        'trail_x': [],
        'trail_y': [],
        'turning_lookahead_point': [],
        'speed_lookahead_point': [],
        'closest_point': [],
        'headed_vector': [],
        'speed_track_vector': [],
        'turning_track_vector': [],
        'turning_circle': [],
        'speed_circle': []
    }


def add_trail(vehicle, parameters):
    vehicle_x, vehicle_y = vehicle.front_mid_point.x, vehicle.front_mid_point.y
    parameters['trail_x'].append(vehicle_x)
    parameters['trail_y'].append(vehicle_y)


# def add_text(vehicle, parameters):
#     text = (
#         # f'per acc: {str(round(vehicle.perpendicular_acc.norm(), 2))}' + ' ms2\n' +
#         f'parallel acc: {str(vehicle.parallel_acc.norm() * vehicle.parallel_acc_sign)}' + ' ms2'
#         # + '\nangle: '
#         # + str(round(math.degrees(vehicle.error), 2)) + ' degrees' +
#             '\nspeed: '
#             + str(f'{vehicle.velocity.norm()}') + ' ms'
#         # + '\nerror: '
#         # + str(f'{round(vehicle.error, 2)}')
#     )
#     parameters['text'].append(text)


def add_positions(vehicle, parameters):
    x, y = vehicle.get_xy_lists()
    parameters['vehicle'].append((x, y))


def add_velocity(vehicle, parameters):
    # Translate velocity to vehicle position
    x = [vehicle.front_mid_point.x, vehicle.velocity.x + vehicle.front_mid_point.x]
    y = [vehicle.front_mid_point.y, vehicle.velocity.y + vehicle.front_mid_point.y]
    parameters['velocity'].append((x, y))


def add_perpendicular_acc(vehicle, parameters):
    x = [vehicle.front_mid_point.x, vehicle.perpendicular_acc.x + vehicle.front_mid_point.x]
    y = [vehicle.front_mid_point.y, vehicle.perpendicular_acc.y + vehicle.front_mid_point.y]
    parameters['perpendicular_acc'].append((x, y))


def add_parallel_acc(vehicle, parameters):
    x = [vehicle.front_mid_point.x, vehicle.parallel_acc.x + vehicle.front_mid_point.x]
    y = [vehicle.front_mid_point.y, vehicle.parallel_acc.y + vehicle.front_mid_point.y]
    parameters['parallel_acc'].append((x, y))


def add_resultant_acc(vehicle, parameters):
    resultant_acc = geometry.keep_in_limit(vehicle.parallel_acc + vehicle.perpendicular_acc, vehicle.acc_limit)
    parameters['resultant_acc_magnitude'].append(resultant_acc.norm())


def add_acc_magnitudes(vehicle, parameters):
    parameters['perpendicular_acc_magnitude'].append(vehicle.perpendicular_acc.norm())
    parameters['parallel_acc_magnitude'].append(vehicle.parallel_acc.norm())


def add_speed(vehicle, parameters):
    parameters['speed'].append(vehicle.velocity.norm())


def add_errors(vehicle, parameters):
    parameters['speed_error'].append(vehicle.speed_lookahead.error)
    parameters['turning_error'].append(vehicle.turning_lookahead.error)


def add_track_vectors(vehicle, parameters):
    vehicle_point = (vehicle.front_mid_point.x, vehicle.front_mid_point.y)
    speed_lookahead_point = (vehicle.speed_lookahead.point.x, vehicle.speed_lookahead.point.y)
    turning_lookahead_point = (vehicle.turning_lookahead.point.x, vehicle.turning_lookahead.point.y)
    parameters['speed_track_vector'].append((vehicle_point, speed_lookahead_point))
    parameters['turning_track_vector'].append((vehicle_point, turning_lookahead_point))


def add_projected_points(vehicle, parameters):
    parameters['turning_lookahead_point'].append(vehicle.turning_lookahead.point)
    parameters['speed_lookahead_point'].append(vehicle.speed_lookahead.point)
    parameters['closest_point'].append(vehicle.closest_point)


def add_centroid(vehicle, parameters):
    parameters['centroid'].append(vehicle.centroid)


def add_circles(vehicle, parameters):
    parameters['turning_circle'].append((vehicle.front_mid_point, vehicle.turning_lookahead.distance))
    parameters['speed_circle'].append((vehicle.front_mid_point, vehicle.speed_lookahead.distance))


def update_parameters_list(vehicle, parameters):
    add_trail(vehicle, parameters)
    # add_text(vehicle, parameters)
    add_positions(vehicle, parameters)
    add_velocity(vehicle, parameters)
    add_perpendicular_acc(vehicle, parameters)
    add_parallel_acc(vehicle, parameters)
    add_resultant_acc(vehicle, parameters)
    add_acc_magnitudes(vehicle, parameters)
    add_projected_points(vehicle, parameters)
    add_speed(vehicle, parameters)
    add_errors(vehicle, parameters)
    add_track_vectors(vehicle, parameters)
    add_centroid(vehicle, parameters)
    add_circles(vehicle, parameters)


def compute_parameters(vehicle, frames):
    print("Computation in progress...")
    start = time.time()
    parameters = initialize_parameters_dict()
    # Use tqdm to display a progress bar while computing the parameters for each frame
    with tqdm(total=frames) as pbar:
        for _ in range(frames):
            vehicle.update_state_vars()
            update_parameters_list(vehicle, parameters)
            pbar.update(1)
    end = time.time()
    print(f"{int(end - start)} Seconds")
    return parameters


def plot_parameter(ax, frames, parameter_list, parameter_name, limit=None):
    ax.plot(frames, parameter_list)
    ax.set_ylabel(parameter_name)
    if limit is None:
        return
    ax.axhline(y=limit, color='blue')
    ax.axhline(y=-1 * limit, color='blue')
    max_val = np.max(parameter_list[10:])
    min_val = np.min(parameter_list[10:])
    ax.text(0.95, 0.95, f"max = {max_val:.2f}\nmin = {min_val:.2f}", transform=ax.transAxes, ha='right',
            va='top', bbox=dict(facecolor='white', edgecolor='none', alpha=0.5))


def load_params():
    return {
        'perpendicular_acc': np.loadtxt("perpendicular_acc_magnitude.txt"),
        'parallel_acc': np.loadtxt("parallel_acc_magnitude.txt"),
        'resultant_acc': np.loadtxt("resultant_acc_magnitude.txt"),
        'speed': np.loadtxt("speed.txt"),
        'turning_error': np.loadtxt("turning_error.txt")
    }


def plot_parameters(vehicle, fig_name):
    params = load_params()
    fig, axs = plt.subplots(3, 2, figsize=(8, 10))
    axs = axs.flatten()
    frames = np.linspace(0, len(params['perpendicular_acc']), len(params['perpendicular_acc']))

    plot_parameter(axs[0], frames, params['perpendicular_acc'], "Perpendicular Acceleration", vehicle.acc_limit)
    plot_parameter(axs[1], frames, params['turning_error'], "Error", math.radians(vehicle.angle_threshold))
    plot_parameter(axs[2], frames, params['parallel_acc'], "Parallel Acceleration", vehicle.acc_limit)
    plot_parameter(axs[3], frames, params['speed'], "Speed")
    plot_parameter(axs[4], frames, params['resultant_acc'], "Resultant Acceleration", vehicle.acc_limit)

    axs[5].axis('off')

    fig.suptitle(fig_name)
    fig.tight_layout()
    fig.savefig(f"{fig_name} graph.png")
    plt.show()


def get_artist_objects(ax, no_of_vehicles):
    vehicle_lines = []
    trail_lines = []
    acc_lines = []
    acc2_lines = []
    velocity_lines = []
    closest_points = []
    turning_lookahead_points = []
    speed_lookahead_points = []
    turning_track_lines = []
    speed_track_lines = []
    turning_circles = []
    speed_circles = []
    # texts = []

    for i in range(no_of_vehicles):
        vehicle_line, = ax.fill([], [], color='white', edgecolor='black')
        trail_line, = ax.plot([], [], color=TRAIL_COLOR, linewidth=0.5)
        acc_line, = ax.plot([], [], linewidth=0.5, color='green')
        acc2_line, = ax.plot([], [], linewidth=0.5, color='green')
        velocity_line, = ax.plot([], [], linewidth=0.5, color='green')
        turning_lookahead_point = ax.scatter([], [], color='red', s=3)
        speed_lookahead_point = ax.scatter([], [], color='green', s=3)
        closest_point = ax.scatter([], [], color='black', s=3)
        text = ax.text(0, 0, "", fontsize=6)
        turning_track_line, = ax.plot([], [], linewidth=0.5, color='red')
        speed_track_line, = ax.plot([], [], linewidth=0.5, color='green')
        turning_circle = Circle((0.5, 0.5), radius=0.3, edgecolor='red', facecolor='none', linewidth=0.3)
        ax.add_patch(turning_circle)
        speed_circle = Circle((0.5, 0.5), radius=0.3, edgecolor='green', facecolor='none', linewidth=0.3)
        ax.add_patch(speed_circle)

        vehicle_lines.append(vehicle_line)
        trail_lines.append(trail_line)
        acc_lines.append(acc_line)
        acc2_lines.append(acc2_line)
        velocity_lines.append(velocity_line)
        closest_points.append(closest_point)
        turning_lookahead_points.append(turning_lookahead_point)
        speed_lookahead_points.append(speed_lookahead_point)
        turning_track_lines.append(turning_track_line)
        speed_track_lines.append(speed_track_line)
        turning_circles.append(turning_circle)
        speed_circles.append(speed_circle)
        # texts.append(text)

    return vehicle_lines, acc_lines, acc2_lines, velocity_lines, closest_points, turning_track_lines, speed_track_lines, \
        trail_lines, turning_lookahead_points, speed_lookahead_points, turning_circles, speed_circles#, texts


def separate_objects(my_list):
    yield from my_list


def simulate(road_network, vehicles, obstacles, frames, parameters_list, file_name, with_road_network):  # reference track
    matplotlib.use('Agg')

    screen_resolution = (1920, 1080)  # Example screen resolution (replace with your actual resolution)
    fig = plt.figure(figsize=(screen_resolution[0] / 100, screen_resolution[1] / 100), dpi=150)  # Adjust dpi if needed

    # Your road network plotting code here...

    # Remove margins and adjust the layout
    plt.subplots_adjust(left=0, right=1, top=1, bottom=0)
    ax = plt.gca()
    ax.axis("off")
    ax.set_aspect('equal', adjustable='box')

    if with_road_network:
        road_network.show_road_network(ax)

    for obstacle in obstacles:
        x, y = obstacle.exterior.xy
        ax.fill(x, y, color='#ff4d4d')
        # buffer = obstacle.centroid.buffer(8)
        # x, y = buffer.exterior.xy
        # ax.plot(x, y, color='yellow', linewidth=0.6)


    # x, y = vehicles[0].reference_track.xy
    # ax.plot(x, y, linewidth=1, color='green')

    vehicle_lines, acc_lines, acc2_lines, velocity_lines, closest_points, turning_track_lines, speed_track_lines, \
        trail_lines, turning_lookahead_points, speed_lookahead_points, turning_circles, speed_circles = get_artist_objects(ax, len(vehicles))

    def init():
        # return vehicle_lines[0], acc_lines[0], acc2_lines[0], velocity_lines[0], closest_points[0], turning_track_lines[0], speed_track_lines[0], \
        #     trail_lines[0], turning_lookahead_points[0], speed_lookahead_points[0], turning_circles[0], speed_circles[0]
        return vehicle_lines[0], acc_lines[0], acc2_lines[0], velocity_lines[0], trail_lines[0]

    def animate(i):
        for j in range(len(vehicles)):
            parameters = parameters_list[j]
            x, y = parameters['vehicle'][i]
            vehicle_lines[j].set_xy(list(zip(x, y)))
            velocity_lines[j].set_data(parameters['velocity'][i])
            trail_lines[j].set_data(parameters['trail_x'][:i], parameters['trail_y'][:i])
            acc_lines[j].set_data(parameters['perpendicular_acc'][i])
            acc2_lines[j].set_data(parameters['parallel_acc'][i])
            turning_lookahead_points[j].set_offsets((parameters['turning_lookahead_point'][i].x,
                                                     parameters['turning_lookahead_point'][i].y))
            speed_lookahead_points[j].set_offsets(
                (parameters['speed_lookahead_point'][i].x, parameters['speed_lookahead_point'][i].y))
            closest_points[j].set_offsets((parameters["closest_point"][i].x, parameters["closest_point"][i].y))
            turning_circles[j].set_radius(parameters["turning_circle"][i][1])
            turning_circles[j].set_center((parameters["turning_circle"][i][0].x, parameters["turning_circle"][i][0].y))
            speed_circles[j].set_radius(parameters["speed_circle"][i][1])
            speed_circles[j].set_center((parameters["speed_circle"][i][0].x, parameters["speed_circle"][i][0].y))

            start, end = parameters['turning_track_vector'][i]
            x = [start[0], end[0]]
            y = [start[1], end[1]]
            turning_track_lines[j].set_data(x, y)

            start, end = parameters['speed_track_vector'][i]
            x = [start[0], end[0]]
            y = [start[1], end[1]]
            speed_track_lines[j].set_data(x, y)

            # x = min(parameters['centroid'][i].get_x() for j in range(len(parameters['centroid'])))
            # y = max(parameters['centroid'][i].get_y() for j in range(len(parameters['centroid'])))
            # # texts[j].set_position((x, y))
            # texts[j].set_text(parameters['text'][i])

            window_size = 40
        # text.set_position((parameters['centroid'][i].get_x() - 1.75 * window_size, parameters['centroid'][i].get_y()))
        # x = min(parameters['centroid'][j].get_x() for j in range(len(parameters['centroid'])))
        # y = max(parameters['centroid'][j].get_y() for j in range(len(parameters['centroid'])))
        # text.set_position((x, y))
        # text.set_text(parameters['text'][i])

            ax.set_xlim(parameters['centroid'][i].get_x() - window_size, parameters['centroid'][i].get_x() + window_size)
            ax.set_ylim(parameters['centroid'][i].get_y() - window_size, parameters['centroid'][i].get_y() + window_size)

        return_string = ""
        for i in range(len(vehicles)):
            return_string += f"vehicle_lines[{i}], "
            return_string += f"velocity_lines[{i}], "
            return_string += f"acc_lines[{i}], "
            return_string += f"acc2_lines[{i}], "
            return_string += f"trail_lines[{i}], "
            return_string += f"turning_track_lines[{i}], "
            return_string += f"speed_track_lines[{i}], "
            return_string += f"turning_circles[{i}], "
            return_string += f"speed_circles[{i}], "

        return eval(return_string)

    print("Animation in progress...")
    start = time.time()
    anim = FuncAnimation(fig, animate, init_func=init, frames=frames, blit=True)
    anim.save(file_name, writer='ffmpeg', fps=240)
    end = time.time()
    print("Animation COMPLETED....")
    print(f"{int(end - start)} Seconds")


def write_parameter_to_file(arrays, parameter):
    with open(f"{parameter}.txt", "w") as file:
        for p in arrays[parameter]:
            file.write(f"{p}\n")


def write_parameters_to_file(arrays):
    write_parameter_to_file(arrays, "perpendicular_acc_magnitude")
    write_parameter_to_file(arrays, "parallel_acc_magnitude")
    write_parameter_to_file(arrays, "resultant_acc_magnitude")
    write_parameter_to_file(arrays, "speed")
    write_parameter_to_file(arrays, "turning_error")


def calculate_tracking_accuracy(vehicle_trail, track):
    total_distance = 0.0
    # Iterate over the vehicle coordinates
    for i in range(len(vehicle_trail) - 1):
        # Create Shapely Point object for the current point
        p = Point(vehicle_trail[i])

        # Calculate the closest point on the track to the current vehicle coordinate
        closest_point = track.interpolate(track.project(p))

        # Calculate the distance between the current vehicle coordinate and the closest point
        # on track
        distance = closest_point.distance(p)

        # Add the distance to the total
        total_distance += distance

    return total_distance


def create_simulation_video(vehicles, road_network, obstacles, frames, with_road_network):
    parameters_list = [compute_parameters(vehicle, frames) for vehicle in vehicles]
    plt.close('all')
    # write_parameters_to_file(parameters)

    # file_name = f"p{car.params['P_PER_ACC_WEIGHT']} " \
    #             f"d {car.params['D_PER_ACC_WEIGHT']} " \
    #             f"acc_lim {vehicle.acc_limit} " \
    #             f"init_speed {round(vehicle.initial_speed, 2)} " \
    #             f"dec {car.params['P_PARALLEL_DEC_WEIGHT']} threshold {car.params['angle_threshold']}" \
    #             f"lookahead_time {vehicle.turning_lookahead.time}" \
    #             f".mp4"
    file_name = "video.mp4"
    simulate(road_network, vehicles, obstacles, frames, parameters_list, file_name, with_road_network)
    winsound.Beep(frequency=2500, duration=1000)
    # plot_parameters(vehicle, fig_name=file_name)


def plot_vehicle_tracking(vehicles, frames, road_network=None):
    parameters_list = [compute_parameters(vehicle, frames) for vehicle in vehicles]
    plt.close('all')
    fig = plt.figure()
    ax = plt.gca()
    ax.axis("off")
    ax.set_aspect('equal', adjustable='box')

    for i in range(len(vehicles)):
        vehicle = vehicles[i]
        parameters = parameters_list[i]

        trail_x = parameters['trail_x']
        trail_y = parameters['trail_y']
        trail = list(zip(trail_x, trail_y))
        tracking_metric_index = calculate_tracking_accuracy(trail, vehicle.reference_track)
        print("Tracking metric index: ", tracking_metric_index)
        ax.plot(trail_x, trail_y, linewidth=1, color=vehicle.trail_color)
        track_x, track_y = vehicle.reference_track.xy
        ax.plot(track_x, track_y, linewidth=1, color=vehicle.track_color)

    # description = f"\nKp Per: {car.params['P_PER_ACC_WEIGHT']} " \
    #               f"   Kd per: {car.params['D_PER_ACC_WEIGHT']} " \
    #               f"   Kp parallel dec: {car.params['P_PARALLEL_DEC_WEIGHT']} " \
    #               f"   Kp parallel acc: {car.params['P_PARALLEL_ACC_WEIGHT']} " \
    #               f"\nAcc Limit: {vehicle.acc_limit} " \
    #               f"\nInit speed: {round(vehicle.initial_speed, 2)} " \
    #               f"   Goal speed: {vehicle.goal_speed} " \
    #               f"\nAngle threshold: {car.params['angle_threshold']}" \
    #               f"\nLook-ahead time: {vehicle.turning_lookahead.time}" \
    #               f"\nFrames: {frames}"
    # fig.suptitle(f"Tracking metric index: {str(round(tracking_metric_index, 2))}{description}", fontsize=8, x=0, ha='left')
    if road_network:
        road_network.show_road_network(ax)
    # file_name = f"{description} tracking.png"
    # fig.savefig(file_name)

    plt.show()