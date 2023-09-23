import time
from tqdm import tqdm
import matplotlib.pyplot as plt
import matplotlib
import numpy as np
from RUSTIC.Utility import geometry
from matplotlib.animation import FuncAnimation
import math
import winsound
from shapely import LineString, Point
from RUSTIC.ObstacleAvoidance.obstacle_avoidance import obstacles

ROAD_EDGE_COLOR = 'black'
ROAD_COLOR = 'lightgrey'
NON_ROAD_COLOR = 'white'
TRAIL_COLOR = '#807E78'#'grey'

"""For testing this module, please see simulation_main in simulation_main.py of Main module"""


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
    # add_trail(vehicle, parameters)
    add_positions(vehicle, parameters)
    add_centroid(vehicle, parameters)


def compute_parameters(vehicles, road_network, frames):
    print("Computation in progress...")
    start = time.time()
    list_of_parameters_list = [
        initialize_parameters_dict() for _ in range(len(vehicles))
    ]
    # Use tqdm to display a progress bar while computing the parameters for each frame
    with tqdm(total=frames) as pbar:
        for _ in range(frames):
            for index, vehicle in enumerate(vehicles):
                vehicle.update_state_vars(road_network)
                update_parameters_list(vehicle, list_of_parameters_list[index])
            pbar.update(1)
    end = time.time()
    print(f"{int(end - start)} Seconds")

    return list_of_parameters_list


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
    # trail_lines = []

    for _ in range(no_of_vehicles):
        vehicle_line, = ax.fill([], [], color='red', edgecolor='black', zorder=13)

        # car_image = plt.imread('car_image.png')

        # vehicle_line = BboxImage(ax.bbox, norm=None, origin=None, clip_on=False)
        # vehicle_line.set_data(car_image)
        # ax.add_artist(vehicle_line)

        # trail_line, = ax.plot([], [], color=TRAIL_COLOR, linewidth=0.5)

        vehicle_lines.append(vehicle_line)
        # trail_lines.append(trail_line)

    return vehicle_lines#, trail_lines


def simulate(road_network, vehicles, frames, parameters_list, file_name, with_road_network):  # reference track
    matplotlib.use('Agg')

    screen_resolution = (1920, 1080)
    fig = plt.figure(figsize=(screen_resolution[0] / 100, screen_resolution[1] / 100), dpi=180)

    plt.subplots_adjust(left=0, right=1, top=1, bottom=0)

    ax = plt.gca()
    ax.axis("off")
    ax.set_aspect('equal', adjustable='box')
    # fig.set_facecolor('#74663b')

    if with_road_network:
        road_network.show_road_network(ax)

    for obstacle in obstacles:
        x, y = obstacle.exterior.xy
        ax.fill(x, y, color='blue', zorder=12)
        # buffer = obstacle.centroid.buffer(6)
        # x, y = buffer.exterior.xy
        # ax.plot(x, y, color='yellow', linewidth=0.6)

    # for vehicle in vehicles:
    #     x, y = vehicle.reference_track.xy
    #     ax.plot(x, y, linewidth=2, color='darkgreen', zorder=10)##0ccb36

    # vehicle_lines, trail_lines = get_artist_objects(ax, len(vehicles))
    vehicle_lines = get_artist_objects(ax, len(vehicles))

    def animate(i):
        for j in range(len(vehicles)):
            parameters = parameters_list[j]
            x, y = parameters['vehicle'][i]

            # ax.images[0].set_extent([0 + i, 4 + i, 0 + i, 2 + i])
            # vehicle_lines[j].set_data(plt.imread('car_image.png'))
            # vehicle_lines[j].set_extent([x - car_width / 2, x + car_width / 2, y - car_height / 2, y + car_height / 2])

            vehicle_lines[j].set_xy(list(zip(x, y)))
            # vehicle_lines[j].set_data(plt.imread('car_image.png'))

            # extent = [min(x), max(x), min(y), max(y)]
            # vehicle_lines[j].set_offset(Bbox.from_extents(*extent))

            # trail_lines[j].set_data(parameters['trail_x'][:i], parameters['trail_y'][:i])

            x_min, x_max = ax.get_xlim()
            y_min, y_max = ax.get_ylim()
            center_x = (x_min + x_max) / 2
            center_y = (y_min + y_max) / 2
            window_size = 120
            ax.set_xlim(center_x - window_size, center_x + window_size)
            ax.set_ylim(center_y - window_size, center_y + window_size)

            #  uncomment the next two lines to have a zoomed in simulation. Adjust window_size variable.

            # ax.set_xlim(parameters['centroid'][i].x - window_size, parameters['centroid'][i].x + window_size)
            # ax.set_ylim(parameters['centroid'][i].y - window_size, parameters['centroid'][i].y + window_size)

        return_string = ""
        for i in range(len(vehicles)):
            return_string += f"vehicle_lines[{i}], "
            # return_string += f"trail_lines[{i}], "

        return eval(return_string)

    print("Animation in progress...")
    start = time.time()
    anim = FuncAnimation(fig, animate, frames=frames, blit=True)
    anim.save(file_name, writer='ffmpeg', fps=240)  # speed of simulation increases with increasing fps
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


def create_simulation_video(vehicles, road_network, frames, with_road_network):
    parameters_list = compute_parameters(vehicles, road_network, frames)
    plt.close('all')
    # write_parameters_to_file(parameters)

    # file_name = f"p{car.params['P_PER_ACC_WEIGHT']} " \
    #             f"d {car.params['D_PER_ACC_WEIGHT']} " \
    #             f"acc_lim {vehicle.acc_limit} " \
    #             f"init_speed {round(vehicle.initial_speed, 2)} " \
    #             f"dec {car.params['P_PARALLEL_DEC_WEIGHT']} threshold {car.params['angle_threshold']}" \
    #             f"lookahead_time {vehicle.turning_lookahead.time}" \
    #             f".mp4"
    file_name = "anothervideotest.mp4"
    simulate(road_network, vehicles, frames, parameters_list, file_name, with_road_network)
    winsound.Beep(frequency=2500, duration=1000) #  a beep sound to indicate simulation video is completed
    # plot_parameters(vehicle, fig_name=file_name)


def plot_vehicle_tracking(vehicle, frames, road_network=None):
    parameters_list = compute_parameters(vehicle, road_network, frames)
    plt.close('all')
    fig = plt.figure()
    ax = plt.gca()
    ax.axis("off")
    ax.set_aspect('equal', adjustable='box')


    parameters = parameters_list[i]

    trail_x = parameters['trail_x']
    trail_y = parameters['trail_y']
    trail = list(zip(trail_x, trail_y))
    tracking_metric_index = calculate_tracking_accuracy(trail, vehicle.reference_track)
    print("Tracking metric index: ", tracking_metric_index)
    ax.plot(trail_x, trail_y, linewidth=1, color=vehicle.trail_color, antialiased=True)
    track_x, track_y = vehicle.reference_track.xy
    ax.plot(track_x, track_y, linewidth=1, color=vehicle.track_color, antialiased=True)

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
        road_network.extract_roads_from_map_region(ax)
    # file_name = f"{description} tracking.png"
    # fig.savefig(file_name)

    plt.show()
