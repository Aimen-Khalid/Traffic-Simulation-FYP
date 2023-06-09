import car
import time
from tqdm import tqdm
import matplotlib.pyplot as plt
import numpy as np
import geometry
from matplotlib.animation import FuncAnimation
import math
import winsound

ROAD_EDGE_COLOR = 'black'
ROAD_COLOR = 'lightgrey'
NON_ROAD_COLOR = 'white'
TRAIL_COLOR = 'grey'


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
        'error': [],
        'text': [],
        'trail_x': [],
        'trail_y': [],
        'lookahead_point': [],
        'closest_point': [],
        'headed_vector': [],
        'track_vector': []
    }


def add_trail(vehicle, parameters):
    vehicle_x, vehicle_y = vehicle.front_mid_point.x, vehicle.front_mid_point.y
    parameters['trail_x'].append(vehicle_x)
    parameters['trail_y'].append(vehicle_y)


def add_text(vehicle, parameters):
    text = (
            f'per acc: {str(round(vehicle.perpendicular_acc.norm(), 2))}' + ' ms2\n'
            + f'parallel acc: {str(round(vehicle.parallel_acc.norm(), 2))}' + ' ms2'
            + '\nangle: '
            + str(round(math.degrees(vehicle.error), 2)) + ' degrees'
            + '\nspeed: '
            + str(f'{round(vehicle.velocity.norm(), 2)}') + ' ms'
            + '\nerror: '
            + str(f'{round(vehicle.error, 2)}')
    )
    parameters['text'].append(text)


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


def add_error(vehicle, parameters):
    parameters['error'].append(vehicle.error)


def add_track_vector(vehicle, parameters):
    vehicle_point = (vehicle.front_mid_point.x, vehicle.front_mid_point.y)
    lookahead_point = (vehicle.lookahead_point.x, vehicle.lookahead_point.y)
    parameters['track_vector'].append((vehicle_point, lookahead_point))


def add_projected_points(vehicle, parameters):
    parameters['lookahead_point'].append(vehicle.lookahead_point)
    parameters['closest_point'].append(vehicle.closest_point)


def add_centroid(vehicle, parameters):
    parameters['centroid'].append(vehicle.centroid)


def update_parameters_list(vehicle, parameters):
    add_trail(vehicle, parameters)
    add_text(vehicle, parameters)
    add_positions(vehicle, parameters)
    add_velocity(vehicle, parameters)
    add_perpendicular_acc(vehicle, parameters)
    add_parallel_acc(vehicle, parameters)
    add_resultant_acc(vehicle, parameters)
    add_acc_magnitudes(vehicle, parameters)
    add_projected_points(vehicle, parameters)
    add_speed(vehicle, parameters)
    add_error(vehicle, parameters)
    add_track_vector(vehicle, parameters)
    add_centroid(vehicle, parameters)


def compute_parameters(vehicle, road_network, frames):
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
        'error': np.loadtxt("error.txt")
    }


def plot_parameters(vehicle, start, end, fig_name):
    params = load_params()
    fig, axs = plt.subplots(3, 2, figsize=(8, 10))
    axs = axs.flatten()
    frames = np.linspace(0, len(params['perpendicular_acc']), len(params['perpendicular_acc']))

    plot_parameter(axs[0], frames, params['perpendicular_acc'], "Perpendicular Acceleration", vehicle.acc_limit)
    plot_parameter(axs[1], frames, params['error'], "Error", math.radians(vehicle.angle_threshold))
    plot_parameter(axs[2], frames, params['parallel_acc'], "Parallel Acceleration", vehicle.acc_limit)
    plot_parameter(axs[3], frames, params['speed'], "Speed")
    plot_parameter(axs[4], frames, params['resultant_acc'], "Resultant Acceleration", vehicle.acc_limit)

    axs[5].axis('off')

    fig.suptitle(fig_name)
    fig.tight_layout()
    fig.savefig(f"{fig_name}.png")
    plt.show()


def get_artist_objects(ax):
    vehicle_line, = ax.plot([], [])
    trail_line, = ax.plot([], [], color=TRAIL_COLOR)
    acc_line, = ax.plot([], [])
    acc2_line, = ax.plot([], [])
    velocity_line, = ax.plot([], [])
    lookahead_point = ax.scatter([], [], color='red', s=5)
    closest_point = ax.scatter([], [], color='green', s=5)
    text = ax.text(0, 0, "", fontsize=6)
    track_line, = ax.plot([], [])
    return vehicle_line, acc_line, acc2_line, velocity_line, closest_point, text, track_line, trail_line, lookahead_point


def simulate(road_network, vehicle, frames, parameters, file_name):  # reference track
    fig = plt.figure()
    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box')
    # road_network.show_road_network(ax)
    x, y = vehicle.reference_track.xy
    plt.plot(x, y)

    vehicle_line, perpendicular_acc_line, parallel_acc_line, velocity_line, closest_point, text, track_line, trail_line, \
        lookahead_point = get_artist_objects(ax)

    def init():
        return vehicle_line, perpendicular_acc_line, parallel_acc_line, velocity_line, closest_point, text, track_line

    def animate(i):
        vehicle_line.set_data(parameters['vehicle'][i])
        velocity_line.set_data(parameters['velocity'][i])
        trail_line.set_data(parameters['trail_x'][:i], parameters['trail_y'][:i])
        perpendicular_acc_line.set_data(parameters['perpendicular_acc'][i])
        parallel_acc_line.set_data(parameters['parallel_acc'][i])
        lookahead_point.set_offsets((parameters['lookahead_point'][i].x, parameters['lookahead_point'][i].y))
        closest_point.set_offsets((parameters["closest_point"][i].x, parameters["closest_point"][i].y))

        start, end = parameters['track_vector'][i]
        x = [start[0], end[0]]
        y = [start[1], end[1]]
        track_line.set_data(x, y)

        window_size = 13
        text.set_position((parameters['centroid'][i].get_x() - 1.75 * window_size, parameters['centroid'][i].get_y()))
        x = min(parameters['centroid'][j].get_x() for j in range(len(parameters['centroid']))) - 15
        y = max(parameters['centroid'][j].get_y() for j in range(len(parameters['centroid']))) / 2
        text.set_position((x, y))
        text.set_text(parameters['text'][i])

        # ax.set_xlim(parameters['centroid'][i].get_x() - window_size, parameters['centroid'][i].get_x() + window_size)
        # ax.set_ylim(parameters['centroid'][i].get_y() - window_size, parameters['centroid'][i].get_y() + window_size)

        return vehicle_line, velocity_line, perpendicular_acc_line, parallel_acc_line, text, trail_line, track_line

    print("Animation in progress...")
    start = time.time()
    anim = FuncAnimation(fig, animate, init_func=init, frames=frames, blit=True)
    anim.save(file_name, writer='ffmpeg', fps=60)
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
    write_parameter_to_file(arrays, "error")


def create_simulation_video(vehicle, road_network, frames):
    parameters = compute_parameters(vehicle, road_network, frames)
    write_parameters_to_file(parameters)

    file_name = f"p{car.params['P_PER_ACC_WEIGHT']} " \
                f"d {car.params['D_PER_ACC_WEIGHT']} " \
                f"acc_lim {vehicle.acc_limit} " \
                f"init_speed {round(vehicle.initial_speed, 2)} " \
                f"dec {car.params['P_PARALLEL_DEC_WEIGHT']} threshold {car.params['angle_threshold']}.mp4"

    simulate(road_network, vehicle, frames=frames, parameters=parameters, file_name=file_name)
    winsound.Beep(frequency=2500, duration=1000)
    plot_parameters(vehicle, start=5, end=frames, fig_name=file_name)
