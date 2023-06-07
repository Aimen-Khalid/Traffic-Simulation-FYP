import car
import time
from tqdm import tqdm
import matplotlib.pyplot as plt
import numpy as np
import geometry_functions
from matplotlib.animation import FuncAnimation
import math

ROAD_EDGE_COLOR = 'black'
ROAD_COLOR = 'lightgrey'
NON_ROAD_COLOR = 'white'
TRAIL_COLOR = 'grey'


def initialize_parameters_dict():
    # utility function for compute_parameters function
    return {
        'vehicle': [],
        'centroid': [],
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


def update_parameters_list(vehicle, parameters, vehicle_x, vehicle_y, intersection_points_list):
    """
    utility function for compute_parameters function
    """
    text = (f'acc: {str(vehicle.perpendicular_acc.norm())}'
            + '\ntheta: ' + str(vehicle.theta)
            + '\nerror: ' + str(vehicle.error)
            + '\nspeed: ' + str(vehicle.velocity.norm())
            )
    parameters['text'].append(text)

    x, y = vehicle.get_xy_lists()
    parameters['vehicle'].append((x, y))

    x = [vehicle_x, vehicle.velocity.get_x() + vehicle_x]
    y = [vehicle_y, vehicle.velocity.get_y() + vehicle_y]
    parameters['velocity'].append((x, y))

    parameters['trail_x'].append(vehicle_x)
    parameters['trail_y'].append(vehicle_y)
    x = [vehicle_x, vehicle.perpendicular_acc.get_x() + vehicle_x]
    y = [vehicle_y, vehicle.perpendicular_acc.get_y() + vehicle_y]
    parameters['acc'].append((x, y))
    parameters['acc_magnitude'].append(vehicle.acc_magnitude)
    parameters['speed'].append(vehicle.velocity.norm())
    parameters['error'].append(vehicle.error)
    parameters['centroid'].append(vehicle.centroid)

    intersection_points_list.append((vehicle.face_mid_point[0], vehicle.face_mid_point[1]))
    parameters['intersection_points'].append(intersection_points_list)

    return parameters


def compute_parameters1(road_network, vehicle, frames):  # face mid-point approach
    """Compute various parameters of a vehicle as it moves along a road network.

    :param road_network: A DCEL object representing the environment in which the vehicle is moving.
    :param vehicle: A vehicle object representing the vehicle whose parameters we want to compute.
    :param frames: The number of frames for which to compute the vehicle parameters.

    :returns: A dictionary containing various vehicle parameters computed over the specified number of frames.
    """
    print("Computation in progress...")
    start = time.time()
    parameters = initialize_parameters_dict()
    # Loop over the specified number of frames
    with tqdm(total=frames) as pbar:
        for _ in range(frames):
            # Get the current position of the vehicle
            vehicle_x, vehicle_y = vehicle.get_car_mid_point()
            vehicle.set_current_face(road_network)
            # If the vehicle is not in a valid face, skip to the next iteration
            if vehicle.current_face is None:
                continue
            # Find the two closest intersection points of the vehicle's current face
            intersection_points_list = vehicle.get_closest_intersection_points()
            # Compute the midpoint of the two intersection points as the current face midpoint
            vehicle.face_mid_point = geometry_functions.get_mid_point(
                [(intersection_points_list[0][0], intersection_points_list[0][1]),
                 (intersection_points_list[1][0], intersection_points_list[1][1])])
            vehicle.prev_error = vehicle.error
            # Compute the vehicle's current error
            vehicle.error = vehicle.get_error_midpoint_approach(intersection_points_list)
            vehicle.update_state_vars()
            # Update the parameters dictionary with the computed values for this iteration
            parameters = update_parameters_list(parameters, vehicle_x, vehicle_y, intersection_points_list)
            # Update the progress bar
            pbar.update(1)

    end = time.time()
    print(f"{int(end - start)} Seconds")

    return parameters


def compute_parameters(vehicle, road_network, frames):  # reference track approach
    """
    Computes vehicle parameters for each frame of a simulation using the reference track approach.

    :param vehicle: A Vehicle object representing the vehicle being simulated.
    :param frames: The number of simulation frames to compute parameters for.
    :param road_network: The DCEL on which vehicle is simulated.
    :return: A dictionary containing vehicle parameters for each frame of the simulation.
    """
    print("Computation in progress...")
    start = time.time()

    # Initialize an empty dictionary to store the computed parameters for each frame of the simulation
    parameters = {
        'vehicle': [],  # The x and y coordinates of the vehicle
        'centroid': [],
        'velocity': [],  # The x and y components of the vehicle's velocity vector
        'speed': [],  # The magnitude of the vehicle's velocity vector
        'acc': [],  # The x and y components of the vehicle's acceleration vector
        'acc2': [],
        'acc_magnitude': [],  # The magnitude of the vehicle's acceleration vector
        'error': [],  # The vehicle's lateral error from the reference track
        'text': [],  # Additional text information to display on the simulation plot
        'trail_x': [],  # The x coordinates of the vehicle's trail (for visualization purposes)
        'trail_y': [],  # The y coordinates of the vehicle's trail (for visualization purposes)
        'projected_points': [],  # The projected point of the vehicle onto the reference track
        'headed_vector': [],
        'track_vector': []
    }

    # Use tqdm to display a progress bar while computing the parameters for each frame
    with tqdm(total=frames) as pbar:
        for _ in range(frames):
            # Get the current x and y coordinates of the vehicle's midpoint
            vehicle_x, vehicle_y = vehicle.get_vehicle_front_mid_point().x, vehicle.get_vehicle_front_mid_point().y

            # Add the current x and y coordinates to the trail list for visualization purposes
            parameters['trail_x'].append(vehicle_x)
            parameters['trail_y'].append(vehicle_y)

            # Compute the vehicle's lateral error from the reference track and update the vehicle's state variables
            vehicle.prev_error = vehicle.error
            # vehicle.error = vehicle.get_error_from_track()
            vehicle.error = vehicle.get_error_ang_disp()
            projected_points = vehicle.get_projected_points()
            vehicle.update_state_vars()

            # Add additional text information to the plot for this frame
            text = (
                    f'per acc: {str(round(vehicle.perpendicular_acc.norm(), 2))}' + ' ms2\n'
                    + f'parallel acc: {str(round(vehicle.parallel_acc.norm(), 2))}' + ' ms2'
                    + '\nframes: '
                    + str(_)
                    + '\nangle: '
                    + str(round(math.degrees(vehicle.error), 2)) + ' degrees'
                    + '\nspeed: '
                    + str(f'{round(vehicle.velocity.norm(), 2)}') + ' ms'
                    + '\nerror: '
                    + str(f'{round(vehicle.error, 2)}')
            )
            parameters['text'].append(text)

            # Add the vehicle's x and y coordinates to the dictionary
            x, y = vehicle.get_xy_lists()
            parameters['vehicle'].append((x, y))

            # Add the x and y components of the vehicle's velocity vector to the dictionary
            x = [vehicle_x, vehicle.velocity.get_x() + vehicle_x]
            y = [vehicle_y, vehicle.velocity.get_y() + vehicle_y]
            parameters['velocity'].append((x, y))

            # Add the x and y components of the vehicle's acceleration vector to the dictionary
            x = [vehicle_x, vehicle.perpendicular_acc.get_x() + vehicle_x]
            y = [vehicle_y, vehicle.perpendicular_acc.get_y() + vehicle_y]
            parameters['acc'].append((x, y))

            x = [vehicle_x, vehicle.parallel_acc.get_x() + vehicle_x]
            y = [vehicle_y, vehicle.parallel_acc.get_y() + vehicle_y]
            parameters['acc2'].append((x, y))

            # Add the magnitude of the vehicle's acceleration vector to the dictionary
            parameters['acc_magnitude'].append(vehicle.perpendicular_acc.norm())

            # Add the magnitude of the vehicle's velocity vector to the dictionary
            parameters['speed'].append(vehicle.velocity.norm())

            # Add the magnitude of the vehicle's error to the dictionary
            parameters['error'].append(vehicle.error)
            headed, track = vehicle.get_vectors()
            parameters['headed_vector'].append(headed)
            parameters['track_vector'].append(track)

            # Add the vehicle's projected point onto the reference track to the dictionary
            parameters['projected_points'].append(projected_points)

            parameters['centroid'].append(vehicle.centroid)

            pbar.update(1)

    end = time.time()
    print(f"{int(end - start)} Seconds")

    return parameters


def plot_parameters(vehicle, start, end):
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
    ax1.axhline(y=-1 * vehicle.acc_limit, color='blue')
    max_acc = np.max(acc[10:])
    min_acc = np.min(acc[10:])
    ax1.text(0.95, 0.95, f"max acc={max_acc:.2f}\nmin acc={min_acc:.2f}", transform=ax1.transAxes, ha='right', va='top',
             bbox=dict(facecolor='white', edgecolor='none', alpha=0.5))

    ax1.legend(loc='upper left')
    ax1.set_ylabel('Acceleration')
    # ax1.set_xlim([400, 450])

    ax2.plot(frames, error)
    ax2.set_ylabel('Error')
    ax2.set_ylim([min(error), max(error)])
    # ax2.set_xlim([400, 450])

    ax3.plot(frames, speed)
    ax3.set_ylabel('Speed')
    ax3.set_xlabel('Frames')

    fig.suptitle(f"kp={car.P_PER_ACC_WEIGHT} kd={car.D_PER_ACC_WEIGHT}")

    # Adjust the spacing between subplots
    fig.tight_layout()

    plt.show()

    fig.savefig(f"p{car.P_PER_ACC_WEIGHT} d {car.D_PER_ACC_WEIGHT} acc_lim {vehicle.acc_limit} "
                f"dist {vehicle.lookahead_distance} init_speed {round(vehicle.initial_speed, 2)} "
                f"dec {car.P_PARALLEL_DEC_WEIGHT} threshold {car.angle_threshold}.png")


def simulate1(road_network, vehicle, frames, parameters, file_name):  # face mid-point
    fig = plt.figure()
    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box')
    x, y = vehicle.reference_track.xy
    ax.plot(x, y)
    road_network.show_road_network(ax)

    # road_network.show_road_network(ax)

    vehicle_line, = ax.plot([], [])
    trail_line, = ax.plot([], [], color=TRAIL_COLOR)
    ideal_trail_line, = ax.plot([], [], color='green')

    ideal_trail_line.set_data(
        [parameters['intersection_points'][j][2][0] for j in range(5, len(parameters['intersection_points']))],
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

    # road_network.show_road_network(ax)

    x, y = vehicle.reference_track.xy
    plt.plot(x, y)

    vehicle_line, = ax.plot([], [])
    trail_line, = ax.plot([], [], color=TRAIL_COLOR)

    acc_line, = ax.plot([], [])
    acc2_line, = ax.plot([], [])
    velocity_line, = ax.plot([], [])
    lookahead_point = ax.scatter([], [], color='red', s=5)
    closest_point = ax.scatter([], [], color='green', s=5)
    text = ax.text(0, 0, "", fontsize=6)

    headed_line, = ax.plot([], [])
    track_line, = ax.plot([], [])

    def init():
        return vehicle_line, acc_line, acc2_line, velocity_line, closest_point, text, headed_line, track_line

    def animate(i):
        vehicle_line.set_data(parameters['vehicle'][i])
        velocity_line.set_data(parameters['velocity'][i])
        trail_line.set_data(parameters['trail_x'][:i], parameters['trail_y'][:i])

        acc_line.set_data(parameters['acc'][i])

        acc2_line.set_data(parameters['acc2'][i])

        lookahead_point.set_offsets((parameters['projected_points'][i]["lookahead_point"].x,
                                     parameters['projected_points'][i]["lookahead_point"].y))
        closest_point.set_offsets((parameters['projected_points'][i]["closest_point"].x,
                                   parameters['projected_points'][i]["closest_point"].y))

        start, end = parameters['headed_vector'][i]
        x = [start[0], end[0]]
        y = [start[1], end[1]]
        headed_line.set_data(x, y)

        start, end = parameters['track_vector'][i]
        x = [start[0], end[0]]
        y = [start[1], end[1]]
        track_line.set_data(x, y)

        window_size = 13
        # text.set_position((parameters['centroid'][i].get_x()-1.75*window_size, parameters['centroid'][i].get_y()))
        x = min(parameters['centroid'][j].get_x() for j in range(len(parameters['centroid']))) - 10
        y = max(parameters['centroid'][j].get_y() for j in range(len(parameters['centroid']))) / 2
        text.set_position((x, y))
        text.set_text(parameters['text'][i])

        # ax.set_xlim(parameters['centroid'][i].get_x() - window_size, parameters['centroid'][i].get_x() + window_size)
        # ax.set_ylim(parameters['centroid'][i].get_y() - window_size, parameters['centroid'][i].get_y() + window_size)



        return vehicle_line, velocity_line, acc_line, acc2_line, text, trail_line, headed_line, track_line

    print("Animation in progress...")
    start = time.time()
    anim = FuncAnimation(fig, animate, init_func=init, frames=frames, blit=True)
    anim.save(file_name, writer='ffmpeg', fps=60)
    end = time.time()
    print("Animation COMPLETED....")
    print(f"{int(end - start)} Seconds")


def write_parameters_to_file(arrays):
    with open("acc.txt", "w") as file:
        for a in arrays['acc_magnitude']:
            file.write(f"{a}\n")
    with open("speed.txt", "w") as file:
        for s in arrays['speed']:
            file.write(f"{s}\n")
    with open("error.txt", "w") as file:
        for e in arrays['error']:
            file.write(f"{e}\n")


def create_simulation_video(vehicle, road_network, frames):
    parameters = compute_parameters(vehicle, road_network, frames)
    write_parameters_to_file(parameters)

    # simulation will start after closing the plot figure
    plot_parameters(vehicle, start=5, end=frames)

    simulate(road_network, vehicle, frames=frames, parameters=parameters,
             file_name=f"p{car.P_PER_ACC_WEIGHT} d {car.D_PER_ACC_WEIGHT} acc_lim {vehicle.acc_limit} "
                       f"dist {vehicle.lookahead_distance} init_speed {round(vehicle.initial_speed, 2)} dec {car.P_PARALLEL_DEC_WEIGHT} threshold {car.angle_threshold}.mp4")
