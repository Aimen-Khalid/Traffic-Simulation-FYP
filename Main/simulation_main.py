from shapely import Point, LineString
from tqdm import tqdm
import random

from RUSTIC.RoadBuildingTool.drawing_tool import draw_custom_road_network
from RUSTIC.SimulationEngine import simulation_with_vectors, simulation
from RUSTIC.ObstacleAvoidance import obstacle_avoidance
from RUSTIC.Main.road_network import get_road_network
from RUSTIC.Utility.point import ScaledPoint
from RUSTIC.Utility import files_functions
from RUSTIC.CarModel.car import Vehicle


def get_velocity_for_vector(reference_track, magnitude):
    """
    :return: velocity vector in direction of track's first segment and given magnitude
    """
    track_start, track_end = reference_track.coords[0], reference_track.coords[1]
    track_vector = ScaledPoint(track_end[0] - track_start[0], track_end[1] - track_start[1])
    normalized_track_vector = track_vector / track_vector.norm()
    return normalized_track_vector * magnitude


def create_track(file_name, new):
    """
    :param file_name: name of file that has points that make up reference track
    :param new: if true, prompts user to create new track and saves points in file_name.
    if false, fetches saved points from file_name to create a track (a line string)
    """
    if new:
        print("Draw a track for your car to follow...")
        track, _ = draw_custom_road_network()
        track.append(track[0])
        files_functions.write_vertices_to_file(track, file_name)
    try:
        track = files_functions.load_vertices_from_file(file_name)
    except Exception as e:
        print(e)
        return
    return LineString(track)


def simulate_on_track(frames, track_name, if_new_track):
    """
    :param frames: number of frames of simulation
    :param track_name: name of track on which you have to simulate the car
    :param if_new_track: prompts user to make a new track if true, otherwise simulates\
    car on track_name track if it exists
    """
    reference_track = create_track(track_name, new=if_new_track)

    vehicle = Vehicle(length=4, width=2,
                      centroid=ScaledPoint(reference_track.coords[0][0], reference_track.coords[0][1]),
                      angle=90, velocity=get_velocity_for_vector(reference_track, 6), acceleration=ScaledPoint(0, 0),
                      reference_track=reference_track)
    # simulation.create_simulation_video(vehicle, nx.Graph(), frames, with_road_network=False)
    simulation.plot_vehicle_tracking(vehicle, frames)


def is_close_to_existing(existing_points, new_point, distance_threshold):
    """
    :param existing_points: a list of shapely points
    :param new_point: a shapely point object
    :return: a boolean indicating if new_point lies within distance_threshold of any of the existing_points
    """
    for point in existing_points:
        distance = point.distance(new_point)
        if distance < distance_threshold:
            return True
    return False


def get_random_track(road_network):
    """
    :param road_network: a dcel object containing road network
    :return: a track that has a random node of road network graph as starting point and another random node as ending
    point
    """
    no_of_nodes = road_network.graph.number_of_nodes() - 1
    start_node_id = random.randint(0, no_of_nodes)
    end_node_id = random.randint(0, no_of_nodes)
    while start_node_id == end_node_id:
        end_node_id = random.randint(0, no_of_nodes)
    return road_network.get_track(start_node_id, end_node_id)


def cut_last_segment_at_random(line_string):
    """
    :returns: cuts the last segment of the given line string at a random point and returns the modified line string
    """
    if not line_string.is_empty and line_string.is_ring:
        raise ValueError("Input LineString must not be empty and must not be a ring.")
    last_segment = line_string.coords[-2:]
    random_fraction = random.uniform(0.4, 0.7)  # Adjust the range as needed
    x_cut = last_segment[0][0] + random_fraction * (last_segment[1][0] - last_segment[0][0])
    y_cut = last_segment[0][1] + random_fraction * (last_segment[1][1] - last_segment[0][1])
    new_line_coords = line_string.coords[:-1] + [(x_cut, y_cut)]
    return LineString(new_line_coords)


def init_car_params(road_network):
    """
    :returns: returns some randomly initialized car parameters for a single car based on given road network
    """
    reference_track = get_random_track(road_network)
    segment = LineString([reference_track.coords[0], reference_track.coords[1]])
    interpolation_distance = random.uniform(0.1, 0.7)
    initial_position = segment.interpolate(interpolation_distance, normalized=True)
    if len(reference_track.coords) > 2:
        reference_track = cut_last_segment_at_random(reference_track)
    final_position = Point(reference_track.coords[-1])
    init_speed = random.uniform(4, 8)
    init_velocity = get_velocity_for_vector(segment, init_speed)
    return reference_track, initial_position, final_position, init_velocity


def get_vehicles_list(no_of_vehicles, road_network):
    """
    :returns: a list having no_of_vehicles instances of vehicle objects that have randomly initialized parameters based
    on road_network
    """
    print(f'{no_of_vehicles} vehicles being initialized...')
    vehicles_list = []
    initial_positions = []
    final_positions = []
    with tqdm(total=no_of_vehicles) as pbar:
        safe_distance = 6
        for vehicle_count in range(no_of_vehicles):
            if vehicle_count % 2 == 0:  # honda civic
                length = 4.6
                width = 1.8
            else:
                length = 3.5  # suzuki alto
                width = 1.5

            reference_track, initial_position, final_position, init_velocity = init_car_params(road_network)
            iterations = 0
            while is_close_to_existing(initial_positions, initial_position, safe_distance) \
                    or is_close_to_existing(final_positions, final_position, safe_distance):
                reference_track, initial_position, final_position, init_velocity = init_car_params(road_network)
                iterations += 1
                if iterations >= 200:
                    raise ValueError(
                        'No more space for more cars. Retry by reducing safe_distance.'
                    )
            initial_positions.append(initial_position)
            final_positions.append(final_position)

            vehicle = Vehicle(length, width,
                              centroid=ScaledPoint(initial_position.x, initial_position.y),
                              angle=90, velocity=init_velocity,
                              acceleration=ScaledPoint(0, 0),
                              reference_track=reference_track
                              )

            vehicles_list.append(vehicle)
            pbar.update(1)
    print('Vehicles initialized.')
    return vehicles_list


def main():
    frames = 2000
    area_name = 'dha'
    road_network = get_road_network(area_name)
    obstacle_avoidance.add_obstacles_on_road_network(area_name, new=False)
    vehicles = get_vehicles_list(1, road_network)
    simulation.create_simulation_video(vehicles, road_network, frames, with_road_network=True)


main()
