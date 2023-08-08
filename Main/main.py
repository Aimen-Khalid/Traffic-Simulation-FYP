import os
import sys
import matplotlib.pyplot as plt
from shapely import Point, LineString, Polygon, box
import networkx as nx
import random
import math
from tqdm import tqdm
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, os.pardir))
sys.path.append(parent_dir)

from MapToRoadMapping import dcel, osm
from CarModel import car
from Utility.point import ScaledPoint
from Utility import files_functions
from SimulationEngine import simulation_with_vectors, simulation
import MapToRoadMapping.osm
from MapToRoadMapping.graph_to_road_network import show_graph_lanes, translate_segment
from RoadBuildingTool.drawing_tool import draw_and_save_road_network_graph, get_vertices_and_segments, get_points_on_graph
from ObstacleAvoidance import obstacle_avoidance

"""
Head over to this link to understand the terms used in this project:
https://drive.google.com/file/d/1s4K-4sUjk51QQ3viBoLcKz9DzYK-8FNi/view?usp=sharing
"""

def show_graph(graph):
    node_positions = nx.get_node_attributes(graph, 'pos')
    node_ids = nx.get_node_attributes(graph, 'id')
    # for node, (x, y) in node_positions.items():
    #     plt.text(x, y, str(node_ids[node]), fontsize=12, ha='center', va='center')
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_aspect('equal')  # Set equal aspect ratio
    graph1 = nx.to_undirected(graph)
    labels = nx.get_node_attributes(graph, 'id')
    nx.draw(graph1, pos=node_positions, node_size=40, font_size=8, labels=labels, with_labels=True,)
    plt.show()


def initialize_velocity(reference_track, magnitude):
    start, end = reference_track.coords[0], reference_track.coords[1]
    end = ScaledPoint(end[0] - start[0], end[1] - start[1])
    end = end / end.norm()
    end = end * magnitude
    return end

def get_random_color():
    # Generate random values for red, green, and blue channels
    red = random.randint(0, 255)
    green = random.randint(0, 255)
    blue = random.randint(0, 255)

    # Convert the values to hexadecimal and format the color code
    color_code = "#{:02x}{:02x}{:02x}".format(red, green, blue)

    return color_code


def show_road_network(coordinates, area_name):
    graph = files_functions.extract_road_network_graph(coordinates, area_name)
    dcel_obj = dcel.Dcel(graph)
    dcel_obj.build_dcel(graph)
    dcel_obj.show_road_network()


def show_saved_road_network(area_name):
    vertices_fn = f"{area_name}_vertices.txt"
    segments_fn = f"{area_name}_segments.txt"
    graph = files_functions.create_graph_from_files(vertices_fn, segments_fn)
    show_graph(graph)
    # show_graph_lanes(graph)
    dcel_obj = dcel.Dcel(graph)
    dcel_obj.build_dcel(graph)

    dcel_obj.show_road_network()



def get_road_network(area_name, new):
    print("Road network creating...")
    vertices_fn = f"{area_name}_vertices.txt"
    segments_fn = f"{area_name}_segments.txt"
    if new:
        draw_and_save_road_network_graph(vertices_fn, segments_fn)
        print("Draw a road network for your car...")
    graph = files_functions.create_graph_from_files(vertices_fn, segments_fn)
    # show_graph(graph)
    dcel_obj = dcel.Dcel(graph)
    dcel_obj.build_dcel(graph)
    print("Road network created")
    return dcel_obj


def create_custom_road_network(fn):
    vertices_fn = f"{fn}_vertices.txt"
    segments_fn = f"{fn}_segments.txt"
    draw_and_save_road_network_graph(vertices_fn, segments_fn)
    graph = files_functions.create_graph_from_files(vertices_fn, segments_fn)
    dcel_obj = dcel.Dcel(graph)
    dcel_obj.build_dcel(graph)
    dcel_obj.show_road_network()
    return dcel_obj


def create_track(file_name, new):
    if new:
        print("Draw a track for your car to follow...")
        track, _ = get_vertices_and_segments()
        track.append(track[0])
        files_functions.write_vertices_to_file(track, file_name)
    try:
        track = files_functions.load_vertices_from_file(file_name)
    except Exception as e:
        print(e)
        return
    return LineString(track)


def road_network_main(fn, new_custom=False, new_from_map=False):
    if new_from_map:
        coordinates = osm.get_coordinates_from_map()
        area_name = fn#input("Name the area that lies within the selected coordinates: ")
        show_road_network(coordinates, area_name)

    elif new_custom:
        create_custom_road_network(fn)

    else:
        show_saved_road_network(fn)


def simulate_on_track(frames, track_name, if_new_track):
    reference_track = create_track(track_name, new=if_new_track)

    vehicle = car.Vehicle(length=4, width=2,
                          centroid=ScaledPoint(reference_track.coords[0][0], reference_track.coords[0][1]),
                          angle=90, velocity=initialize_velocity(reference_track, 6), acceleration=ScaledPoint(0, 0),
                          reference_track=reference_track)
    # simulation.create_simulation_video(vehicle, nx.Graph(), frames, with_road_network=False)
    simulation.plot_vehicle_tracking(vehicle, frames)


def is_close_to_existing(point_list, given_point, distance_threshold):
    for point in point_list:
        distance = point.distance(given_point)
        if distance < distance_threshold:
            return True
    return False


def get_random_track(road_network, obstacles):
    no_of_nodes = road_network.graph.number_of_nodes() - 1
    start_node_id = random.randint(0, no_of_nodes)#8
    end_node_id = random.randint(0, no_of_nodes)#0
    while start_node_id == end_node_id:
        end_node_id = random.randint(0, no_of_nodes)
    # print(start_node_id, end_node_id)
    track = road_network.get_track(start_node_id, end_node_id, obstacles)
    return track


def cut_last_segment_at_random(line_string):
    if not line_string.is_empty and line_string.is_ring:
        raise ValueError("Input LineString must not be empty and must not be a ring.")
    last_segment = line_string.coords[-2:]
    random_fraction = random.uniform(0.4, 0.7)  # Adjust the range as needed
    x_cut = last_segment[0][0] + random_fraction * (last_segment[1][0] - last_segment[0][0])
    y_cut = last_segment[0][1] + random_fraction * (last_segment[1][1] - last_segment[0][1])
    new_line_coords = line_string.coords[:-1] + [(x_cut, y_cut)]
    return LineString(new_line_coords)

def get_vehicles_list(no_of_vehicles, road_network, obstacles):
    print(f'{no_of_vehicles} vehicles being initialized...')
    vehicles_list = []
    initial_positions = []
    final_positions = []
    obstacle_centroids = [obstacle.centroid for obstacle in obstacles]
    with tqdm(total=no_of_vehicles) as pbar:
        for _ in range(no_of_vehicles):
            #  honda civic
            if _ % 2 == 0:
                length = 4.6
                width = 1.8
            else:
                #  suzuki alto
                length = 3.5
                width = 1.5

            init_speed = random.uniform(4, 8)

            reference_track = get_random_track(road_network, obstacles)
            segment = LineString([reference_track.coords[0], reference_track.coords[1]])
            interpolation_distance = random.uniform(0.1, 0.7) #* segment_length

            initial_position = segment.interpolate(interpolation_distance, normalized=True)
            if len(reference_track.coords) > 2:
                reference_track = cut_last_segment_at_random(reference_track)
            final_position = Point(reference_track.coords[-1])
            iterations = 0
            while is_close_to_existing(initial_positions, initial_position, 6) or is_close_to_existing(obstacle_centroids, initial_position, 5)\
                or is_close_to_existing(final_positions, final_position, 6):
                # print('in while loop')
                reference_track = get_random_track(road_network, obstacles)
                # if len(reference_track.coords) < 3:
                #     continue
                # if Point(reference_track.coords[0]).distance(Point(reference_track.coords[-1])) < 20:
                #     continue
                # rand_node_index = random.randint(0, len(reference_track.coords) - 2)
                # segment = LineString([reference_track.coords[rand_node_index], reference_track.coords[rand_node_index + 1]])
                # initial_position = segment.interpolate(random.random(), normalized=True)
                segment = LineString([reference_track.coords[0], reference_track.coords[1]])
                segment_length = Point(reference_track.coords[0]).distance(Point(reference_track.coords[1]))
                interpolation_distance = random.uniform(0.1, 0.7) #* segment_length
                initial_position = segment.interpolate(interpolation_distance)

                if len(reference_track.coords) > 2:
                    reference_track = cut_last_segment_at_random(reference_track)
                final_position = Point(reference_track.coords[-1])
                iterations += 1
            initial_positions.append(initial_position)
            final_positions.append(final_position)

            vehicle = car.Vehicle(length, width,
                                  centroid=ScaledPoint(initial_position.x, initial_position.y),
                                  angle=90, velocity=initialize_velocity(segment, init_speed),
                                  acceleration=ScaledPoint(0, 0),
                                  reference_track=reference_track
                                  )

            vehicles_list.append(vehicle)
            pbar.update(1)
    print('Vehicles initialized.')
    return vehicles_list


def simulate_on_road_network(frames, road_network_name, if_new_network, if_new_obstacles):
    road_network = get_road_network(road_network_name, new=if_new_network)
    try:
        if if_new_obstacles:
            obstacle_positions = get_points_on_graph(road_network, road_network_name)
            files_functions.write_vertices_to_file(obstacle_positions, f'{road_network_name}_obstacles_positions')
        else:
            obstacle_positions = files_functions.load_vertices_from_file(f'{road_network_name}_obstacles_positions')

        obstacles = []
        i = 0
        while i <= len(obstacle_positions)-2:
            segment = [obstacle_positions[i], obstacle_positions[i+1]]
            translated_segment_cw = translate_segment(segment, 1)
            translated_segment_acw = translate_segment(segment, 1, anticlockwise=True)
            obstacles.append(Polygon([translated_segment_cw[0], translated_segment_cw[1], translated_segment_acw[1],
                                        translated_segment_acw[0]]))
            i += 2
    except Exception:
        print('Obstacles not initialized. Creating simulation without obstacles.')
        obstacles = []

    obstacle_avoidance.obstacles = obstacles
    # obstacles = [
    #     Point(position).buffer(obstacle_radius)
    #     for position in obstacles_positions
    # ]
    # obstacles = []
    vehicles = get_vehicles_list(50, road_network, obstacles)
    simulation.create_simulation_video(vehicles, road_network, obstacles, frames, with_road_network=True)


def simulation_main():
    frames = 5000
    # simulate_on_track(frames, "hexagon", if_new_track=False)
    simulate_on_road_network(frames, "dha", if_new_network=False, if_new_obstacles=False)

    plt.close('all')

# road_network_main("full_network", new_from_map=False, new_custom=False)#MT


simulation_main()

