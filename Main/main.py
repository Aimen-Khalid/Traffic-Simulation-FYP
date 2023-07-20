import os
import sys
import matplotlib.pyplot as plt
from shapely import Point, LineString, Polygon
import networkx as nx
import random
import math

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, os.pardir))
sys.path.append(parent_dir)

from MapToRoadMapping import dcel
from CarModel import car
from Utility.point import ScaledPoint
from Utility import files_functions
from SimulationEngine import simulation
import MapToRoadMapping.osm
from MapToRoadMapping.graph_to_road_network import show_graph_lanes
from RoadBuildingTool.drawing_tool import draw_and_save_road_network_graph, get_vertices_and_segments, get_points_on_graph


"""
Head over to this link to understand the terms used in this project:
https://drive.google.com/file/d/1s4K-4sUjk51QQ3viBoLcKz9DzYK-8FNi/view?usp=sharing
"""


def show_graph(graph):
    node_positions = nx.get_node_attributes(graph, 'pos')
    node_ids = nx.get_node_attributes(graph, 'id')
    for node, (x, y) in node_positions.items():
        plt.text(x, y, str(node_ids[node]), fontsize=12, ha='center', va='center')
    nx.draw(graph, pos=node_positions, node_size=5, font_size=8, with_labels=False)
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
    # show_graph(graph)
    dcel_obj = dcel.Dcel(graph)
    dcel_obj.build_dcel(graph)

    dcel_obj.show_road_network()



def get_road_network(area_name, new):
    vertices_fn = f"{area_name}_vertices.txt"
    segments_fn = f"{area_name}_segments.txt"
    if new:
        draw_and_save_road_network_graph(vertices_fn, segments_fn)
        print("Draw a road network for your car...")
    graph = files_functions.create_graph_from_files(vertices_fn, segments_fn)
    # show_graph(graph)
    dcel_obj = dcel.Dcel(graph)
    dcel_obj.build_dcel(graph)
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


def road_network_main(fn, new):
    # coordinates = osm.get_coordinates_from_map()
    # area_name = input("Name the area that lies within the selected coordinates: ")
    # show_road_network(coordinates, area_name)

    if new:
        create_custom_road_network(fn)

    show_saved_road_network(fn)


def simulate_on_track(frames, track_name, if_new_track):
    reference_track = create_track(track_name, new=if_new_track)

    vehicle = car.Vehicle(length=4, width=2,
                          centroid=ScaledPoint(reference_track.coords[0][0], reference_track.coords[0][1]),
                          angle=90, velocity=initialize_velocity(reference_track, 6), acceleration=ScaledPoint(0, 0),
                          reference_track=reference_track)
    # simulation.create_simulation_video(vehicle, nx.Graph(), frames, with_road_network=False)
    simulation.plot_vehicle_tracking(vehicle, frames)


def get_vehicles_list(no_of_vehicles, road_network, obstacles):
    no_of_nodes = road_network.graph.number_of_nodes() - 1
    vehicles_list = []
    for _ in range(no_of_vehicles):
        start_node_id = random.randint(0, no_of_nodes)
        end_node_id = random.randint(0, no_of_nodes)

        while start_node_id == end_node_id:
            end_node_id = random.randint(0, no_of_nodes)
        print(start_node_id, end_node_id)
        reference_track = road_network.get_track(start_node_id, end_node_id, obstacles)#start_node_id, end_node_id)
        if _ % 2 == 0:
            length = 4
            width = 2
        else:
            length = 6
            width = 3
        init_speed = random.uniform(4, 8)
        vehicle = car.Vehicle(length, width,
                              centroid=ScaledPoint(reference_track.coords[0][0], reference_track.coords[0][1]),
                              angle=90, velocity=initialize_velocity(reference_track, init_speed),
                              acceleration=ScaledPoint(0, 0),
                              reference_track=reference_track
                              )

        vehicles_list.append(vehicle)
    return vehicles_list


def simulate_on_road_network(frames, road_network_name, if_new_network, if_new_obstacles):
    road_network = get_road_network(road_network_name, new=if_new_network)
    if if_new_obstacles:
        obstacles_positions = get_points_on_graph(road_network, road_network_name)
        files_functions.write_vertices_to_file(obstacles_positions, f'{road_network_name}_obstacles_positions')
    else:
        obstacles_positions = files_functions.load_vertices_from_file(f'{road_network_name}_obstacles_positions')
    obstacle_radius = 2
    obstacles = [
        Point(position).buffer(obstacle_radius)
        for position in obstacles_positions
    ]

    vehicles = get_vehicles_list(20, road_network, obstacles)
    simulation.create_simulation_video(vehicles, road_network, obstacles, frames, with_road_network=True)


def simulation_main():
    frames = 3000
    # simulate_on_track(frames, "hexagon", if_new_track=False)
    simulate_on_road_network(frames, "full_network", if_new_network=False, if_new_obstacles=False)


# road_network_main("full_network", new=False)

simulation_main()

