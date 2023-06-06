import math
import dcel
import car
from point import ScaledPoint
import matplotlib.pyplot as plt
import numpy as np
import time
import winsound
from shapely import Point, LineString
import networkx as nx
import geometry_functions
import files_functions
import simulation
from graph_to_road_network import show_graph_lanes
from drawing_tool import draw_and_save_road_network_graph, get_vertices_and_segments
from geometry_functions import get_interpolated_curve

"""
Head over to this link to understand the terms used in this project:
https://drive.google.com/file/d/1s4K-4sUjk51QQ3viBoLcKz9DzYK-8FNi/view?usp=sharing
"""


def show_graph(graph):
    node_positions = nx.get_node_attributes(graph, 'pos')
    # Add node labels with coordinates
    for node, (x, y) in node_positions.items():
        plt.text(x, y, f'({x}, {y})', fontsize=8, ha='center', va='center')
    nx.draw(graph, pos=node_positions, node_size=5, font_size=8, with_labels=False)
    plt.show()


def show_road_network(coordinates, area_name):
    graph = files_functions.extract_road_network_graph(coordinates, area_name)
    dcel_obj = dcel.Dcel(graph)
    dcel_obj.build_dcel(graph)
    dcel_obj.show_road_network()


def show_saved_road_network(area_name):
    vertices_fn = f"{area_name}_vertices.txt"
    segments_fn = f"{area_name}_segments.txt"
    graph = files_functions.create_graph_from_files(vertices_fn, segments_fn)
    dcel_obj = dcel.Dcel(graph)
    dcel_obj.build_dcel(graph)
    dcel_obj.show_road_network()


def get_saved_road_network(area_name):
    vertices_fn = f"{area_name}_vertices.txt"
    segments_fn = f"{area_name}_segments.txt"
    graph = files_functions.create_graph_from_files(vertices_fn, segments_fn)
    dcel_obj = dcel.Dcel(graph)
    dcel_obj.build_dcel(graph)
    return dcel_obj


def create_own_road_network():
    vertices_fn = "test_vertices.txt"
    segments_fn = "test_segments.txt"
    draw_and_save_road_network_graph(vertices_fn, segments_fn)
    graph = files_functions.create_graph_from_files(vertices_fn, segments_fn)
    dcel_obj = dcel.Dcel(graph)
    dcel_obj.build_dcel(graph)
    dcel_obj.show_road_network()


def create_track():
    track, _ = get_vertices_and_segments()
    file_name = "track.txt"
    files_functions.write_vertices_to_file(track, file_name)
    track = files_functions.load_vertices_from_file(file_name)
    track.append(track[0])
    return LineString(track)


def road_network_main():
    coordinates = [
        [74.18257, 31.378014],
        [74.191622, 31.378207],
        [74.19174, 31.375046],
        [74.182527, 31.374844],
        [74.18257, 31.378014]
    ]
    area_name = "Bahria"
    # show_road_network(coordinates, area_name)
    show_saved_road_network("Bahria")


def simulation_main():
    frames = 3000
    track = create_track()
    vehicle = car.Vehicle(length=6, width=3, centroid=ScaledPoint(track.coords[0][0], track.coords[0][1]),
                          angle=90, velocity=ScaledPoint(6, 0), acceleration=ScaledPoint(0, 0), reference_track=track)
    road_network = get_saved_road_network("Bahria")
    simulation.create_simulation_video(vehicle, road_network, frames)
    winsound.Beep(frequency=2500, duration=1000)


road_network_main()
simulation_main()

