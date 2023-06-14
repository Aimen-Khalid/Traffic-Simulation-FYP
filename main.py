import dcel
import car
from point import ScaledPoint
import matplotlib.pyplot as plt
from shapely import Point, LineString
import networkx as nx
import files_functions
import simulation
import osm
from graph_to_road_network import show_graph_lanes
from drawing_tool import draw_and_save_road_network_graph, get_vertices_and_segments

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


def initialize_velocity(reference_track, magnitude):
    start, end = reference_track.coords[0], reference_track.coords[1]
    end = ScaledPoint(end[0] - start[0], end[1] - start[1])
    end = end / end.norm()
    end = end * magnitude
    return end


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


def get_saved_road_network(area_name):
    vertices_fn = f"{area_name}_vertices.txt"
    segments_fn = f"{area_name}_segments.txt"
    graph = files_functions.create_graph_from_files(vertices_fn, segments_fn)
    # show_graph(graph)
    dcel_obj = dcel.Dcel(graph)
    dcel_obj.build_dcel(graph)
    return dcel_obj


def create_own_road_network(fn):
    vertices_fn = f"{fn}_vertices.txt"
    segments_fn = f"{fn}_segments.txt"
    draw_and_save_road_network_graph(vertices_fn, segments_fn)
    graph = files_functions.create_graph_from_files(vertices_fn, segments_fn)
    dcel_obj = dcel.Dcel(graph)
    dcel_obj.build_dcel(graph)
    dcel_obj.show_road_network()


def create_track(file_name, new):
    if new:
        track, _ = get_vertices_and_segments()
        track.append(track[0])
        files_functions.write_vertices_to_file(track, file_name)
    try:
        track = files_functions.load_vertices_from_file(file_name)
    except Exception as e:
        print(e)
    # return LineString([(2, -7), (35, 8), (55, 12), (75, 15), (95, 10), (115, 8), (135, 9), (150, 12), (170, 20)])
    # return LineString([(2, 7), (30, 8), (60, 20), (90, 7), (120, 6), (150, 3), (180, 2), (220, 50), (250, 20)])
    return LineString(track)


def road_network_main(fn, new):
    # coordinates = osm.get_coordinates_from_map()
    # area_name = input("Name the area that lies within the selected coordinates: ")
    # show_road_network(coordinates, area_name)

    if new:
        create_own_road_network(fn)

    show_saved_road_network(fn)


def simulation_main():
    frames = 3500
    track = create_track("triangle", new=False)
    road_network = get_saved_road_network("single_junction")
    st_face = road_network.faces[0]

    reference_track = track#LineString(st_face.lane_curves[0])

    vehicle = car.Vehicle(length=4, width=2,
                          centroid=ScaledPoint(reference_track.coords[0][0], reference_track.coords[0][1]),
                          angle=90, velocity=initialize_velocity(reference_track, 6), acceleration=ScaledPoint(0, 0),
                          reference_track=reference_track)
    vehicle.current_face = st_face
    vehicle.prev_face = vehicle.current_face
    # vehicle.set_reference_curve(road_network)
    simulation.create_simulation_video(vehicle, road_network, frames)


# road_network_main("single_junction", new=False)
simulation_main()
