import matplotlib.pyplot as plt
import networkx as nx

from RUSTIC.settings import road_width, scaling_factor
from RUSTIC.MapToRoadMapping import dcel, osm
from RUSTIC.Utility.files_functions import \
    (load_vertices_from_file, load_segments_from_file, write_vertices_to_file, write_segments_to_file)
from RUSTIC.RoadBuildingTool.drawing_tool import draw_custom_road_network


def get_road_network_graph(area_name):
    """
    :returns: underlying graph of the saved road network represented by area_name
    """
    graph = nx.DiGraph()
    vertices = list(set(load_vertices_from_file(area_name)))
    segments = load_segments_from_file(area_name)
    # Add nodes to the graph with their specific positions
    for i, vertex in enumerate(vertices):
        graph.add_node(vertex, pos=vertex, id=i)
    for segment in segments:
        graph.add_edge(segment[0], segment[1], visited=False, cw_start=None, cw_end=None, acw_start=None, acw_end=None,
                       lane_width=road_width)
        graph.add_edge(segment[1], segment[0], visited=False, cw_start=None, cw_end=None, acw_start=None, acw_end=None,
                       lane_width=road_width)
    return graph


def show_graph(graph, with_labels=False, label_with_coordinates=True):
    """
    :param graph: networkx directed graph object to display
    :param with_labels: if true, displays node IDs as labels with nodes
    :param label_with_coordinates: displays coordinates as node labels if true, otherwise displays node IDs as labels
    """
    node_positions = nx.get_node_attributes(graph, 'pos')
    node_ids = nx.get_node_attributes(graph, 'id')
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_aspect('equal')
    undirected_graph = nx.to_undirected(graph)  # Converting to undirected graph to not show arrows when displayed
    labels = nx.get_node_attributes(graph, 'id') if label_with_coordinates is False else node_positions
    nx.draw(undirected_graph, pos=node_positions, node_size=40, font_size=8, with_labels=with_labels, labels=labels)
    plt.show()


def get_road_network(area_name, new_custom=False, new_from_map=False):
    """
    Returns a dcel containing a road network of roads in area_name
    :param area_name: name of the area whose road network is to be fetched
    :param new_custom: if true, prompts user to create new road network's graph and saves it in area_name_vertices.txt,
    area_name_segments.txt, area_name_obstacles_positions.txt
    :param new_from_map: if true, prompts user to select a geojson file from file explorer, extracts the coordinates of
    bounding box of map from it, creates a road network of the area inside the bounding box and saves it in
    area_name_vertices.txt, area_name_segments.txt, area_name_obstacles_positions.txt.
    If both bool params are false, the function expects that a saved road network exists in files with prefix area_name
    and fetches the network from these files and returns it.
    """
    if new_from_map:
        print('Run the map.html file. Select the region from map and click export.\n'
              'A geojson file will be saved. When this function is run, select the geojson file from the\n'
              'explorer that appeared. This will extract roads from that region and display the corresponding DCEL.')
        coordinates = osm.select_map_region()
        vertices, segments = osm.extract_roads_from_osm(coordinates)
        write_vertices_to_file(vertices, area_name)
        write_segments_to_file(segments, area_name)
    elif new_custom:
        print('Draw the underlying graph of the road network on the tool.\n'
              'Click on a vertex twice to complete the current component. Click thrice to quit.')
        vertices, segments = draw_custom_road_network()
        write_vertices_to_file(vertices, area_name)
        write_segments_to_file(segments, area_name)

    graph = get_road_network_graph(area_name)
    road_network = dcel.Dcel(graph)
    road_network.build_dcel(graph)
    return road_network


def main():
    road_network = get_road_network("dha", new_from_map=False, new_custom=False)
    road_network.show_road_network()


main()
