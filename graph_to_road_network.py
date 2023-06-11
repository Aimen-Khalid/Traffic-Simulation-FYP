import math
from shapely import Point, LineString
import files_functions
import networkx as nx
# from rtree import index
import geometry
import matplotlib.pyplot as plt


def translate_segment(segment, length, anticlockwise=False):
    if anticlockwise:
        length = -1 * length
    start, end = segment

    start = Point(start[0], start[1])
    end = Point(end[0], end[1])
    # Compute the vector representing the segment
    dx = end.x - start.x
    dy = end.y - start.y

    vector_length = ((dx ** 2) + (dy ** 2)) ** 0.5
    # Compute the cosine and sine of the angle between the vector and the x-axis
    cos_theta = dx / vector_length
    sin_theta = dy / vector_length

    # Translate the starting and ending points of the segment
    new_start_x = start.x + length * sin_theta
    new_start_y = start.y - length * cos_theta
    new_end_x = end.x + length * sin_theta
    new_end_y = end.y - length * cos_theta

    # Return the translated segment as a list of two tuples
    return [(new_start_x, new_start_y), (new_end_x, new_end_y)]


def get_angle(edge):
    """Calculate the angle the edge makes with x-axis, in radians.
    :param edge: a tuple containing the coordinates of the edge's endpoints
    :return: the angle of the edge, in radians
    """
    origin, destination = edge[0], edge[1]
    dx = destination[0] - origin[0]
    dy = destination[1] - origin[1]
    length = math.sqrt(dx * dx + dy * dy)

    if length == 0:
        return 0
    if dy > 0:
        return math.acos(dx / length)
    else:
        return 2 * math.pi - math.acos(dx / length)


def get_edges_cw(graph, node):
    outgoing_edges = list(graph.edges(node))
    outgoing_edges.sort(key=lambda e: get_angle(e), reverse=True)
    return outgoing_edges


def set_edge_attribute(graph, edge, atr_name, atr_value):
    nx.set_edge_attributes(graph, {
        (edge[0], edge[1]): {atr_name: atr_value}})


def get_edge_attribute(graph, edge, atr_name):
    return graph.get_edge_data(edge[0], edge[1])[atr_name]


def set_edge2_translated_points(graph, outgoing_edge):
    if graph.get_edge_data(outgoing_edge[0], outgoing_edge[1])['cw_start'] is not None:
        return
    cw_start = translate_segment(outgoing_edge, geometry.lane_width)[0]
    acw_start = translate_segment(outgoing_edge, geometry.lane_width, anticlockwise=True)[0]
    if graph.get_edge_data(outgoing_edge[0], outgoing_edge[1])['cw_start'] is None:

        nx.set_edge_attributes(graph, {
            (outgoing_edge[0], outgoing_edge[1]): {'cw_start': cw_start, 'acw_start': acw_start}})
        nx.set_edge_attributes(graph, {
            (outgoing_edge[1], outgoing_edge[0]): {'acw_end': cw_start, 'cw_end': acw_start}})


def set_edges_translated_points(graph, edges, translated_segments_cw, translated_segments_acw):
    for i, edge in enumerate(edges):
        if i + 1 == len(edges):
            segment1 = translated_segments_acw[0]
        else:
            segment1 = translated_segments_acw[i + 1]
        segment2 = translated_segments_cw[i]
        segment3 = translated_segments_acw[i]
        if i == 0:
            segment4 = translated_segments_cw[len(edges) - 1]
        else:
            segment4 = translated_segments_cw[i - 1]
        cw_start = tuple(geometry.get_intersection_point(segment1, segment2))
        acw_start = tuple(geometry.get_intersection_point(segment3, segment4))
        if graph.get_edge_data(edge[0], edge[1])['cw_start'] is None:
            nx.set_edge_attributes(graph, {(edge[0], edge[1]): {'cw_start': cw_start, 'acw_start': acw_start}})
            nx.set_edge_attributes(graph, {(edge[1], edge[0]): {'acw_end': cw_start, 'cw_end': acw_start}})


def set_remaining_edges_translated_points(graph):
    for node in graph.nodes:
        edges = get_edges_cw(graph, node)
        for edge in edges:
            if graph.get_edge_data(edge[0], edge[1])['cw_end'][0] is not None:
                continue
            neighbor_edges = get_edges_cw(graph, edge[1])
            if len(neighbor_edges) == 1:
                cw_end = translate_segment(edge, geometry.lane_width)[1]
                acw_end = translate_segment(edge, geometry.lane_width, anticlockwise=True)[1]
                nx.set_edge_attributes(graph, {(edge[0], edge[1]): {'cw_end': cw_end, 'acw_end': acw_end}})
                nx.set_edge_attributes(graph, {(edge[1], edge[0]): {'acw_start': cw_end, 'cw_start': acw_end}})
                continue
            # index = neighbor_edges.index(edge)
            if edge in neighbor_edges:
                index = neighbor_edges.index(edge)
            else:
                index = neighbor_edges.index(((edge[1][0], edge[1][1]), (edge[0][0], edge[0][1])))
            prev = len(neighbor_edges) - 1 if index == 0 else index - 1
            cw_end = graph.get_edge_data(neighbor_edges[prev][0], neighbor_edges[prev][1])['cw_start']
            next_ = 0 if index == len(neighbor_edges) - 1 else index + 1
            acw_end = graph.get_edge_data(neighbor_edges[next_][0], neighbor_edges[next_][1])['acw_start']
            if graph.get_edge_data(edge[0], edge[1])['cw_end'] is None:
                nx.set_edge_attributes(graph, {(edge[0], edge[1]): {'cw_end': cw_end, 'acw_end': acw_end}})
                nx.set_edge_attributes(graph, {(edge[1], edge[0]): {'acw_start': cw_end, 'cw_start': acw_end}})


def populate_translated_lists(graph, translated_segments, translated_vertices, partitions):
    for node in graph.nodes:
        edges = get_edges_cw(graph, node)
        for edge in edges:
            if graph.get_edge_data(edge[0], edge[1])['visited']:
                continue
            nx.set_edge_attributes(graph, {(edge[0], edge[1]): {'visited': True}})
            nx.set_edge_attributes(graph, {(edge[1], edge[0]): {'visited': True}})
            translated_segments.extend(
                (
                    [
                        (graph.get_edge_data(edge[0], edge[1])['cw_start']),
                        (graph.get_edge_data(edge[0], edge[1])['cw_end']),
                    ],
                    [
                        (graph.get_edge_data(edge[0], edge[1])['acw_start']),
                        (graph.get_edge_data(edge[0], edge[1])['acw_end']),
                    ],
                )
            )
            translated_vertices.extend(
                (
                    (graph.get_edge_data(edge[0], edge[1])['cw_start']),
                    (graph.get_edge_data(edge[0], edge[1])['cw_end']),
                    (graph.get_edge_data(edge[0], edge[1])['acw_start']),
                    (graph.get_edge_data(edge[0], edge[1])['acw_end']),
                )
            )
            partitions.extend(
                (
                    [
                        (graph.get_edge_data(edge[0], edge[1])['cw_start']),
                        (graph.get_edge_data(edge[0], edge[1])['acw_start']),
                    ],
                    [
                        (graph.get_edge_data(edge[0], edge[1])['cw_end']),
                        (graph.get_edge_data(edge[0], edge[1])['acw_end']),
                    ],)
            )
    partitions = {frozenset(segment) for segment in partitions}
    partitions = [list(segment) for segment in partitions]


def graph_to_lanes(graph):
    translated_vertices = []
    translated_segments = []
    partitions = []

    for node in graph.nodes:
        if graph.degree(node) == 2:
            outgoing_edge = list(graph.edges(node))[0]
            set_edge2_translated_points(graph, outgoing_edge)
        else:
            translated_segments_cw = []
            translated_segments_acw = []

            edges = get_edges_cw(graph, node)
            for edge in edges:
                nx.set_edge_attributes(graph, {(edge[0], edge[1]): {'visited': False}})
                translated_segments_cw.append(translate_segment(edge, geometry.lane_width))
                translated_segments_acw.append(translate_segment(edge, geometry.lane_width, anticlockwise=True))

            set_edges_translated_points(graph, edges, translated_segments_cw, translated_segments_acw)

    set_remaining_edges_translated_points(graph)
    populate_translated_lists(graph, translated_segments, translated_vertices, partitions)
    return translated_vertices, translated_segments, partitions


def plot_segments(segments, color, ax):
    for segment in segments:
        start, end = segment
        x_values = [start[0], end[0]]
        y_values = [start[1], end[1]]
        # ax.text(start[0], start[1], f"{round(start[0])},   {str(round(start[1]))}", fontsize=6, color=color)
        # ax.text(end[0], end[1], f"{round(end[0])},   {str(round(end[1]))}", fontsize=6, color=color)
        ax.plot(x_values, y_values, color=color)


def show_graph_lanes(graph):
    base_edges = graph.to_undirected().edges
    translated_vertices, translated_segments, partition_edges = graph_to_lanes(graph)
    fig, ax = plt.subplots()

    plot_segments(translated_segments, "green", ax)
    plot_segments(base_edges, "black", ax)
    plot_segments(partition_edges, "grey", ax)

    ax.set_aspect('equal')
    plt.show()


def get_translated_vertices_segments(graph):
    vertices, segments, partitions = graph_to_lanes(graph)
    # remove duplicates from partitions list
    unique_partitions = {frozenset(segment) for segment in partitions}
    # Re-cast partitions to list
    unique_partitions = [list(segment) for segment in unique_partitions]
    segments.extend(unique_partitions)
    return vertices, segments
