import math
from shapely import Point, LineString
import files_functions
import networkx as nx
from rtree import index
import geometry_functions
import matplotlib.pyplot as plt


def angle_between_segments(seg1, seg2):
    # Compute the vectors for the two segments
    seg1_point_1 = seg1[0]
    seg1_point_2 = seg1[1]
    seg2_point1 = seg2[0]
    seg2_point2 = seg2[1]
    v1 = (seg1_point_2.x - seg1_point_1.x, seg1_point_2.y - seg1_point_1.y)
    v2 = (seg2_point2.x - seg2_point1.x, seg2_point2.y - seg2_point1.y)

    # Compute the angle between the two vectors
    dot_product = v1[0] * v2[0] + v1[1] * v2[1]
    mag_v1 = math.sqrt(v1[0] ** 2 + v1[1] ** 2)
    mag_v2 = math.sqrt(v2[0] ** 2 + v2[1] ** 2)
    cos_angle = dot_product / (mag_v1 * mag_v2)
    angle = math.acos(cos_angle)

    # Determine the sign of the angle based on the direction of the turn
    cross_product = v1[0] * v2[1] - v1[1] * v2[0]
    if cross_product > 0:
        angle = 2 * math.pi - angle

    # Convert the angle to degrees and return it
    return math.degrees(angle)


def rotate_line(start_point, end_point, theta, anticlockwise=False):
    # Convert the angle from degrees to radians
    if not anticlockwise:
        theta = -1 * theta
    theta_rad = math.radians(theta)

    # Translate the line so that the starting point is at the origin
    translated_end_point = (end_point.x - start_point.x, end_point.y - start_point.y)

    # Rotate the translated end point by the angle theta
    rotated_end_point = (translated_end_point[0] * math.cos(theta_rad) - translated_end_point[1] * math.sin(theta_rad),
                         translated_end_point[0] * math.sin(theta_rad) + translated_end_point[1] * math.cos(theta_rad))

    # Translate the rotated end point back to the original coordinates
    new_end_point = (rotated_end_point[0] + start_point.x, rotated_end_point[1] + start_point.y)

    return new_end_point


def translate_segment(segment, length, anticlockwise=False):
    if anticlockwise:
        length = -1 * length
    # Extract the starting and ending points of the segment
    start, end = segment

    start = Point(start[0], start[1])
    end = Point(end[0], end[1])
    # Compute the vector representing the segment
    dx = end.x - start.x
    dy = end.y - start.y

    # Compute the length of the vector
    vector_length = ((dx ** 2) + (dy ** 2)) ** 0.5
    if vector_length == 0:
        vector_length = 1
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


def create_graph(vertices_file_name, segment_file_name):
    graph = nx.DiGraph()
    # Add vertices to the graph
    vertices_ = files_functions.load_vertices_from_file(vertices_file_name)
    segments_ = files_functions.load_segments_from_file(segment_file_name)
    # Add nodes to the graph with their specific positions
    for vertex in vertices_:
        graph.add_node(vertex, pos=vertex)
    # Add edges to the graph
    for segment in segments_:
        graph.add_edge(segment[0], segment[1], visited=False, cw_start=None, cw_end=None, acw_start=None, acw_end=None)
        graph.add_edge(segment[1], segment[0], visited=False, cw_start=None, cw_end=None, acw_start=None, acw_end=None)

    return graph


def create_spatial_index(graph):
    p = index.Property()
    p.dimension = 2  # 2-dimensional index for (x, y) coordinates
    idx = index.Index(properties=p)

    for count, node in enumerate(graph.nodes()):
        x, y = node
        idx.insert(count, (x, y, x, y), obj=node)  # Insert each node with its bounding box

    return idx


def get_nodes_in_range(graph, idx, x_range, y_range):
    return [
        node.object
        for node in idx.intersection(
            (x_range[0], y_range[0], x_range[1], y_range[1]), objects=True
        )
    ]


def line_lies_left(reference_line, line):
    # # Extract the coordinates of the line strings
    # coords1 = list(reference_line.coords)
    # coords2 = list(line.coords)
    #
    # # Calculate the cross product of the vectors formed by the line segments
    # cross_product = (
    #     (coords2[1][0] - coords2[0][0]) * (coords1[1][1] - coords1[0][1])
    #     - (coords2[1][1] - coords2[0][1]) * (coords1[1][0] - coords1[0][0])
    # )
    #
    # # Check if the cross product is positive
    # if cross_product > 0:
    #     return True
    # else:
    #     return False
    ref_line_max_x = max(reference_line.coords[0][0], reference_line.coords[1][0])
    return line.coords[0][0] < ref_line_max_x and line.coords[1][0] < ref_line_max_x


def line_lies_up(reference_line, line):
    # # Calculate the vectors formed by the endpoints of the lines
    # vector1 = (reference_line.coords[1][0] - reference_line.coords[0][0], reference_line.coords[1][1] - reference_line.coords[0][1])
    # vector2 = (line.coords[1][0] - line.coords[0][0], line.coords[1][1] - line.coords[0][1])
    #
    # # Calculate the cross product of the vectors
    # cross_product = vector1[0] * vector2[1] - vector1[1] * vector2[0]
    #
    # # Check if the cross product is positive
    # if cross_product > 0:
    #     return True
    # else:
    #     return False
    ref_line_min_y = min(reference_line.coords[0][1], reference_line.coords[1][1])
    return line.coords[0][1] > ref_line_min_y and line.coords[1][1] > ref_line_min_y


def line_lies_right(reference_line, line):
    # # Calculate the vectors formed by the endpoints of the lines
    # vector1 = (reference_line.coords[1][0] - reference_line.coords[0][0], reference_line.coords[1][1] - reference_line.coords[0][1])
    # vector2 = (line.coords[1][0] - line.coords[0][0], line.coords[1][1] - line.coords[0][1])
    #
    # # Calculate the cross product of the vectors
    # cross_product = vector1[0] * vector2[1] - vector1[1] * vector2[0]
    #
    # # Check if the cross product is positive
    # if cross_product > 0:
    #     return True
    # else:
    #     return False
    ref_line_min_x = min(reference_line.coords[0][0], reference_line.coords[1][0])
    return line.coords[0][0] > ref_line_min_x and line.coords[1][0] > ref_line_min_x


def line_lies_down(reference_line, line):
    # # Calculate the vectors formed by the endpoints of the lines
    # vector1 = (reference_line.coords[1][0] - reference_line.coords[0][0], reference_line.coords[1][1] - reference_line.coords[0][1])
    # vector2 = (line.coords[1][0] - line.coords[0][0], line.coords[1][1] - line.coords[0][1])
    #
    # # Calculate the cross product of the vectors
    # cross_product = vector1[0] * vector2[1] - vector1[1] * vector2[0]
    #
    # # Check if the cross product is negative
    # if cross_product < 0:
    #     return True
    # else:
    #     return False
    ref_line_max_y = max(reference_line.coords[0][1], reference_line.coords[1][1])
    return line.coords[0][1] < ref_line_max_y and line.coords[1][1] < ref_line_max_y


def is_relative_position_same(edge, outgoing_edge, edge_cw, edge_acw, outgoing_edge_cw, outgoing_edge_acw):
    if line_lies_up(edge, outgoing_edge):
        if not line_lies_up(edge_cw, outgoing_edge_cw):
            return False
        if not line_lies_up(edge_cw, outgoing_edge_acw):
            return False
        if not line_lies_up(edge_acw, outgoing_edge_cw):
            return False
        if not line_lies_up(edge_acw, outgoing_edge_acw):
            return False

    if line_lies_down(edge, outgoing_edge):
        if not line_lies_down(edge_cw, outgoing_edge_cw):
            return False
        if not line_lies_down(edge_cw, outgoing_edge_acw):
            return False
        if not line_lies_down(edge_acw, outgoing_edge_cw):
            return False
        if not line_lies_down(edge_acw, outgoing_edge_acw):
            return False

    if line_lies_left(edge, outgoing_edge):
        if not line_lies_left(edge_cw, outgoing_edge_cw):
            return False
        if not line_lies_left(edge_cw, outgoing_edge_acw):
            return False
        if not line_lies_left(edge_acw, outgoing_edge_cw):
            return False
        if not line_lies_left(edge_acw, outgoing_edge_acw):
            return False

    if line_lies_right(edge, outgoing_edge):
        if not line_lies_right(edge_cw, outgoing_edge_cw):
            return False
        if not line_lies_right(edge_cw, outgoing_edge_acw):
            return False
        if not line_lies_right(edge_acw, outgoing_edge_cw):
            return False
        if not line_lies_right(edge_acw, outgoing_edge_acw):
            return False

    return True

def graph_to_lanes(graph):
    vertices = []
    segments = []
    partitions = []

    for node in graph.nodes:
        if graph.degree(node) == 2:
            outgoing_edge = list(graph.edges(node))[0]

            if not graph.get_edge_data(outgoing_edge[0], outgoing_edge[1])['cw_start']:
                cw_start = translate_segment(outgoing_edge, graph.get_edge_data(outgoing_edge[0], outgoing_edge[1])['lane_width'])[0]
                acw_start = translate_segment(outgoing_edge, graph.get_edge_data(outgoing_edge[0], outgoing_edge[1])['lane_width'], anticlockwise=True)[0]
                if graph.get_edge_data(outgoing_edge[0], outgoing_edge[1])['cw_start'] is None:
                    nx.set_edge_attributes(graph, {
                        (outgoing_edge[0], outgoing_edge[1]): {'cw_start': cw_start, 'acw_start': acw_start}})
                    nx.set_edge_attributes(graph, {
                        (outgoing_edge[1], outgoing_edge[0]): {'acw_end': cw_start, 'cw_end': acw_start}})
            continue
        translated_segments_cw = []
        translated_segments_acw = []

        edges = get_edges_cw(graph, node)
        for edge in edges:
            nx.set_edge_attributes(graph, {(edge[0], edge[1]): {'visited': False}})
            translated_segments_cw.append(translate_segment(edge, graph.get_edge_data(edge[0], edge[1])['lane_width']))
            translated_segments_acw.append(translate_segment(edge, graph.get_edge_data(edge[0], edge[1])['lane_width'], anticlockwise=True))

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
            cw_start = tuple(geometry_functions.get_intersection_point(segment1, segment2))
            acw_start = tuple(geometry_functions.get_intersection_point(segment3, segment4))
            if graph.get_edge_data(edge[0], edge[1])['cw_start'] is None:
                nx.set_edge_attributes(graph, {(edge[0], edge[1]): {'cw_start': cw_start, 'acw_start': acw_start}})
                nx.set_edge_attributes(graph, {(edge[1], edge[0]): {'acw_end': cw_start, 'cw_end': acw_start}})
            # vertices.extend((cw_start, acw_start))
    for node in graph.nodes:
        edges = get_edges_cw(graph, node)
        for edge in edges:
            if graph.get_edge_data(edge[0], edge[1])['cw_end'][0] is not None:
                continue
            neighbor_edges = get_edges_cw(graph, edge[1])
            if len(neighbor_edges) == 1:
                cw_end = translate_segment(edge, graph.get_edge_data(edge[0], edge[1])['lane_width'])[1]
                acw_end = translate_segment(edge, graph.get_edge_data(edge[0], edge[1])['lane_width'], anticlockwise=True)[1]
                nx.set_edge_attributes(graph, {(edge[0], edge[1]): {'cw_end': cw_end, 'acw_end': acw_end}})
                nx.set_edge_attributes(graph, {(edge[1], edge[0]): {'acw_start': cw_end, 'cw_start': acw_end}})
                continue
            index = neighbor_edges.index(edge)
            prev = len(neighbor_edges) - 1 if index == 0 else index - 1
            cw_end = graph.get_edge_data(neighbor_edges[prev][0], neighbor_edges[prev][1])['cw_start']
            next_ = 0 if index == len(neighbor_edges) - 1 else index + 1
            acw_end = graph.get_edge_data(neighbor_edges[next_][0], neighbor_edges[next_][1])['acw_start']
            if graph.get_edge_data(edge[0], edge[1])['cw_end'] is None:
                nx.set_edge_attributes(graph, {(edge[0], edge[1]): {'cw_end': cw_end, 'acw_end': acw_end}})
                nx.set_edge_attributes(graph, {(edge[1], edge[0]): {'acw_start': cw_end, 'cw_start': acw_end}})
            # vertices.extend((cw_end, acw_end))

    for node in graph.nodes:
        edges = get_edges_cw(graph, node)
        for edge in edges:
            if graph.get_edge_data(edge[0], edge[1])['visited']:
                continue
            nx.set_edge_attributes(graph, {(edge[0], edge[1]): {'visited': True}})
            nx.set_edge_attributes(graph, {(edge[1], edge[0]): {'visited': True}})
            segments.extend(
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
            vertices.extend(
                (
                    (graph.get_edge_data(edge[0], edge[1])['cw_start']),
                    (graph.get_edge_data(edge[0], edge[1])['cw_end']),
                    (graph.get_edge_data(edge[0], edge[1])['acw_start']),
                    (graph.get_edge_data(edge[0], edge[1])['acw_end']),
                    # (edge[0]),
                    # (edge[1])
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

    unique_partitions = {frozenset(segment) for segment in partitions}
    unique_partitions = [list(segment) for segment in unique_partitions]
    return vertices, segments, unique_partitions


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
    unique_partitions = {frozenset(segment) for segment in partitions}
    unique_partitions = [list(segment) for segment in unique_partitions]
    segments.extend(unique_partitions)
    return vertices, segments
