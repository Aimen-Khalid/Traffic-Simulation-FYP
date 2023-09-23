import math as m
from matplotlib import patches
from shapely.geometry import Polygon, Point, LineString
from random import randint
import matplotlib.pyplot as plt
from RUSTIC.Utility.geometry import get_intersection_point, merge_line_strings, get_edge_intersection_points
from RUSTIC.MapToRoadMapping.generic_interpolation import path_interpolation
from RUSTIC.MapToRoadMapping.graph_to_road_network import get_translated_vertices_segments, translate_segment
import networkx as nx
from RUSTIC.settings import road_width

CLOCKWISE = 0  # outside edges
ANTICLOCKWISE = 1  # inside edges
ROAD = 1
NON_ROAD = 2
BOUNDARY = 3
PARTITION = 4
JUNCTION = 5
ROAD_SEPARATOR = 6


def update_curve(origin, curve):
    if Point(origin.x, origin.y).distance(Point(curve[0])) < Point(origin.x, origin.y).distance(Point(curve[1])):
        return curve
    curve[0], curve[1] = curve[1], curve[0]
    return curve


def get_angle(line):
    if line.coords[0][1] < line.coords[1][1]:
        st_index = 0
        end_index = 1
    elif line.coords[0][1] > line.coords[1][1]:
        st_index = 1
        end_index = 0
    else:
        return 0
    dx = line.coords[end_index][0] - line.coords[st_index][0]
    dy = line.coords[end_index][1] - line.coords[st_index][1]
    length = m.sqrt(dx * dx + dy * dy)
    return m.degrees(m.acos(dx / length) if dy > 0 else 2 * m.pi - m.acos(dx / length))


def get_node_from_graph(graph, attribute, value):
    matching_nodes = [node for node, attributes in graph.nodes(data=True) if
            attributes.get(attribute) == value]
    return matching_nodes[0]


def is_parallel(edge, lane_curve):
    edge_ls = LineString([(edge.origin.x, edge.origin.y), (edge.destination.x, edge.destination.y)])
    lane_curve_ls = LineString(lane_curve)
    tolerance = 0.01
    return abs(get_angle(edge_ls) - get_angle(lane_curve_ls)) < tolerance


class Face:
    def __init__(self):
        self.name = None
        self.outer_component = None  # One half edge of the outer-cycle
        self.polygon = None
        self.faces_inside = []
        self.adjacent_faces = []
        self.parent = None
        self.tag = NON_ROAD
        self.road_separator = None
        self.lane_separators = []
        self.lane_curves = []

    def get_face_vertices(self):
        """Returns vertices of outer boundary of face"""
        hedge = self.outer_component
        face_vertices = [[hedge.origin.x, hedge.origin.y]]
        hedge = hedge.next
        while hedge != self.outer_component:
            face_vertices.append((hedge.origin.x, hedge.origin.y))
            hedge = hedge.next
        return face_vertices

    def get_face_hedges(self):
        # returns hedges of outer boundary of face along with hedges of inside faces
        hedges = []
        h = self.outer_component.twin
        while h.next != self.outer_component.twin:
            hedges.append(h)
            h = h.next
        hedges.append(h)

        for inside_face in h.incident_face.faces_inside:
            h1 = inside_face.outer_component.twin
            while h1.next != inside_face.outer_component.twin:
                hedges.append(h1)
                h1 = h1.next
            hedges.append(h1)

        return hedges

    def get_adjacent_faces(self):
        adjacent_faces = []
        h = self.outer_component
        while h.next != self.outer_component:
            if h.twin.incident_face.name != "BBox":
                adjacent_faces.append(h.twin.incident_face)
            h = h.next
        if h.twin.incident_face.name != "BBox":
            adjacent_faces.append(h.twin.incident_face)
        return adjacent_faces

    def set_curves_direction_helper(self, edge):
        edge_ls = LineString([(edge.origin.x, edge.origin.y), (edge.destination.x, edge.destination.y)])
        if LineString(self.lane_curves[0][:2]).distance(edge_ls) < LineString(self.lane_curves[3][:2]).distance(edge_ls):
            closest_curve_index = 0
        else:
            closest_curve_index = 3
        self.lane_curves[closest_curve_index] = update_curve(edge.origin, self.lane_curves[closest_curve_index])
        next_curve_index = 1 if closest_curve_index == 0 else 2
        self.lane_curves[next_curve_index] = update_curve(edge.origin, self.lane_curves[next_curve_index])

    def set_curves_direction(self):
        """Sets the direction of lane curves for roads"""
        if self.tag != ROAD:
            return
        boundary_edges = []
        edge = self.outer_component
        while edge.next != self.outer_component:
            if edge.tag == BOUNDARY and is_parallel(edge, self.lane_curves[0]):
                boundary_edges.append(edge)
            edge = edge.next
        if edge.tag == BOUNDARY and is_parallel(edge, self.lane_curves[0]):
            boundary_edges.append(edge)
        self.set_curves_direction_helper(boundary_edges[0].twin)
        self.set_curves_direction_helper(boundary_edges[1].twin)

    def set_junction_curves_direction(self):
        if self.tag != JUNCTION:
            return
        updated_curves = []
        for adj_face in self.adjacent_faces:
            for i in range(4):
                if self.polygon.distance(Point(adj_face.lane_curves[i][1])) < 0.01:
                    emerging_curves = []
                    for lane_curve in self.lane_curves:
                        if Point(adj_face.lane_curves[i][1]).distance(Point(lane_curve[0])) < 0.5:
                            emerging_curves.append(lane_curve)
                        if Point(adj_face.lane_curves[i][1]).distance(Point(lane_curve[1])) < 0.5:
                            lane_curve[0], lane_curve[1] = lane_curve[1], lane_curve[0]
                            emerging_curves.append(lane_curve)
                    updated_curves.extend(emerging_curves)
        self.lane_curves = updated_curves

    def get_connected_curves(self, lane_curve):
        """Returns all lane curves that emerge from the input lane curve"""
        connected_curves = []
        for adj_face in self.adjacent_faces:
            if adj_face.name == 'BBox':
                continue
            for adj_face_curve in adj_face.lane_curves:
                end_point = (round(adj_face_curve[0][0], 2), round(adj_face_curve[0][1], 2))
                start_point = (round(lane_curve.coords[-1][0], 2), round(lane_curve.coords[-1][1], 2))
                if end_point == start_point:
                    connected_curves.append(adj_face_curve)
        return connected_curves

    def __repr__(self):
        return f"Face : (n[{self.name}], outer[{self.outer_component.origin.x}, {self.outer_component.origin.y}])"

    def __eq__(self, rhs):
        return self.name is rhs.name and self.name is rhs.name


class HalfEdge:
    def __init__(self, origin, destination):
        self.origin = origin
        self.destination = destination
        self.incident_face = None
        self.twin = None
        self.next = None
        self.prev = None
        self.tag = None
        self.direction = None
        self.visited = False

    def __repr__(self):
        return f"E(o:[{self.origin.x}, {self.origin.y}], d:[{self.destination.x}, {self.destination.y}])"

    def __eq__(self, rhs):
        return self.origin is rhs.origin and self.destination is rhs.destination

    def get_length(self):
        return m.sqrt((self.destination.x - self.origin.x) ** 2 + (self.destination.y - self.origin.y) ** 2)

    def get_angle(self):
        dx = self.destination.x - self.origin.x
        dy = self.destination.y - self.origin.y
        length = m.sqrt(dx * dx + dy * dy)
        return m.acos(dx / length) if dy > 0 else 2 * m.pi - m.acos(dx / length)


class Edge:
    def __init__(self, half_edge1, half_edge2):
        if half_edge1.destination.x > half_edge2.destination.x:
            self.right_arrow = half_edge1
            self.left_arrow = half_edge2
        else:
            self.right_arrow = half_edge2
            self.left_arrow = half_edge1

        # Assumed is that of undirected edges, the destination is the `right-most' endpoint
        self.origin = self.right_arrow.origin
        # Assumed is that of undirected edges, the origin is the `left-most' endpoint
        self.destination = self.right_arrow.destination
        self.tag = None

    def __repr__(self):
        return f"Edge: [ ({self.origin.x}, {self.origin.y}), ({self.destination.x}, {self.destination.y})]"

    def get_edge_length(self):
        return self.right_arrow.get_length()

    def get_y_at_x(self, x):
        # In case the x coordinate lies outside of the range of the line return None
        if x < self.origin.x or x > self.destination.x:
            return None

        slope = self.get_slope()
        y_at_x = slope * (x - self.origin.x) + self.origin.y
        return y_at_x

    def get_slope(self):
        edge_x_width = self.destination.x - self.origin.x
        return (self.destination.y - self.origin.y) / edge_x_width

    def point_lies_above_edge(self, point):
        return point.y > self.get_y_at_x(point.x)

    def point_lies_on_edge(self, point):
        if point.x < self.origin.x or point.x > self.destination.x:
            return False
        elif self.get_y_at_x(point.x) == point.y:
            return True
        else:
            return False


class Vertex:
    def __init__(self, x, y, name='none'):
        self.x = x
        self.y = y
        self.name = name

    def __repr__(self):
        return f"Vertex coords: ({self.x}, {self.y})"

    def __eq__(self, rhs):
        return self.x == rhs.x and self.y == rhs.y

    def __hash__(self):
        return hash(self.x) + hash(self.y)


# Mapping used to quickly find hedge belonging to a certain origin and destination
class HedgesMap:
    def __init__(self):
        self.origin_destination_map = {}
        self.destination_origin_map = {}

    def insert_hedge(self, origin, destination, hedge):
        self.origin_destination_map.setdefault(origin, {})
        self.origin_destination_map[origin][destination] = hedge

        self.destination_origin_map.setdefault(destination, {})
        self.destination_origin_map[destination][origin] = hedge

    def get_hedge(self, origin, destination):
        return self.origin_destination_map[origin][destination]

    def get_outgoing_hedges(self, origin):
        return list(self.origin_destination_map[origin].values())

    def get_incoming_hedges(self, destination):
        return list(self.destination_origin_map[destination].values())

    # Returns outgoing half edges in clockwise order
    def get_outgoing_hedges_clockwise(self, origin):
        outgoing_hedges = list(self.origin_destination_map[origin].values())
        outgoing_hedges.sort(key=lambda e: e.get_angle(), reverse=True)
        return outgoing_hedges

    # Returns incoming half edges in clockwise order
    def get_incoming_hedges_clockwise(self, destination):
        incoming_hedges = list(self.destination_origin_map[destination].values())
        incoming_hedges.sort(key=lambda e: e.get_angle(), reverse=True)
        return incoming_hedges

    # Returns all the incoming and outgoing half edges
    def get_all_hedges_of_vertex(self, vertex):
        return self.get_incoming_hedges_clockwise(
            vertex
        ) + self.get_outgoing_hedges(vertex)

    # Returns all hedges of the mapping
    def get_all_hedges(self):
        all_hedges = []
        for key, hedges_dic in self.origin_destination_map.items():
            all_hedges = all_hedges + (list(hedges_dic.values()))
        return all_hedges

    # Deletes half edge from the mapping
    def delete_hedge(self, origin, destination):
        del self.origin_destination_map[origin][destination]
        del self.destination_origin_map[destination][origin]


class OuterFace(Face):
    def __init__(self):
        super().__init__()
        self.name = "BBox"
        self.upper_left = None
        self.bottom_left = None
        self.upper_right = None
        self.bottom_right = None
        self.segments = None
        self.top_segment = None
        self.bottom_segment = None
        self.inner_component = None
        self.tag = NON_ROAD

    def set_vertices(self, vertices):
        self.upper_left = vertices[0]
        self.bottom_left = vertices[1]
        self.upper_right = vertices[2]
        self.bottom_right = vertices[3]

    def set_edges(self, edges):
        self.segments = edges
        self.top_segment = edges[0]
        self.bottom_segment = edges[2]
        self.outer_component = edges[2].right_arrow


class Dcel:
    def __init__(self, graph=None):
        # (x coordinate, y coordinate) -> vertex
        self.graph = graph
        self.vertices_map = {}
        self.hedges_map = HedgesMap()
        self.faces = []
        self.edges = []
        self.outer_face = OuterFace()

    def build_dcel(self, graph):
        print('Creating DCEL...')
        vertices, segments = get_translated_vertices_segments(graph)
        self.__add_points(vertices)
        self.__add_edges_and_twins(segments)
        self.__add_next_and_previous_pointers()
        self.__add_face_pointers()
        self.__create_outer_face(vertices)
        self.__set_polygons()
        self.__set_hedges_direction()
        self.__set_faces_inside_faces()
        self.__set_face_tags()
        self.__set_edges_tags()
        self.__set_faces_curves()
        self.__set_adjacent_faces()
        self.__set_junctions_curves()
        print('DCEL created.')

    def __set_edges_tags(self):
        for edge in self.edges:
            if edge.tag is not None:
                continue
            if edge.right_arrow.incident_face.tag in [ROAD, JUNCTION] and \
                    edge.left_arrow.incident_face.tag in [ROAD, JUNCTION]:
                edge.tag = edge.left_arrow.tag = edge.right_arrow.tag = PARTITION
            else:
                edge.tag = edge.left_arrow.tag = edge.right_arrow.tag = BOUNDARY

    def __set_adjacent_faces(self):
        for face in self.faces:
            face.adjacent_faces = face.get_adjacent_faces()

    def __set_faces_curves(self):
        edges = list(self.graph.to_undirected().edges)
        for edge in edges:
            midpoint = ((edge[0][0] + edge[1][0]) / 2, (edge[0][1] + edge[1][1]) / 2)
            face = self.get_face_for_point(midpoint)
            if face is None:
                print('none face in set face curves')
                return
            face.road_separator = get_edge_intersection_points(edge, face.polygon)

            face.lane_separators.append(
                get_edge_intersection_points(translate_segment(edge, 0.5 * road_width), face.polygon))
            face.lane_separators.append(
                get_edge_intersection_points(translate_segment(edge, 0.5 * road_width, True), face.polygon))

            face.lane_curves.append(
                get_edge_intersection_points(translate_segment(edge, 0.75 * road_width), face.polygon))
            face.lane_curves.append(
                get_edge_intersection_points(translate_segment(edge, 0.25 * road_width), face.polygon))
            face.lane_curves.append(
                get_edge_intersection_points(translate_segment(edge, 0.25 * road_width, True), face.polygon))
            face.lane_curves.append(
                get_edge_intersection_points(translate_segment(edge, 0.75 * road_width, True), face.polygon))
            face.set_curves_direction()

            for i in range(len(face.lane_curves)):
                face.lane_curves[i] = face.lane_curves[i][:2]

    def __set_face_tags(self):

        edges = list(self.graph.to_undirected().edges)
        for edge in edges:
            midpoint = ((edge[0][0] + edge[1][0]) / 2, (edge[0][1] + edge[1][1]) / 2)

            face = self.get_face_for_point(midpoint)
            if face is None:
                print('Error: None face in set face tags')
                return
            face.tag = ROAD

        nodes = [node for node in self.graph.nodes if self.graph.degree(node) > 4]
        for node in nodes:
            face = self.get_face_for_point(node)
            if face is None:
                print('Error: None face in set face tags')
                return
            face.tag = JUNCTION

    def __set_polygons(self):
        for face in self.faces:
            vertices = face.get_face_vertices()
            # print(vertices)
            face.polygon = Polygon(list(vertices))

    def add_text_to_plot(self, ax):
        def add_face_name():
            x_, y_ = face.polygon.centroid.x, face.polygon.centroid.y
            ax.text(x_, y_, face.name)

        def add_coordinates():
            if face.tag in [ROAD, JUNCTION]:
                for i in range(len(face.lane_curves)):
                    x = [face.lane_curves[i][0][0], face.lane_curves[i][1][0]]
                    y = [face.lane_curves[i][0][1], face.lane_curves[i][1][1]]

                    x, y = LineString(face.lane_curves[i]).xy

                    ax.scatter(x[0], y[0], s=font_size)
                    ax.text(x[0], y[0], f'{int(x[0])}, {int(y[0])}', fontsize=font_size)

                    ax.scatter(x[1], y[1], s=font_size)
                    ax.text(x[-1], y[-1], f'{int(x[1])}, {int(y[1])}', fontsize=font_size)

            boundary_edges = [edge for edge in self.edges if edge.tag == BOUNDARY]
            for edge in boundary_edges:
                x = [edge.origin.x, edge.destination.x]
                y = [edge.origin.y, edge.destination.y]
                ax.plot(x, y, color='black')
                ax.scatter(x[0], y[0], s=font_size)
                ax.text(x[0], y[0], f'{int(x[0])}, {int(y[0])}', fontsize=font_size)
                ax.scatter(x[1], y[1], s=font_size)
                ax.text(x[1], y[1], f'{int(x[1])}, {int(y[1])}', fontsize=font_size)

        font_size = 8
        for face in self.faces:
            add_face_name()
            add_coordinates()

    def plot_separators(self, ax, face):
        x = [face.road_separator[0][0], face.road_separator[1][0]]
        y = [face.road_separator[0][1], face.road_separator[1][1]]
        ax.plot(x, y, color='black')

        x = [face.lane_separators[0][0][0], face.lane_separators[0][1][0]]
        y = [face.lane_separators[0][0][1], face.lane_separators[0][1][1]]
        ax.plot(x, y, color='white', linewidth=3, linestyle='dashed', dashes=[5, 5], zorder=10)

        x = [face.lane_separators[1][0][0], face.lane_separators[1][1][0]]
        y = [face.lane_separators[1][0][1], face.lane_separators[1][1][1]]
        ax.plot(x, y, color='white', linewidth=3, linestyle='dashed', dashes=[5, 5], zorder=10)
        plt.draw()

    def plot_curves(self, ax, face):
        def plot_junction_curves():
            x, y = LineString(face.lane_curves[i]).xy
            ax.plot(x, y, color='blue', linewidth=0.5)
            for j in range(len(x) - 1):
                ax.text(x[j], y[j], f'{int(x[j])}, {int(y[j])}', fontsize=6)
                ax.scatter(x, y, color='red', s=0.8)

        for i in range(len(face.lane_curves)):
            x = [face.lane_curves[i][0][0], face.lane_curves[i][1][0]]
            y = [face.lane_curves[i][0][1], face.lane_curves[i][1][1]]
            if face.tag == JUNCTION:
                plot_junction_curves()
            else:
                start_point = (x[0], y[0])
                end_point = (x[1], y[1])
                ax.plot(x, y, color='blue', linewidth=0.5)
                # Add arrowhead manually
                arrow_direction = (end_point[0] - start_point[0],
                                   end_point[1] - start_point[1])
                # # Create an arrow patch
                arrow_patch = patches.FancyArrow(start_point[0], start_point[1], arrow_direction[0], arrow_direction[1],
                                                 head_width=1, head_length=1, linewidth=0.5, color='blue', zorder=10)
                # # Add the arrow patch to the axis
                ax.add_patch(arrow_patch)

    def plot_edges(self, ax):
        boundary_edges = [edge for edge in self.edges if edge.tag == BOUNDARY]
        for edge in boundary_edges:
            x = [edge.origin.x, edge.destination.x]
            y = [edge.origin.y, edge.destination.y]
            ax.plot(x, y, color='black')
        partition_edges = [edge for edge in self.edges if edge.tag == PARTITION]
        # for edge in partition_edges:
        #     x = [edge.origin.x, edge.destination.x]
        #     y = [edge.origin.y, edge.destination.y]
        #     ax.plot(x, y, color='white', linewidth=0.5)

    def show_road_network(self, axis=None, figure=None):
        print('Plotting DCEL...')
        fig, ax = plt.subplots()
        if axis is not None:
            ax = axis

        if figure is not None:
            fig = figure
        self.add_text_to_plot(ax)

        plt.subplots_adjust(left=0, right=1, top=1, bottom=0)
        for face in self.faces:
            if face.tag in [ROAD, JUNCTION]:
                x, y = face.polygon.exterior.xy
                color = '#807E78'
                ax.fill(x, y, color=color, edgecolor=color)

            if face.tag == ROAD:
                self.plot_separators(ax, face)
            if face.tag in [ROAD, JUNCTION]:
                self.plot_curves(ax, face)
        self.plot_edges(ax)
        ax.axis("equal")
        ax.axis("off")
        print('DCEL plotted.')
        if figure is None:
            plt.show()

    def get_face_for_point(self, point):
        if not isinstance(point, Point):
            point = Point(point[0], point[1])
        for face in self.faces:
            if point.within(face.polygon) or face.polygon.boundary.contains(point):
                return face

    def get_vertices(self):
        return list(self.vertices_map.values())

    def get_edges(self):
        return self.edges

    def __add_points(self, points):
        # Creates a hashmap (x coordinate, y coordinate) -> vertex
        label = 'A'
        for point in points:
            self.vertices_map[point] = Vertex(point[0], point[1], label)
            label = chr(ord(label) + 1)

    def __add_edges_and_twins(self, segments):
        # Connects vertices and hedges and assign twins
        for segment in segments:
            origin = self.vertices_map[segment[0]]
            destination = self.vertices_map[segment[1]]

            hedge = HalfEdge(origin, destination)
            twin_hedge = HalfEdge(destination, origin)

            hedge.twin = twin_hedge
            twin_hedge.twin = hedge

            self.hedges_map.insert_hedge(hedge.origin, hedge.destination, hedge)
            self.hedges_map.insert_hedge(twin_hedge.origin, twin_hedge.destination, twin_hedge)

            self.edges.append(Edge(hedge, twin_hedge))

    def __create_outer_face(self, points):
        min_x = points[0][0]
        max_x = points[0][0]
        min_y = points[0][1]
        max_y = points[0][1]
        for point in points:
            if point[0] < min_x: min_x = point[0]
            if point[0] > max_x: max_x = point[0]
            if point[1] < min_y: min_y = point[1]
            if point[1] > max_y: max_y = point[1]

        bounding_box_upper_left = Vertex(min_x - 1, max_y + 1, "ul")
        bounding_box_lower_left = Vertex(min_x - 1, min_y - 1, "ll")
        bounding_box_upper_right = Vertex(max_x + 1, max_y + 1, "rr")
        bounding_box_lower_right = Vertex(max_x + 1, min_y - 1, "lr")

        outer_face_vertices = [
            bounding_box_upper_left,
            bounding_box_lower_left,
            bounding_box_upper_right,
            bounding_box_lower_right,
        ]
        self.outer_face.set_vertices(outer_face_vertices)

        hedge = HalfEdge(bounding_box_upper_left, bounding_box_upper_right)
        twin_hedge = HalfEdge(bounding_box_upper_right, bounding_box_upper_left)
        twin_hedge.incident_face = self.outer_face
        hedge.twin = twin_hedge
        twin_hedge.twin = hedge
        outer_face_edges = [Edge(hedge, twin_hedge)]
        hedge = HalfEdge(bounding_box_upper_right, bounding_box_lower_right)
        twin_hedge = HalfEdge(bounding_box_lower_right, bounding_box_upper_right)
        hedge.twin = twin_hedge
        twin_hedge.twin = hedge
        outer_face_edges.append(Edge(hedge, twin_hedge))

        hedge = HalfEdge(bounding_box_lower_right, bounding_box_lower_left)
        twin_hedge = HalfEdge(bounding_box_lower_left, bounding_box_lower_right)
        twin_hedge.incident_face = self.outer_face
        hedge.twin = twin_hedge
        twin_hedge.twin = hedge
        outer_face_edges.append(Edge(hedge, twin_hedge))

        hedge = HalfEdge(bounding_box_lower_left, bounding_box_upper_left)
        twin_hedge = HalfEdge(bounding_box_upper_left, bounding_box_lower_left)
        hedge.twin = twin_hedge
        twin_hedge.twin = hedge
        outer_face_edges.append(Edge(hedge, twin_hedge))

        self.outer_face.set_edges(outer_face_edges)

    def __add_next_and_previous_pointers(self):
        # Identify next and previous half edges
        for vertex in list(self.vertices_map.values()):
            outgoing_hedges = self.hedges_map.get_outgoing_hedges_clockwise(vertex)
            # Consider the outgoing half edges in clockwise order
            # Assign to the twin of each outgoing half edge the next outgoing half edge
            for i in range(len(outgoing_hedges)):
                h1 = outgoing_hedges[i]
                h2 = outgoing_hedges[(i + 1) % len(outgoing_hedges)]

                h1.twin.next = h2
                h2.prev = h1.twin

    def __add_face_pointers(self):
        # Create a face for every cycle of half edges
        number_of_faces = 0
        max_face = None
        max_face_area = 0
        for hedge in self.hedges_map.get_all_hedges():
            if hedge.incident_face is None:
                vertex_list = [(hedge.origin.x, hedge.origin.y)]  # For calculating face size later

                number_of_faces += 1

                f = Face()
                f.name = f"f{number_of_faces}"

                f.outer_component = hedge
                hedge.incident_face = f

                h = hedge
                while h.next != hedge:  # Walk through all hedges of the cycle and set incident face
                    h.incident_face = f
                    # h.twin.incident_face = f
                    h = h.next
                    vertex_list.append((h.origin.x, h.origin.y))
                h.incident_face = f
                # h.twin.incident_face = f

                self.faces.append(f)

                # Calculate area of face formed by the half-edges

                polygon = Polygon(vertex_list)

                if polygon.area > max_face_area:  # Find largest face
                    max_face_area = polygon.area
                    max_face = f

    def __set_hedges_direction(self):
        for hedge in self.hedges_map.get_all_hedges():
            if hedge.direction is None:
                edges_sum = 0
                h = hedge
                while h.next != hedge:  # Walk through all hedges of the cycle and find edges sum
                    edges_sum += (h.next.origin.x - h.origin.x) * (h.next.origin.y + h.origin.y)  # (x2 − x1)(y2 + y1)
                    h = h.next
                edges_sum += (h.next.origin.x - h.origin.x) * (h.next.origin.y + h.origin.y)
                direction = CLOCKWISE if edges_sum > 0 else ANTICLOCKWISE

                h = hedge
                while h.next != hedge:
                    h.direction = direction
                    h = h.next

    def __set_faces_inside_faces(self):
        for i, face1 in enumerate(self.faces):
            for j in range(i + 1, len(self.faces)):
                face2 = self.faces[j]
                # Check if face1 is contained within face2 or vice versa

                try:
                    if face1.polygon.within(face2.polygon) and face1 != face2:
                        face2.faces_inside.append(face1)
                    elif face2.polygon.within(face1.polygon) and face1 != face2:
                        face1.faces_inside.append(face2)
                except Exception:
                    print("reduce lane width. error in set faces inside faces function in dcel")
                    # raise ValueError("Increase the scaling factor or decrease the line width. The created road faces are overlapping")


        # Loop over each face
        for i in range(len(self.faces)):
            # Sort the inside faces of this face in descending order by area
            self.faces[i].faces_inside.sort(key=lambda f: f.polygon.area, reverse=True)

            # Loop over each inside face
            for inside_face in self.faces[i].faces_inside:
                inside_face.parent = self.faces[i]
                for nested_inside_face in inside_face.faces_inside:
                    nested_inside_face.parent = inside_face
                    self.faces[i].faces_inside.remove(nested_inside_face)

        for hedge in self.hedges_map.get_all_hedges():
            if not hedge.visited:
                h = hedge
                h.visited = True
                while h.direction == CLOCKWISE and h.next != hedge:
                    if h.incident_face is not None:
                        if h.incident_face.parent is None:
                            self.faces.remove(h.incident_face)
                            h.incident_face.parent = self.outer_face
                        # h.incident_face.parent.outer_component = h
                        h.incident_face = h.incident_face.parent
                    h = h.next
                    h.visited = True
                if h.direction == CLOCKWISE:
                    h.incident_face = h.incident_face.parent

            # hedge.incident_face.outer_component = hedge

    def __set_junctions_curves(self):

        def find_closest_lane_curve(target_lane_curve, lane_curves_list):
            min_distance = float('inf')
            closest_lane_curve = None
            closest_face = None
            index = None

            for item in lane_curves_list:
                lane_curve = item[2]
                distance = target_lane_curve.distance(LineString(lane_curve))
                if distance < min_distance:
                    min_distance = distance
                    closest_lane_curve = lane_curve
                    closest_face = item[0]
                    index = item[1]

            return closest_lane_curve, closest_face, index

        def find_closest_point(lane_curve, polygon):
            min_distance = float('inf')
            closest_point = None

            for point in list(lane_curve.coords)[:2]:
                distance = Point(point).distance(polygon)
                if distance < min_distance:
                    min_distance = distance
                    closest_point = point

            return closest_point

        def get_potential_closest_curves(starting_face):
            potential_curves = []
            for adj_face in face.adjacent_faces:
                if adj_face != starting_face:
                    potential_curves.append([adj_face, 0, adj_face.lane_curves[0]])
                    potential_curves.append([adj_face, 3, adj_face.lane_curves[3]])
            return potential_curves

        def connect_curves(curve1, curve2):
            if interpolate:
                interpolated_curve = path_interpolation(LineString(curve1), LineString(curve2), no_of_points)
                face.lane_curves.append(interpolated_curve)
            else:
                closest_point1 = find_closest_point(LineString(curve1), face.polygon)
                closest_point2 = find_closest_point(LineString(curve2), face.polygon)
                face.lane_curves.append([closest_point1, closest_point2])

        def connect_curves_sets(first_curve_index):
            potential_curves = get_potential_closest_curves(starting_face)
            try:
                current_curve = starting_face.lane_curves[first_curve_index]
            except Exception:
                raise ValueError("Decrease the lane width or increase scaling factor. Road polygons are overlapping.")
            master_curve = current_curve
            master_next_curve = starting_face.lane_curves[1 if first_curve_index == 0 else 2]
            for _ in range(len(face.adjacent_faces) - 1):
                closest_curve, closest_face, index = find_closest_lane_curve(LineString(current_curve),
                                                                             potential_curves)
                next_curve = closest_face.lane_curves[1] if index == 0 else closest_face.lane_curves[2]

                connect_curves(master_curve, closest_curve)
                connect_curves(master_curve, next_curve)
                connect_curves(master_next_curve, closest_curve)
                connect_curves(master_next_curve, next_curve)

                current_curve = closest_face.lane_curves[3 if index == 0 else 0]
                potential_curves = get_potential_closest_curves(closest_face)

        interpolate = True
        no_of_points = 5

        for face in self.faces:
            if face.tag == JUNCTION:
                for i in range(len(face.adjacent_faces)):
                    starting_face = face.adjacent_faces[i]
                    connect_curves_sets(first_curve_index=0)  # connects sets of two lane curves having same direction
                    connect_curves_sets(first_curve_index=3)
                face.set_junction_curves_direction()

    def get_track(self, start_node_id, end_node_id):
        def get_lane_curve(target_point, lane_curves_list):
            """Returns that outer-most lane curve from lane_curves_list the origin of which
            is closest to the target point"""
            min_distance = float('inf')
            closest_lane_curve = None
            index = None
            for i, lane_curve in enumerate(lane_curves_list):
                distance = Point(target_point).distance(Point(lane_curve[0]))
                if distance < min_distance:
                    min_distance = distance
                    closest_lane_curve = lane_curve
                    index = i
            if index == 1:
                return lane_curves_list[0]
            if index == 2:
                return lane_curves_list[3]
            # return lane_curves_list[index]

        def get_face_sequence():
            face_sequence = []
            for i, node in enumerate(shortest_path_nodes[:-1]):
                mid_point = ((shortest_path_nodes[i][0] + shortest_path_nodes[i + 1][0]) / 2,
                             (shortest_path_nodes[i][1] + shortest_path_nodes[i + 1][1]) / 2)
                face = self.get_face_for_point(mid_point)
                face_sequence.append((face, node))

                # Check if end point of the segment is contained in a junction
                end_node = shortest_path_nodes[i + 1]
                if self.graph.degree[end_node] > 4:
                    face = self.get_face_for_point(end_node)
                    face_sequence.append((face, end_node))
            return face_sequence

        def create_track_from_faces(face_sequence):
            track = LineString([])
            for i, (face, node) in enumerate(face_sequence):
                if face.tag == ROAD:
                    face_curve = LineString(get_lane_curve(node, face.lane_curves))
                    # face_curve = obstacle_avoidance.modify_reference_track_for_obstacles(obstacles, face_curve,
                    #                                                                      face.polygon)
                    track = merge_line_strings(LineString(track), face_curve)
                if face.tag == JUNCTION:
                    if i == len(face_sequence) - 1:
                        return track
                    start_point = track.coords[-1]
                    end_point = get_lane_curve(node, face_sequence[i + 1][0].lane_curves)[0]
                    track_part = LineString([start_point, end_point])
                    start_point = (round(start_point[0], 2), round(start_point[1], 2))
                    end_point = (round(end_point[0], 2), round(end_point[1], 2))
                    for lane_curve in face.lane_curves:
                        curve_start = (round(lane_curve[0][0], 2), round(lane_curve[0][1], 2))
                        curve_end = (round(lane_curve[-1][0], 2), round(lane_curve[-1][1], 2))
                        if curve_start == start_point and curve_end == end_point:
                            track_part = LineString(lane_curve)

                    # track_part = obstacle_avoidance.modify_reference_track_for_obstacles(obstacles, track_part,
                    #                                                                      face.polygon)

                    track = merge_line_strings(LineString(track), track_part)
            return track

        start_node = get_node_from_graph(self.graph, 'id', start_node_id)
        end_node = get_node_from_graph(self.graph, 'id', end_node_id)

        shortest_path_nodes = nx.shortest_path(self.graph, start_node, end_node)
        if not shortest_path_nodes:
            return None
        # shortest_path_nodes.extend(nx.shortest_path(self.graph, end_node, start_node))
        face_sequence = get_face_sequence()
        # points = drawing_tool.get_points_on_graph(self, 'cars_initiation')
        # face_sequence = []
        # for point in points:
        #     f = self.get_face_for_point(point)
        #     face_sequence.append((f, point))
        track = create_track_from_faces(face_sequence)
        return track








    def get_track1(self, start_node_id, end_node_id, obstacles):
        track = LineString([])
        road_faces = [face for face in self.faces if face.tag == ROAD]

        lane_curve = LineString(road_faces[randint(0, len(road_faces)-1)].lane_curves[randint(0, 3)])

        start_point = lane_curve.coords[0]
        end_point = lane_curve.coords[1]
        i = 0
        while i < 8 or Point(track.coords[0]).distance(Point(track.coords[-1])) < 10:

            track_part = lane_curve  #LineString([start_point, end_point])
            mid_point = ((start_point[0] + end_point[0]) / 2,
                         (start_point[1] + end_point[1]) / 2)
            face = self.get_face_for_point(mid_point)
            # try:
            #     track_part = obstacle_avoidance.modify_reference_track_for_obstacles(obstacles, track_part, face.polygon)
            # except Exception:
            #     ValueError("Obstacles not created correctly")
            track = merge_line_strings(LineString(track), track_part)
            connected_curves = face.get_connected_curves(LineString(lane_curve))
            if len(connected_curves) == 0:
                return track
            if not connected_curves:
                print(face.name)
            lane_curve = LineString(connected_curves[randint(0, len(connected_curves))-1])
            # lane_curve = LineString(connected_curves[0])
            start_point = lane_curve.coords[0]
            end_point = lane_curve.coords[1]
            i += 1

        return track



