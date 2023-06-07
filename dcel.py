import math as m
from shapely.geometry import Polygon, Point, LineString, MultiPoint
import random
import matplotlib.pyplot as plt
import traceback
import shapely
from geometry import get_intersection_point, get_interpolated_curve
from graph_to_road_network import get_translated_vertices_segments

CLOCKWISE = 0  # outside edges
ANTICLOCKWISE = 1  # inside edges
ROAD = 1
NON_ROAD = 2
BOUNDARY = 3
PARTITION = 4
JUNCTION = 5
ROAD_SEPARATOR = 6


class Face:
    def __init__(self):
        self.name = None
        self.outer_component = None  # One half edge of the outer-cycle
        self.fill_color = (random.random(), random.random(), random.random())
        self.polygon = None
        self.faces_inside = []
        self.adjacent_faces = []
        self.parent = None
        self.tag = NON_ROAD
        self.road_separator = None
        self.lane_separators = []
        self.lane_curves = []

    def get_face_vertices(self):  # returns vertices of outer boundary of face
        hedge = self.outer_component
        face_vertices = [[hedge.origin.x, hedge.origin.y]]
        hedge = hedge.next
        while hedge != self.outer_component:
            face_vertices.append((hedge.origin.x, hedge.origin.y))
            hedge = hedge.next
        return face_vertices

    def get_face_hedges(self):  # returns hedges of outer boundary of face along with hedges of inside faces
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
        while h.next != self.outer_component:  # Walk through all hedges of the cycle and set incident face
            if h.twin.incident_face.name != "BBox":
                adjacent_faces.append(h.twin.incident_face)
            h = h.next
        if h.twin.incident_face.name != "BBox":
            adjacent_faces.append(h.twin.incident_face)
        return adjacent_faces

    def is_polygon_inside(self, polygon2):
        try:
            if self.within(polygon2):
                return True
            elif polygon2.within(self):
                return True
        except shapely.errors.TopologicalError:
            for poly in [self, polygon2]:
                for i, pt in enumerate(poly.exterior.coords):
                    try:
                        poly.representative_point().within(polygon2)
                    except shapely.errors.TopologicalError:
                        continue
                    else:
                        face = poly.exterior.coords[i - 1:i + 2]
                        face_polygon = shapely.geometry.Polygon(face)
                        return poly.is_polygon_inside(face_polygon)
        return False

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
    translated_segment = [(new_start_x, new_start_y), (new_end_x, new_end_y)]
    return translated_segment


def get_edge_intersection_points(edge, polygon):
    exterior = polygon.exterior
    points = list(exterior.coords)
    polygon_edges = [(points[i], points[i + 1]) for i in range(len(points) - 1)]
    polygon_edges.append((points[-1], points[0]))
    min_x, min_y, max_x, max_y = polygon.bounds
    intersection_points = []
    for e in polygon_edges:
        point = get_intersection_point(e, edge)
        if point != [None] and min_x < point[0] < max_x and min_y < point[1] < max_y:
            intersection_points.append(point)
    return intersection_points


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
        self.__set_edge_tags()
        self.__set_adjacent_faces()
        self.__set_junctions_curves()

    def build_dcel_primary(self, points, segments):
        self.__add_points(points)
        self.__add_edges_and_twins(segments)
        self.__add_next_and_previous_pointers()

    def __set_edge_tags(self):
        for edge in self.edges:
            if edge.tag is not None:
                continue
            if edge.right_arrow.incident_face.tag in [ROAD, JUNCTION] and \
                    edge.left_arrow.incident_face.tag in [ROAD, JUNCTION]:
                edge.tag = PARTITION
            else:
                edge.tag = BOUNDARY

    def __set_adjacent_faces(self):
        for face in self.faces:
            face.adjacent_faces = face.get_adjacent_faces()

    def __set_face_tags(self):
        edges = list(self.graph.to_undirected().edges)
        for edge in edges:

            midpoint = ((edge[0][0] + edge[1][0]) / 2, (edge[0][1] + edge[1][1]) / 2)

            face = self.get_face_for_point(midpoint)
            if face is None:
                print("None face encountered in __set_face_tags method for edge ", edge)
                continue
            face.tag = ROAD
            face.road_separator = get_edge_intersection_points(edge, face.polygon)

            lane_width = self.graph.get_edge_data(edge[0], edge[1])['lane_width']

            face.lane_separators.append(
                get_edge_intersection_points(translate_segment(edge, 0.5 * lane_width), face.polygon))
            face.lane_separators.append(
                get_edge_intersection_points(translate_segment(edge, 0.5 * lane_width, True), face.polygon))

            face.lane_curves.append(
                get_edge_intersection_points(translate_segment(edge, 0.75 * lane_width), face.polygon))
            face.lane_curves.append(
                get_edge_intersection_points(translate_segment(edge, 0.25 * lane_width), face.polygon))
            face.lane_curves.append(
                get_edge_intersection_points(translate_segment(edge, 0.25 * lane_width, True), face.polygon))
            face.lane_curves.append(
                get_edge_intersection_points(translate_segment(edge, 0.75 * lane_width, True), face.polygon))

        nodes = [node for node in self.graph.nodes if self.graph.degree(node) > 4]
        for node in nodes:
            face = self.get_face_for_point(node)
            if face is None:
                print("None face encountered in __set_face_tags method at the end")
                continue
            face.tag = JUNCTION

    def __set_polygons(self):
        for face in self.faces:
            vertices = face.get_face_vertices()
            # print(vertices)
            face.polygon = Polygon(list(vertices))

    def show_dcel(self):
        for face in self.faces:
            # if not face.polygon.is_valid:

            x, y = face.polygon.exterior.xy
            plt.plot(x, y)
            if face.tag is not ROAD:
                continue
            x = [face.lane_curves[0][0][0], face.lane_curves[0][1][0]]
            y = [face.lane_curves[0][0][1], face.lane_curves[0][1][1]]
            plt.plot(x, y, color='blue', linewidth=0.5)
            plt.text(x[0], y[0], f"{x}, {y}")
            plt.text(x[-1], y[-1], f"{x}, {y}")

            x = [face.lane_curves[1][0][0], face.lane_curves[1][1][0]]
            y = [face.lane_curves[1][0][1], face.lane_curves[1][1][1]]
            plt.plot(x, y, color='blue', linewidth=0.5)
            plt.text(x[0], y[0], f"{x}, {y}")
            plt.text(x[0], y[0], f"{x}, {y}")
            break
            # plt.text(x[0], y[1], face.name, fontsize=6, color='black')

            # for i in range (len(x)):
            #     plt.text(x[i], y[i], f"{x[i]}, {y[i]}", fontsize=6, color='black')
            # plt.fill(x, y, color=face.fill_color)
        # for vertex in vertices:
        #     plt.scatter(vertex[0], vertex[1])
        plt.show()

    def show_road_network(self, axis=None):
        fig, ax = plt.subplots()
        if axis is not None:
            ax = axis
        for face in self.faces:
            if face.tag in [ROAD, JUNCTION]:
                x, y = face.polygon.exterior.xy
                # Plot the polygon and fill it with grey color
                ax.fill(x, y, color='grey', alpha=0.5, edgecolor='none')

            try:
                if face.tag == ROAD:
                    x = [face.road_separator[0][0], face.road_separator[1][0]]
                    y = [face.road_separator[0][1], face.road_separator[1][1]]
                    ax.plot(x, y, color='black')

                    # x_, y_ = face.polygon.centroid.x, face.polygon.centroid.y
                    # ax.text(x_, y_, face.name)

                    x = [face.lane_separators[0][0][0], face.lane_separators[0][1][0]]
                    y = [face.lane_separators[0][0][1], face.lane_separators[0][1][1]]
                    ax.plot(x, y, color='white', linewidth=3, linestyle='dashed')

                    x = [face.lane_separators[1][0][0], face.lane_separators[1][1][0]]
                    y = [face.lane_separators[1][0][1], face.lane_separators[1][1][1]]
                    ax.plot(x, y, color='white', linewidth=3, linestyle='dashed')

                    font_size = 6
                if face.tag in [ROAD, JUNCTION]:
                    for i in range(len(face.lane_curves)):
                        x = [face.lane_curves[i][0][0], face.lane_curves[i][1][0]]
                        y = [face.lane_curves[i][0][1], face.lane_curves[i][1][1]]

                        # ax.scatter(x[0], y[0], s=font_size)
                        # ax.text(x[0], y[0], f'{int(x[0])}, {int(y[0])}', fontsize=font_size)
                        #
                        # ax.scatter(x[1], y[1], s=font_size)
                        # ax.text(x[1], y[1], f'{int(x[1])}, {int(y[1])}', fontsize=font_size)

                        ax.plot(x, y, color='blue', linewidth=0.5)

            except Exception as e:
                print(e, "face: ", face.name, face, "\n---- in show_road_network")
        boundary_edges = [edge for edge in self.edges if edge.tag == BOUNDARY]
        for edge in boundary_edges:
            x = [edge.origin.x, edge.destination.x]
            y = [edge.origin.y, edge.destination.y]
            ax.plot(x, y, color='black')

            # ax.scatter(x[0], y[0], s=font_size)
            # ax.text(x[0], y[0], f'{int(x[0])}, {int(y[0])}', fontsize=font_size)
            # ax.scatter(x[1], y[1], s=font_size)
            # ax.text(x[1], y[1], f'{int(x[1])}, {int(y[1])}', fontsize=font_size)

        partition_edges = [edge for edge in self.edges if edge.tag == PARTITION]
        for edge in partition_edges:
            x = [edge.origin.x, edge.destination.x]
            y = [edge.origin.y, edge.destination.y]
            ax.plot(x, y, color='white', linewidth=0.2)

        ax.axis("equal")
        plt.show()

    def get_face_for_point(self, point):
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
                try:
                    polygon = Polygon(vertex_list)
                except Exception as e:
                    print(vertex_list)
                if polygon.area > max_face_area:  # Find largest face
                    max_face_area = polygon.area
                    max_face = f

        # The max face is actually the inner cycle of the outer face under assumption that faces
        # do not contain holes or are separated
        # self.faces.remove(max_face)
        # h = max_face.outer_component
        # h.incident_face = self.outer_face
        # self.outer_face.inner_component = h
        # while h.next != max_face.outer_component:
        #     h = h.next
        #     h.incident_face = self.outer_face

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
                except Exception as e:
                    print(face1.name, face2.name)
                    print(face1.get_face_vertices())
                    print(face2.get_face_vertices())
                    print(e)
                    traceback.print_exc()
                    return

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

            for point in list(lane_curve.coords):
                distance = Point(point).distance(polygon)
                if distance < min_distance:
                    min_distance = distance
                    closest_point = point

            return closest_point

        # visited_faces = []

        for face in self.faces:
            if face.tag == JUNCTION:
                for i in range(len(face.adjacent_faces)):
                    first_face = face.adjacent_faces[i]
                    # visited_faces.append(first_face)
                    potential_curves = []
                    for adj_face in face.adjacent_faces:
                        if adj_face != first_face:
                            potential_curves.append([adj_face, 0, adj_face.lane_curves[0]])
                            potential_curves.append([adj_face, 3, adj_face.lane_curves[3]])

                    current_curve = first_face.lane_curves[0]
                    master_curve = current_curve
                    for _ in range(len(face.adjacent_faces) - 1):

                        closest_curve, closest_face, index = find_closest_lane_curve(LineString(current_curve),
                                                                                     potential_curves)
                        # if closest_face in visited_faces:
                        #     continue
                        next_curve = closest_face.lane_curves[1] if index == 0 else closest_face.lane_curves[2]

                        closest_point1 = find_closest_point(LineString(master_curve), face.polygon)
                        closest_point2 = find_closest_point(LineString(closest_curve), face.polygon)

                        # interpolated_curve = get_interpolated_curve(master_curve, closest_curve)
                        # face.lane_curves.append(interpolated_curve)
                        face.lane_curves.append([closest_point1, closest_point2])

                        closest_point1 = find_closest_point(LineString(master_curve), face.polygon)
                        closest_point2 = find_closest_point(LineString(next_curve), face.polygon)

                        # interpolated_curve = get_interpolated_curve(master_curve, next_curve)
                        # face.lane_curves.append(interpolated_curve)
                        face.lane_curves.append([closest_point1, closest_point2])

                        closest_point1 = find_closest_point(LineString(first_face.lane_curves[1]), face.polygon)
                        closest_point2 = find_closest_point(LineString(closest_curve), face.polygon)

                        # interpolated_curve = get_interpolated_curve(first_face.lane_curves[1], closest_curve)
                        # face.lane_curves.append(interpolated_curve)
                        face.lane_curves.append([closest_point1, closest_point2])

                        closest_point1 = find_closest_point(LineString(first_face.lane_curves[1]), face.polygon)
                        closest_point2 = find_closest_point(LineString(next_curve), face.polygon)

                        # interpolated_curve = get_interpolated_curve(first_face.lane_curves[1], next_curve)
                        # face.lane_curves.append(interpolated_curve)
                        face.lane_curves.append([closest_point1, closest_point2])

                        current_curve = closest_face.lane_curves[3] if index == 0 else closest_face.lane_curves[0]

                        potential_curves.clear()
                        for adj_face in face.adjacent_faces:
                            if adj_face != closest_face:
                                potential_curves.append([adj_face, 0, adj_face.lane_curves[0]])
                                potential_curves.append([adj_face, 3, adj_face.lane_curves[3]])

                    potential_curves.clear()
                    for adj_face in face.adjacent_faces:
                        if adj_face != first_face:
                            potential_curves.append([adj_face, 0, adj_face.lane_curves[0]])
                            potential_curves.append([adj_face, 3, adj_face.lane_curves[3]])

                    current_curve = first_face.lane_curves[3]
                    master_curve = current_curve
                    for _ in range(len(face.adjacent_faces) - 1):
                        closest_curve, closest_face, index = find_closest_lane_curve(LineString(current_curve),
                                                                                     potential_curves)
                        next_curve = closest_face.lane_curves[1] if index == 0 else closest_face.lane_curves[2]

                        closest_point1 = find_closest_point(LineString(master_curve), face.polygon)
                        closest_point2 = find_closest_point(LineString(closest_curve), face.polygon)

                        # interpolated_curve = get_interpolated_curve(master_curve, closest_curve)
                        # face.lane_curves.append(interpolated_curve)
                        face.lane_curves.append([closest_point1, closest_point2])

                        closest_point1 = find_closest_point(LineString(master_curve), face.polygon)
                        closest_point2 = find_closest_point(LineString(next_curve), face.polygon)

                        # interpolated_curve = get_interpolated_curve(master_curve, next_curve)
                        # face.lane_curves.append(interpolated_curve)
                        face.lane_curves.append([closest_point1, closest_point2])

                        closest_point1 = find_closest_point(LineString(first_face.lane_curves[2]), face.polygon)
                        closest_point2 = find_closest_point(LineString(closest_curve), face.polygon)

                        # interpolated_curve = get_interpolated_curve(first_face.lane_curves[2], closest_curve)
                        # face.lane_curves.append(interpolated_curve)
                        face.lane_curves.append([closest_point1, closest_point2])

                        closest_point1 = find_closest_point(LineString(first_face.lane_curves[2]), face.polygon)
                        closest_point2 = find_closest_point(LineString(next_curve), face.polygon)

                        # interpolated_curve = get_interpolated_curve(first_face.lane_curves[2], next_curve)
                        # face.lane_curves.append(interpolated_curve)
                        face.lane_curves.append([closest_point1, closest_point2])

                        current_curve = closest_face.lane_curves[3] if index == 0 else closest_face.lane_curves[0]

                        potential_curves.clear()
                        for adj_face in face.adjacent_faces:
                            if adj_face != closest_face:
                                potential_curves.append([adj_face, 0, adj_face.lane_curves[0]])
                                potential_curves.append([adj_face, 3, adj_face.lane_curves[3]])
