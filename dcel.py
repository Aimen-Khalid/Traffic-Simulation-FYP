import math as m
from shapely.geometry import Polygon, Point
import random
import matplotlib.pyplot as plt

CLOCKWISE = 0  # outside
ANTICLOCKWISE = 1  # inside


class Face:
    def __init__(self):
        self.name = None
        self.outer_component = None  # One half edge of the outer-cycle
        self.fill_color = (random.random(), random.random(), random.random())
        self.polygon = None
        self.faces_inside = []
        self.parent = None

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
        l = m.sqrt(dx * dx + dy * dy)
        if dy > 0:
            return m.acos(dx / l)
        else:
            return 2 * m.pi - m.acos(dx / l)


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
    def __init__(self, x, y, name):
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
    def __init__(self):
        # (x coordinate, y coordinate) -> vertex
        self.vertices_map = {}
        self.hedges_map = HedgesMap()
        self.faces = []
        self.edges = []
        self.outer_face = OuterFace()

    def build_dcel(self, points, segments):
        self.__add_points(points)
        self.__add_edges_and_twins(segments)
        self.__add_next_and_previous_pointers()
        self.__add_face_pointers()
        self.__create_outer_face(points)
        self.__set_polygons()
        self.__set_hedges_direction()
        self.__set_faces_inside_faces()

    def __set_polygons(self):
        for face in self.faces:
            vertices = face.get_face_vertices()
            face.polygon = Polygon(list(vertices))

    def show_dcel(self):
        for face in self.faces:
            vertices = face.get_face_vertices()
            x = [x for x, y in vertices]
            y = [y for x, y in vertices]
            plt.fill(x, y, color=face.fill_color)
            for vertex in vertices:
                plt.scatter(vertex[0], vertex[1])
        plt.show()

    def get_face_for_point(self, point):
        point = Point(point[0], point[1])
        for face in self.faces:
            if point.within(face.polygon):
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
            if hedge.incident_face is None and hedge.twin.incident_face is None:
                vertex_list = [(hedge.origin.x, hedge.origin.y)]  # For calculating face size later

                number_of_faces += 1

                f = Face()
                f.name = f"f{number_of_faces}"

                f.outer_component = hedge
                hedge.incident_face = f

                h = hedge
                while h.next != hedge:  # Walk through all hedges of the cycle and set incident face
                    h.incident_face = f
                    h.twin.incident_face = f
                    h = h.next
                    vertex_list.append((h.origin.x, h.origin.y))
                h.incident_face = f
                h.twin.incident_face = f

                self.faces.append(f)

                # Calculate area of face formed by the half-edges
                polygon = Polygon(vertex_list)
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
                if face1.polygon.within(face2.polygon) and face1 != face2:
                    face2.faces_inside.append(face1)
                elif face2.polygon.within(face1.polygon) and face1 != face2:
                    face1.faces_inside.append(face2)

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
                while h.direction == CLOCKWISE and h.next != hedge:  # Walk through all hedges of the cycle and set incident face
                    if h.incident_face is not None:
                        if h.incident_face.parent is None:
                            h.incident_face.parent = self.outer_face
                        # h.incident_face.parent.outer_component = h
                        h.incident_face = h.incident_face.parent
                    h = h.next
                    h.visited = True
                if h.direction == CLOCKWISE:
                    h.incident_face = h.incident_face.parent

            # hedge.incident_face.outer_component = hedge
