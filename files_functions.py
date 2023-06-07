import osmnx
from shapely import Polygon
import networkx as nx
from geometry import lane_width


def write_vertices_to_file(vertices, file_name):
    with open(file_name, "w") as file:
        for vertex in vertices:
            for coordinate in vertex:
                file.write(str(coordinate) + "\t")
            file.write("\n")


def load_vertices_from_file(file_name):
    vertices = []
    with open(file_name, "r") as file:
        for line in file:
            vertex = tuple(float(x) for x in line.strip().split('\t'))
            vertices.append(vertex)
    return vertices


def write_segments_to_file(segments, file_name):
    with open(file_name, "w") as file:
        for segment in segments:
            for vertices in segment:
                for vertex in vertices:
                    file.write(str(vertex) + "\t")
            file.write("\n")


def load_segments_from_file(file_name):
    segments = []
    with open(file_name, "r") as file:
        for line in file:
            vertices = [float(x) for x in line.strip().split("\t")]
            segment = [(vertices[0], vertices[1]), (vertices[2], vertices[3])]
            segments.append(segment)
    return segments


def create_verts_and_segs_files_from_coords(coordinates, vertices_fn, segments_fn):
    tags = {
        "highway": ["primary", "secondary", "tertiary", "residential"]
    }

    geom = Polygon(coordinates)

    gdf = osmnx.geometries_from_polygon(geom, tags=tags)
    gdf = osmnx.projection.project_gdf(gdf, to_crs='EPSG:3857', to_latlong=False)
    gdf = gdf[['name', 'geometry']]
    segments = []
    vertices = []
    for index, row in gdf.iterrows():
        coords = row['geometry'].coords
        for i, coord in enumerate(coords[:-1]):
            coords1 = (55*coords[i][1], 55*coords[i][0])
            coords2 = (55*coords[i + 1][1], 55*coords[i + 1][0])
            segment = [coords1, coords2]
            vertices.extend((coords1, coords2))
            segments.append(segment)

    gdf = osmnx.projection.project_gdf(gdf, to_crs='EPSG:3857', to_latlong=False)
    write_vertices_to_file(vertices, vertices_fn)
    write_segments_to_file(segments, segments_fn)


def create_graph_from_files(vertices_fn, segments_fn):
    graph = nx.DiGraph()
    # Add vertices to the graph
    vertices_ = load_vertices_from_file(vertices_fn)
    segments_ = load_segments_from_file(segments_fn)

    # Add nodes to the graph with their specific positions
    for vertex in vertices_:
        graph.add_node(vertex, pos=vertex)
    # Add edges to the graph
    for segment in segments_:
        graph.add_edge(segment[0], segment[1], visited=False, cw_start=None, cw_end=None, acw_start=None, acw_end=None, lane_width=lane_width)
        graph.add_edge(segment[1], segment[0], visited=False, cw_start=None, cw_end=None, acw_start=None, acw_end=None, lane_width=lane_width)
    return graph


def extract_road_network_graph(coordinates, area_name):
    vertices_fn = f"{area_name}_vertices.txt"
    segments_fn = f"{area_name}_segments.txt"
    create_verts_and_segs_files_from_coords(coordinates, vertices_fn, segments_fn)
    graph = create_graph_from_files(vertices_fn, segments_fn)
    return graph
