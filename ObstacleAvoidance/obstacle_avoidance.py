from shapely.geometry import Point, LineString, Polygon, MultiLineString
from RUSTIC.Main.road_network import get_road_network
from RUSTIC.RoadBuildingTool.drawing_tool import get_points_on_graph
from RUSTIC.Utility import files_functions
from RUSTIC.MapToRoadMapping.graph_to_road_network import translate_segment


obstacles = []
static_cars = []


def add_obstacles_on_road_network(road_network_name, new=False):
    road_network = get_road_network('dha')
    try:
        if new:
            print('Plot points in pairs. Two points of the pair are the two end points of the obstacle.')
            obstacle_positions = get_points_on_graph(road_network)
            files_functions.write_vertices_to_file(obstacle_positions, f'{road_network_name}_obstacles_positions')
        else:
            obstacle_positions = files_functions.load_vertices_from_file(f'{road_network_name}_obstacles_positions')

        global obstacles
        i = 0
        while i <= len(obstacle_positions) - 2:
            segment = [obstacle_positions[i], obstacle_positions[i + 1]]
            translated_segment_cw = translate_segment(segment, 1)
            translated_segment_acw = translate_segment(segment, 1, anticlockwise=True)
            obstacles.append(Polygon([translated_segment_cw[0], translated_segment_cw[1], translated_segment_acw[1],
                                      translated_segment_acw[0]]))
            i += 2
    except Exception:
        print('Obstacles not initialized. Creating simulation without obstacles.')


def find_closest_point_index(point, polygon):
    closest_distance = float('inf')
    closest_index = None

    for i, coord in enumerate(polygon.exterior.coords):
        vertex = Point(coord)
        distance = point.distance(vertex)
        if distance < closest_distance:
            closest_distance = distance
            closest_index = i

    return closest_index


def get_polygon_halves(intersection_point1, intersection_point2, polygon):  # polygon --> buffer
    exterior_coordinates = list(polygon.exterior.coords)

    # Find the closest points on the polygon to the start and end points
    index1 = find_closest_point_index(Point(intersection_point1), polygon) + 0
    index2 = find_closest_point_index(Point(intersection_point2), polygon) - 0

    if index1 < index2:
        polygon_half1 = exterior_coordinates[index1:index2+1]
        polygon_half2 = exterior_coordinates[index2:] + exterior_coordinates[:index1+1]
    else:
        polygon_half1 = exterior_coordinates[index2:index1+1]
        polygon_half2 = exterior_coordinates[index1:] + exterior_coordinates[:index2+1]

    return polygon_half1, polygon_half2


def estimate_polygon_radius(polygon):
    """
    Returns radius of a circle that can contain the polygon inside it
    """
    centroid = polygon.centroid
    max_distance = 0

    for vertex in polygon.exterior.coords:
        distance = Point(vertex).distance(centroid)
        if distance > max_distance:
            max_distance = distance

    return max_distance


def modify_reference_track_for_obstacles(obstacles, reference_track, road_network):
    """
    :param obstacles: The obstacles (shapely polygons) around which the track is to be modified.
    :param reference_track: The reference track (shapely line string) to be modified
    :param road_network: The road network(dcel object) in which the track is to be modified
    """
    for obstacle in obstacles:
        reference_track = modify_reference_track(obstacle, reference_track, road_network)
    return reference_track


def find_segment_indices(line, point):
    if not isinstance(point, Point):
        point = Point(point)
    projected_point = line.interpolate(line.project(point))

    # Find the indices of the two coordinates between which the projected point lies
    coordinates = list(line.coords)
    index1 = None
    for i in range(len(coordinates) - 1):
        p1 = Point(coordinates[i])
        p2 = Point(coordinates[i + 1])
        segment = LineString([p1, p2])
        if projected_point.distance(segment) < 1e-6:  # You can adjust this threshold value as needed
            index1 = i
            break
    return index1


def modify_reference_track(obstacle, reference_track, road_network):
    centroid = obstacle.centroid
    buffer = centroid.buffer(estimate_polygon_radius(obstacle)+5)
    intersection_points = reference_track.intersection(buffer)

    # Handle case when there are less than two intersection points
    if intersection_points.is_empty:
        return reference_track

    # If intersection points is multiline string, track has already been modified around this obstacle
    if isinstance(intersection_points, MultiLineString):
        return reference_track
    intersection_coordinates = list(intersection_points.coords)

    intersection_point1 = intersection_coordinates[0]  # start_point
    intersection_point2 = intersection_coordinates[1]  # end_point

    index = find_segment_indices(reference_track, intersection_point1)

    part1 = list(reference_track.coords)[:index+1]

    # the two halves of buffer around obstacle
    part2_1, part2_2 = get_polygon_halves(intersection_point1,  intersection_point2, buffer)
    part2_1_inside_roads = True
    for coordinate in part2_1:
        face = road_network.get_face_for_point(Point(coordinate[0], coordinate[1]))
        if face is None or face.tag == 2:  # non-road
            part2_1_inside_roads = False
            break
    part2 = part2_1 if part2_1_inside_roads else part2_2

    index = find_segment_indices(reference_track, intersection_point2)
    part3 = [intersection_point2]
    part3.extend(list(reference_track.coords)[index+1:])

    if Point(part2[0]).distance(Point(part1[-1])) > Point(part2[-1]).distance(Point(part1[-1])):
        part2.reverse()

    return LineString(part1 + part2 + part3)

