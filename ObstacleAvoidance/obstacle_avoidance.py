import matplotlib.pyplot as plt
from shapely.geometry import Point, LineString, Polygon, MultiLineString

obstacles = []
static_cars = []
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


def find_point_index(point, polygon):
    for i, coord in enumerate(polygon.exterior.coords):
        if Point(coord).equals(point):
            return i
    return -1  # Point not found

def circle_legend_handler(legend, orig_handle, fontsize, handlebox):
    from matplotlib.patches import Circle
    x, y = handlebox.xdescent, handlebox.ydescent
    width, height = handlebox.width, handlebox.height
    radius = min(width, height) / 2.0
    handlebox.add_artist(Circle([width/2.0, height/2.0], radius, facecolor=orig_handle.get_fc(), edgecolor='black'))
import matplotlib.patches as mpatches
def plot_geometry(road, car, reference_track, obstacle, buffer, modified_track):
    fig, ax = plt.subplots()

    # Plot the road
    x, y = road.exterior.xy
    r = ax.fill(x, y, color='#807E78', linewidth=2, edgecolor='black', label='road')

    # Plot the car
    # x, y = car.xy
    # ax.plot(x, y, marker='o', markersize=10, color='red')

    # Plot the reference track
    x, y = reference_track.xy
    ax.plot(x, y, color='darkgreen', linewidth=2, label='track')

    # Plot the modified reference track
    # x, y = modified_track.xy
    # ax.plot(x, y, color='darkgreen', linewidth=2.5, label='modified track')

    # Plot the obstacle and its buffer
    x, y = obstacle.exterior.xy
    ax.fill(x, y, color='blue', linewidth=2, label='obstacle')

    # x, y = buffer.exterior.xy
    # d = ax.plot(x, y, color='black', linewidth=0.5, label='deviation around obstacle')

    # Set axis limits and labels
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 12)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_aspect('equal')
    ax.set_xlim(2, 28)
    # Show the plot
    ax.legend(fontsize='large')
    from matplotlib.lines import Line2D

    o = Line2D([], [], color="white", marker='o', markerfacecolor="blue", markersize=15)
    road = Line2D([], [], color='#807E78', markerfacecolor='#807E78')
    line3 = Line2D([], [], color="white", marker='o', markersize=5, markerfacecolor="slategray")
    line4 = Line2D([], [], color="white", marker='o', markersize=10, markerfacecolor="slategray")
    # plt.legend((o, road, t, d), ('obstacle', 'Thing 2', 'Thing 3', 'Thing 4'), numpoints=1, loc=1)
    ax.axis('off')
    plt.show()


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


def clipped_buffer_boundary(polygon, start_point, end_point):
    boundary_coords = list(polygon.exterior.coords)
    closest_start_index = None
    closest_end_index = None

    # Find the closest points on the polygon to the start and end points
    closest_start_index = find_closest_point_index(Point(start_point), polygon)
    closest_end_index = find_closest_point_index(Point(end_point), polygon)

    closest_start_point = boundary_coords[closest_start_index]
    closest_end_point = boundary_coords[closest_end_index]

    if closest_start_index is None or closest_end_index is None:
        raise ValueError("Unable to find closest points on the polygon's boundary.")

    if closest_start_index < closest_end_index:
        boundary_coords_clipped_1 = boundary_coords[closest_start_index:closest_end_index + 1]
    else:
        boundary_coords_clipped_1 = boundary_coords[closest_start_index:] + boundary_coords[:closest_end_index + 1]

    boundary_coords_clipped_2 = list(set(boundary_coords) - set(boundary_coords_clipped_1))

    return {'clipped_boundary_1': boundary_coords_clipped_1, 'clipped_boundary_2': boundary_coords_clipped_2}


def get_polygon_halves(intersection_point1, intersection_point2, polygon): # polygon --> buffer
    exterior_coords = list(polygon.exterior.coords)

    # Find the closest points on the polygon to the start and end points
    index1 = find_closest_point_index(Point(intersection_point1), polygon) + 0
    index2 = find_closest_point_index(Point(intersection_point2), polygon) - 0

    if index1 < index2:
        polygon_half1 = exterior_coords[index1:index2+1]
        polygon_half2 = exterior_coords[index2:] + exterior_coords[:index1+1]
    else:
        polygon_half1 = exterior_coords[index2:index1+1]
        polygon_half2 = exterior_coords[index1:] + exterior_coords[:index2+1]

    return polygon_half1, polygon_half2


def estimate_polygon_radius(polygon):
    centroid = polygon.centroid
    max_distance = 0

    for vertex in polygon.exterior.coords:
        distance = Point(vertex).distance(centroid)
        if distance > max_distance:
            max_distance = distance

    return max_distance


def modify_reference_track(reference_track, buffer, road):
    intersection_points = reference_track.intersection(buffer)
    intersection_coords = list(intersection_points.coords)

    if len(intersection_coords) < 2:
        # Handle case when there are less than two intersection points
        return reference_track

    intersection_point1 = intersection_coords[0]  # start_point
    intersection_point2 = intersection_coords[1]  # end_point
    part1 = list(reference_track.coords)[:-1]

    #clipped_boundary = clipped_buffer_boundary(buffer, start_point, end_point)

    part2_1, part2_2 = get_polygon_halves(intersection_point1,  intersection_point2, buffer)
    try:
        part2 = part2_1 if road.contains(LineString(part2_1)) else part2_2
    except Exception:
        print('here')
        part2 = part2_1
    part3 = [intersection_point2, list(reference_track.coords)[-1]]


    # if swap:
    #     return LineString(part2)
    # else:
    if Point(part2[0]).distance(Point(part1[-1])) > Point(part2[-1]).distance(Point(part1[-1])):
        part2.reverse()
    return LineString(part1 + part2 + part3)

def modify_reference_track2(reference_track, buffer, road):
    intersection_points = reference_track.intersection(buffer)
    intersection_coords = list(intersection_points.coords)

    if len(intersection_coords) < 2:
        # Handle case when there are less than two intersection points
        return reference_track

    intersection_point1 = intersection_coords[0]  # start_point
    intersection_point2 = intersection_coords[1]  # end_point

    # swap = False
    # if Point(reference_track.coords[0]).distance(Point(intersection_point1)) > Point(reference_track.coords[0]).distance(Point(intersection_point2)):
    #     intersection_point1, intersection_point2 = intersection_point2, intersection_point1
    #     swap = True


    part1 = list(reference_track.coords)[:-1]

    #clipped_boundary = clipped_buffer_boundary(buffer, start_point, end_point)

    part2_1, part2_2 = get_polygon_halves(intersection_point1,  intersection_point2, buffer)

    part2 = part2_1 if road.contains(LineString(part2_1)) else part2_2

    part3 = [intersection_point2, list(reference_track.coords)[-1]]

    # if swap:
    #     return LineString(part2)
    # else:
    if Point(part2[0]).distance(Point(part1[-1])) > Point(part2[-1]).distance(Point(part1[-1])):
        part2.reverse()
    return LineString(part1 + part2 + part3)


def get_road_halves(reference_track, road_polygon):
    intersection_points = reference_track.intersection(road_polygon)
    intersection_coords = list(intersection_points.coords)

    if len(intersection_coords) < 2:
        # Handle case when there are less than two intersection points
        return reference_track

    intersection_point1 = intersection_coords[0]  # start_point
    intersection_point2 = intersection_coords[1]  # end_point

    road_half_1, road_half_2 = get_polygon_halves(intersection_point1, intersection_point2, road_polygon)

    road_half_1.append(intersection_point2)
    road_half_1.append(intersection_point1)

    road_half_2.append(intersection_point1)
    road_half_2.append(intersection_point2)

    return road_half_1,  road_half_2



def modify_reference_track_for_obstacles(obstacles, reference_track, road_polygon):
    # road_half_1, road_half_2 = get_road_halves(LineString(road_separator), road_polygon)
    #
    for obstacle in obstacles:
        obstacle_centre = obstacle.centroid
        buffer = obstacle_centre.buffer(estimate_polygon_radius(obstacle))
    #
    #     if Polygon(road_half_1).contains(obstacle_centre):
    #         road = road_half_1
    #         print("half 1")
    #     elif Polygon(road_half_2).contains(obstacle_centre):
    #         road = road_half_2
    #         print("half 2")
    #     else:
    #         return reference_track
        try:
            reference_track = modify_reference_track_dynamic(reference_track, obstacle, road_polygon)
        except Exception:
            print('error a gya')
            print(f'track: {reference_track}, \nobstacle: {obstacle}, \nroad: {road_polygon.get_face_for_point(obstacle.centroid).polygon}')
            x, y = reference_track.xy
            plt.plot(x, y)
            plt.show()
            reference_track = modify_reference_track_dynamic(reference_track, obstacle, road_polygon)
        reference_track = modify_reference_track_dynamic(reference_track, obstacle, road_polygon)

    return reference_track


def modify_reference_track3(reference_track, obstacle, road):
    centroid = obstacle.centroid
    buffer = centroid.buffer(estimate_polygon_radius(obstacle)+1)
    intersection_points = reference_track.intersection(buffer)
    if intersection_points.is_empty:
        # Handle case when there are less than two intersection points
        return reference_track
    intersection_coords = list(intersection_points.coords)

    intersection_point1 = intersection_coords[0]  # start_point
    intersection_point2 = intersection_coords[1]  # end_point

    part1 = list(reference_track.coords)[:-1]

    part2_1, part2_2 = get_polygon_halves(intersection_point1,  intersection_point2, buffer)
    try:
        part2 = part2_1 if road.contains(LineString(part2_1)) else part2_2
    except Exception:
        print('here')
        part2 = part2_1
    part3 = [intersection_point2, list(reference_track.coords)[-1]]


    # if swap:
    #     return LineString(part2)
    # else:
    if Point(part2[0]).distance(Point(part1[-1])) > Point(part2[-1]).distance(Point(part1[-1])):
        part2.reverse()
    return LineString(part1 + part2 + part3)


def find_segment_indices(line, point):
    # Ensure the input is a Point and a LineString
    if not isinstance(point, Point):
        point = Point(point)

    # Get the projection of the point onto the line
    projected_point = line.interpolate(line.project(point))

    # Find the indices of the two coordinates between which the projected point lies
    coordinates = list(line.coords)
    index1 = None
    index2 = None
    for i in range(len(coordinates) - 1):
        p1 = Point(coordinates[i])
        p2 = Point(coordinates[i + 1])
        segment = LineString([p1, p2])
        if projected_point.distance(segment) < 1e-6:  # You can adjust this threshold value as needed
            index1 = i
            index2 = i + 1
            break

    return index1


def modify_reference_track_dynamic(reference_track, obstacle, road_network):
    centroid = obstacle.centroid
    buffer = centroid.buffer(estimate_polygon_radius(obstacle)+5)
    intersection_points = reference_track.intersection(buffer)
    if intersection_points.is_empty:
        # Handle case when there are less than two intersection points
        return reference_track
    if isinstance(intersection_points, MultiLineString):
        return reference_track
    intersection_coords = list(intersection_points.coords)

    intersection_point1 = intersection_coords[0]  # start_point
    intersection_point2 = intersection_coords[1]  # end_point

    index = find_segment_indices(reference_track, intersection_point1)

    part1 = list(reference_track.coords)[:index+1]

    part2_1, part2_2 = get_polygon_halves(intersection_point1,  intersection_point2, buffer)
    part2_1_inside_roads = True
    for coordinate in part2_1:
        face = road_network.get_face_for_point(Point(coordinate[0], coordinate[1]))
        if face is None:
            part2_1_inside_roads = False
            break
    try:
        if part2_1_inside_roads:
            part2 = part2_1
        else:
            part2 = part2_2
        # part2 = part2_1 if road_polygon.contains(LineString(part2_1)) else part2_2
    except Exception:
        print('here')
        part2 = part2_1
    index = find_segment_indices(reference_track, intersection_point2)
    part3 = [intersection_point2]
    part3.extend(list(reference_track.coords)[index+1:])
    # part3 = list(reference_track.coords)[index + 2:]


    # if swap:
    #     return LineString(part2)
    # else:
    if Point(part2[0]).distance(Point(part1[-1])) > Point(part2[-1]).distance(Point(part1[-1])):
        part2.reverse()

    return LineString(part1 + part2 + part3)


# # Example usage
road = Polygon([(3, 1.5), (3, 6), (26, 6), (26, 1.5)])
car = Point((5, 5))
reference_track = LineString([(5, 3), (25, 3)])
# obstacle = Polygon([(15, 6), (16.23, 4.53), (14.71, 3.89), (14.41, 5.59)])
#obstacle = Polygon([(13.82, 3.7), (12, 6), (13, 7), (14.96, 4.62)])
#obstacle = Polygon([(11, 4), (11, 6), (17, 6), (17, 4)]) # rectangle
#obstacle = Polygon([(11, 7.8), (7, 2.2), (17, 2.2)]) # triangle out of boundary
#obstacle = Polygon([(10.83, 6.01), (11.8, 4.4), (10.2, 4.6)]) # triangle inside road
#obstacle1 = Polygon([(10,6), (11,6), (12.14, 5.28), (12.16, 3.84), (10.94, 3.18), (9.3, 3.96)]) # hexagon

obstacle1 = Point(15, 3).buffer(1)
obstacle2 = Point(10, 3).buffer(1)

segment = [(12, 3), (14, 3)]
translated_segment_cw = translate_segment(segment, 1)
translated_segment_acw = translate_segment(segment, 1, anticlockwise=True)
obstacle3 = Polygon([translated_segment_cw[0], translated_segment_cw[1], translated_segment_acw[1],
                            translated_segment_acw[0]])

obstacle_centre1 = obstacle1.centroid
buffer1 = obstacle1.buffer(1)
buffer3 = obstacle3.centroid.buffer(estimate_polygon_radius(obstacle3)+1)

# modified_track = modify_reference_track3(reference_track, obstacle3, road)
# modified_track2 = modify_reference_track2(modified_track, buffer3, road)

# for i, point in enumerate(buffer.exterior.coords):
#     print(i, point)

# plot_geometry(road, car, reference_track, obstacle3, buffer3, modified_track)#=LineString([(1, 2), (3, 4)]))
