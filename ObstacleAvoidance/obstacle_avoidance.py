import matplotlib.pyplot as plt
from shapely.geometry import Point, LineString, Polygon


def find_point_index(point, polygon):
    for i, coord in enumerate(polygon.exterior.coords):
        if Point(coord).equals(point):
            return i
    return -1  # Point not found


def plot_geometry(road, car, reference_track, obstacle, buffer, modified_track):
    fig, ax = plt.subplots()

    # Plot the road
    x, y = road.exterior.xy
    ax.plot(x, y, color='black', linewidth=2)

    # Plot the car
    x, y = car.xy
    ax.plot(x, y, marker='o', markersize=10, color='red')

    # Plot the reference track
    x, y = reference_track.xy
    ax.plot(x, y, color='green', linewidth=2)

    # Plot the modified reference track
    x, y = modified_track.xy
    # ax.plot(x, y, marker='o', markersize=10, color='yellow')
    ax.plot(x, y, color='red', linewidth=2)

    # Plot the obstacle and its buffer
    x, y = obstacle.exterior.xy
    ax.plot(x, y, color='blue', linewidth=2)

    x, y = buffer.exterior.xy
    ax.plot(x, y, color='lightblue', linewidth=0.5)

    # Set axis limits and labels
    ax.set_xlim(0, 20)
    ax.set_ylim(0, 10)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_aspect('equal')

    # Show the plot
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



def modify_reference_track_for_obstacles(obstacles, reference_track, road_polygon, road_separator):
    # road_half_1, road_half_2 = get_road_halves(LineString(road_separator), road_polygon)
    #
    for obstacle in obstacles:
        obstacle_centre = obstacle.centroid
        buffer = obstacle_centre.buffer(8)
    #
    #     if Polygon(road_half_1).contains(obstacle_centre):
    #         road = road_half_1
    #         print("half 1")
    #     elif Polygon(road_half_2).contains(obstacle_centre):
    #         road = road_half_2
    #         print("half 2")
    #     else:
    #         return reference_track

        reference_track = modify_reference_track(reference_track, buffer, road_polygon)

    return reference_track

# #
# # # Example usage
# road = Polygon([(3, 2), (3, 8), (19, 8), (19, 2)])
# car = Point((5, 5))
# reference_track = LineString([(19, 3), (5, 3)])
# # obstacle = Polygon([(15, 6), (16.23, 4.53), (14.71, 3.89), (14.41, 5.59)])
# #obstacle = Polygon([(13.82, 3.7), (12, 6), (13, 7), (14.96, 4.62)])
# #obstacle = Polygon([(11, 4), (11, 6), (17, 6), (17, 4)]) # rectangle
# #obstacle = Polygon([(11, 7.8), (7, 2.2), (17, 2.2)]) # triangle out of boundary
# #obstacle = Polygon([(10.83, 6.01), (11.8, 4.4), (10.2, 4.6)]) # triangle inside road
# #obstacle1 = Polygon([(10,6), (11,6), (12.14, 5.28), (12.16, 3.84), (10.94, 3.18), (9.3, 3.96)]) # hexagon
#
# obstacle1 = Point(15, 3).buffer(1)
# obstacle2 = Point(10, 3).buffer(1)
# obstacle_centre1 = obstacle1.centroid
# buffer1 = obstacle1.buffer(1)
# buffer2 = obstacle2.buffer(1)
#
# modified_track = modify_reference_track(reference_track, buffer1, road)
# modified_track2 = modify_reference_track2(modified_track, buffer2, road)
#
# # for i, point in enumerate(buffer.exterior.coords):
# #     print(i, point)
#
# plot_geometry(road, car, reference_track, obstacle1, buffer1, modified_track2)#=LineString([(1, 2), (3, 4)]))
