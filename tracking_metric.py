from shapely.geometry import LineString, Point
from math import sqrt


def calculate_tracking_accuracy(vehicle_coordinates, linestring):
    total_distance = 0.0

    # Convert the linestring to a Shapely LineString object
    track = LineString(linestring)

    # Iterate over the vehicle coordinates
    for i in range(len(vehicle_coordinates)-1):
        # Create Shapely Point objects for the current and next vehicle coordinates
        p1 = Point(vehicle_coordinates[i])
        p2 = Point(vehicle_coordinates[i+1])

        # Calculate the closest point on the track to the current vehicle coordinate
        closest_point = track.interpolate(track.project(p1))

        # Calculate the distance between the closest point and the next vehicle coordinate
        distance = closest_point.distance(p2)

        # Add the distance to the total
        total_distance += distance

    return total_distance


vehicle_coordinates = [(0, 0), (1, 1), (2, 2), (3, 3)]
linestring = [(0, 0), (1, 1), (2, 2), (3, 3)]

accuracy = calculate_tracking_accuracy(vehicle_coordinates, linestring)
print("Accuracy:", accuracy)
