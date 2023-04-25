def get_y_at_x(origin, destination, x):
    """
    :param origin: tuple in the form (x, y) representing starting point of a segment
    :param destination: tuple in the form (x, y) representing ending point of a segment
    :param x: x coordinate of a point
    :return: y coordinate corresponding to x coordinate of the point on the segment
    """
    m, b = get_slope_and_y_intercept([origin, destination])
    return m * x + b


def get_slope_and_y_intercept(segment):
    """
    :param segment: a list of two tuples in the form (x, y) representing starting and ending coordinates of the segment.
    :returns: slope and y-intercept of the segment
    """
    x1, y1 = segment[0]
    x2, y2 = segment[1]

    if x2 - x1 == 0:
        # slope of vertical segment is infinity
        slope = "inf"
        y_intercept = 0
    else:
        slope = (y2 - y1) / (x2 - x1)
        y_intercept = y1 - slope * x1
    return slope, y_intercept


def get_intersection_point(segment1, segment2):
    """
    Calculates the intersection point of two line segments.

    :param segment1: A list of two tuples representing the starting and ending points of the first line segment.
    :param segment2: A list of two tuples representing the starting and ending points of the second line segment.

    :returns: A list containing coordinates of the intersection point, or [None] if the segments do not intersect.
    """

    # Calculate the slope and y-intercept of each segment
    m1, b1 = get_slope_and_y_intercept(segment1)
    m2, b2 = get_slope_and_y_intercept(segment2)

    # Check if the segments are parallel
    if m1 != "inf" and m2 != "inf" and m1 - m2 == 0:
        return [None]

    # Check if either segment is vertical
    if m1 == "inf":
        # Calculate x coordinate of intersection point for vertical segment1
        x = segment1[0][0]
        y = get_y_at_x((segment2[0][0], segment2[0][1]), (segment2[1][0], segment2[1][1]), x)
    elif m2 == "inf":
        # Calculate x coordinate of intersection point for vertical segment2
        x = segment2[0][0]
        y = get_y_at_x((segment1[0][0], segment1[0][1]), (segment1[1][0], segment1[1][1]), x)

    # Check if either segment is horizontal
    elif m1 == 0:
        # Calculate y coordinate of intersection point for horizontal segment1
        y = segment1[0][1]
        x = get_y_at_x((segment2[0][1], segment2[0][0]), (segment2[1][1], segment2[1][0]), y)
    elif m2 == 0:
        # Calculate y coordinate of intersection point for horizontal segment2
        y = segment2[0][1]
        x = get_y_at_x((segment1[0][1], segment1[0][0]), (segment1[1][1], segment1[1][0]), y)

    # Otherwise, calculate the intersection point normally
    else:
        x = (b2 - b1) / (m1 - m2)
        y = m1 * x + b1

    return [x, y]
