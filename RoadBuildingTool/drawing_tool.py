import matplotlib.pyplot as plt


def draw_custom_road_network():
    """
        Allows the user to interactively plot a graph by clicking on a grid of points.

        To draw the graph in continuity, simply click on each vertex you want to add.

        To start a new, disconnected component of the graph, click on the last vertex you
        created. This will allow you to draw a new vertex without having a line segment
        between this vertex and the previous vertex.

        To finish creating the graph, click twice on the last vertex you plotted.

        vertices - a list of tuples representing the coordinates of each point clicked.

        segments - a list of lists, where each inner list contains two tuples representing the start and end points of
        a line segment in the graph.

        :returns: vertices and segments
    """

    # create a figure and axes object
    fig, ax = plt.subplots(figsize=(100, 100))
    # set the aspect ratio of the plot to be equal
    ax.set_aspect('equal')

    box_size = 5
    # set the axis limits and ticks
    ax.axis([0, 50, 0, 50])
    # ax.get_xaxis().set_visible(False)
    # ax.get_yaxis().set_visible(False)

    ax.xaxis.set_ticks(range(0, 200, box_size))
    ax.yaxis.set_ticks(range(0, 200, box_size))
    ax.tick_params(axis='both', colors='white')

    # add a grid to the plot
    ax.grid(color='green', linestyle='-', linewidth=0.5)

    # initialize variables to keep track of vertices and line segments
    prev_x = 0
    prev_y = 0
    vertices = []
    segments = []
    first = True
    grid_spacing = box_size

    # continue drawing vertices and line segments until the function is interrupted
    while True:
        # get a single point from user input
        point = fig.ginput(n=1, show_clicks=True, mouse_add=1)
        # if there is no point, continue waiting for input
        if len(point) == 0:
            continue
        # round the point to the nearest grid point
        x = round(point[0][0] / grid_spacing) * grid_spacing
        y = round(point[0][1] / grid_spacing) * grid_spacing
        # add the point to the list of vertices if it hasn't already been added
        if (x, y) not in vertices:
            vertices.append((x, y))
        # if this isn't the first point, plot a line segment between this point and the previous point
        if not first:
            plt.plot([x, prev_x], [y, prev_y], 'o-')
            segment = [(prev_x, prev_y), (x, y)]
            # if the previous point is different from this point, add the segment to the list of segments
            if prev_x != x or prev_y != y:
                segments.append(segment)
        else:
            # if this is the first point, just plot it
            plt.plot(x, y, 'o-')
        # draw the plot
        plt.draw()
        # if this is the first point of a connected component, and it is the same as the previous point, we're done
        # drawing
        if first and prev_x == x and prev_y == y:
            plt.close(fig)
            return vertices, segments
        # update the first variable and previous point coordinates
        first = prev_x == x and prev_y == y
        prev_x = x
        prev_y = y


def get_points_on_graph(road_network):
    obstacles_positions = []
    # create a figure and axes object
    fig, ax = plt.subplots(figsize=(100, 100))
    # set the aspect ratio of the plot to be equal
    ax.set_aspect('equal')
    ax.axis([0, 50, 0, 50])

    road_network.extract_roads_from_map_region(ax, fig)
    prev_x = 0
    prev_y = 0
    while True:
        # get a single point from user input
        point = fig.ginput(n=1, show_clicks=True, mouse_add=1)
        # if there is no point, continue waiting for input
        if len(point) == 0:
            continue
        x = point[0][0]
        y = point[0][1]
        ax.scatter(x, y, s=0.6)
        plt.draw()
        # add the point to the list of vertices if it hasn't already been added
        if (x, y) not in obstacles_positions:
            obstacles_positions.append((x, y))
        if prev_x == x and prev_y == y:
            plt.close(fig)
            return obstacles_positions
        # update previous point coordinates
        prev_x = x
        prev_y = y


