import os
import sys
import matplotlib.pyplot as plt

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, os.pardir))
sys.path.append(parent_dir)

from Utility import files_functions


def get_vertices_and_segments():
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
    ax.xaxis.set_ticks(range(0, 200, box_size))
    ax.yaxis.set_ticks(range(0, 200, box_size))

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
            return vertices, segments
        # update the first variable and previous point coordinates
        first = prev_x == x and prev_y == y
        prev_x = x
        prev_y = y


def draw_and_save_road_network_graph(vertices_file_name, segments_file_name):
    vertices, segments = get_vertices_and_segments()
    files_functions.write_vertices_to_file(vertices, vertices_file_name)
    files_functions.write_segments_to_file(segments, segments_file_name)


def main():
    graph_name = "graph"
    vertices_fn = f"{graph_name}_vertices.txt"
    segments_fn = f"{graph_name}_segments.txt"
    draw_and_save_road_network_graph(vertices_fn, segments_fn)
    print(
        f"Graph's vertices and segments have been written to {segments_fn}and {vertices_fn}respectively."
    )


# main()
