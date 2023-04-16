import osmnx as ox
# import matplotlib.pyplot as plt
from matplotlib import pyplot as plt, patches
from math import sqrt, acos, atan2, sin, cos
import numpy as np
import utm
import random
import math

#%matplotlib inline

# Specify the name that is used to seach for the data
place_name = "Model Town, Lahore, Pakistan"

# Fetch OSM street network from the location
#graph = ox.graph_from_address('350 5th Ave, New York, New York', network_type='drive') # '350 5th Ave, New York, New York'
graph = ox.graph_from_place(place_name, network_type = 'drive')

# lat lon
# y x

# nodes = graph.nodes # returns node ids each node contains y,x,street count
edges = graph.edges # returns a tuple ---- u, v, key

def get_graph_bounding_box(G, margin=0.02):
    north = G.nodes[max(G.nodes, key=lambda x: G.nodes[x]['y'])]['y']
    south = G.nodes[min(G.nodes, key=lambda x: G.nodes[x]['y'])]['y']
    east = G.nodes[max(G.nodes, key=lambda x: G.nodes[x]['x'])]['x']
    west = G.nodes[min(G.nodes, key=lambda x: G.nodes[x]['x'])]['x']
    
    margin_ns = 0 # abs(north - south) * margin
    margin_ew = 0 # abs(east - west) * margin
    
    return north + margin_ns, south - margin_ns, east + margin_ew, west - margin_ew


def plot_graph2(G, bbox=None, fig_height = 6, fig_width = 6, margin=0.02,
               axis_off=True, bgcolor = 'black', show=True,
               close=False, save=False, filename='temp_plot', file_format='png',
               dpi=300, annotate=False, node_color = 'red', node_size=100,
               node_alpha = None, node_edgecolor = "none", node_zorder=1,
               edge_color = 'w', edge_linewidth=1, edge_alpha = None, use_geom=True):

    # create figure and axis to plot the graph on
    fig, ax = plt.subplots(figsize=(fig_width, fig_height))

    # set the background color
    ax.set_facecolor(bgcolor)

    # set the axis limits to tightly fit the graph
    if bbox is None:
        bbox = get_graph_bounding_box(G, margin=margin)

    margin_ns = margin * abs((bbox[1] - bbox[0]))
    margin_ew = margin * abs((bbox[3] - bbox[2]))

    
    ax.set_ylim(bbox[0] - margin_ns, bbox[1] + margin_ns)
    ax.set_xlim(bbox[2] - margin_ew, bbox[3] + margin_ew)

    # turn off the axis
    if axis_off:
        ax.axis('off')

    # plot the nodes and edges of the graph
    for u, v, data in G.edges(keys=False, data=True):
        if 'geometry' in data and use_geom:
            xs, ys = data['geometry'].xy
            ax.plot(xs, ys, color=edge_color, linewidth=edge_linewidth, alpha=edge_alpha)
        else:
            x1, y1 = G.nodes[u]['x'], G.nodes[u]['y']
            x2, y2 = G.nodes[v]['x'], G.nodes[v]['y']
            ax.plot([x1, x2], [y1, y2], color=edge_color, linewidth=edge_linewidth, alpha=edge_alpha)

    node_Xs = [float(x) for _, x in G.nodes(data='x')]
    node_Ys = [float(y) for _, y in G.nodes(data='y')]

    # plot the nodes of the graph
    ax.scatter(node_Xs, node_Ys, s=node_size, c=node_color, alpha=node_alpha,
               edgecolor=node_edgecolor, zorder=node_zorder)

    # add optional annotation
    if annotate:
        for node, data in G.nodes(data=True):
            ax.annotate(node, (data['x'], data['y']))

    # set the aspect ratio of the plot
   
    ax.set_aspect('equal')

    # display or save the plot
    if save:
        if not filename.endswith('.{}'.format(file_format)):
            filename = '{}.{}'.format(filename, file_format)
        plt.savefig(filename, dpi=dpi, bbox_inches='tight', facecolor=fig.get_facecolor())
        if close:
            plt.close()
    elif show:
        plt.show()

    # return the figure and axis objects
    return fig, ax

#fig, ax = plot_graph2(graph, edge_linewidth = 0.02, node_zorder=2)
fig, ax = ox.plot_graph(graph, edge_linewidth = 10)
    
ax.set_axis_on()