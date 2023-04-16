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

# nodes = graph.nodes    # returns node ids each node contains y,x,street count
edges = graph.edges      # returns a tuple ---- u, v, key

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


def get_m_and_b(segment):
    x1, y1 = segment[0]
    x2, y2 = segment[1]

    if x2 - x1 == 0:
        m = "inf"
        b = 0
    else:
        m = (y2 - y1) / (x2 - x1)
        b = y1 - m * x1
    return m, b


# unit shift
def plot_graph31(G, ax = None, figsize = (6, 6), bgcolor = "#111111", 
    node_color = "w", node_size = 15, node_alpha = None, node_edgecolor = "none", node_zorder = 1,
    edge_color = "#999999", edge_linewidth = 1, edge_alpha = None,
    show = True, close = False, save = False, filepath = None,
    dpi = 300, bbox = None):

    if ax is None:
        #fig, ax = plt.subplots(figsize=figsize, facecolor=bgcolor)
        # create figure and axis to plot the graph on
        fig, ax = plt.subplots(figsize=figsize)
        # set the background color
        ax.set_facecolor(bgcolor)
    else:
        fig = ax.figure

    # if node size is an array, make sure it's a list
    if isinstance(node_size, np.ndarray):
        node_size = node_size.tolist()

     # if no node alpha, make all nodes completely opaque
    if node_alpha is None:
        na = [1] * len(G.nodes())
    else:
        na = node_alpha

    hwidth = 0.0071 # edge_linewidth /2
    unit = 0.02
    cnode = 99061081
    segments = []

    for u, v, data in G.edges(keys=False, data=True):
        segment = [(u), (v)]

        if (u, v) not in segments and (v, u) not in segments:
            segments.append(segment)

            x1, y1 = G.nodes[u]['x'], G.nodes[u]['y']
            x2, y2 = G.nodes[v]['x'], G.nodes[v]['y']
            #ax.plot([x1, x2], [y1, y2], color=edge_color, linewidth=edge_linewidth, alpha=edge_alpha)

            x1m , y1m, zn, zl = utm.from_latlon(float(y1), float(x1)) # lat lon
            x2m , y2m, zn, zl = utm.from_latlon(float(y2), float(x2)) # lat lon
            
            x1m = x1m/1000
            y1m = y1m/1000
            x2m = x2m/1000
            y2m = y2m/1000
            
            #print(x1m, ',', y1m, ' -- ', x2m, ',' , y2m)

            segment1 = [(x1m, y1m), (x2m, y2m)]
            m1, b1 = get_m_and_b(segment1)
        
            if m1 == 0:  # horizontal line y = c
                if x2m > x1m:
                    x1 = x1m + unit
                    x2 =  x2m - unit
                elif x2m < x1m:
                    x1 = x1m - unit
                    x2 =  x2m + unit

                p1 = [x1, y1m + unit]
                p2 = [x1, y1m - unit]
                p3 = [x2, y2m - unit]
                p4 = [x2, y2m + unit]

            elif m1 == "inf":  # vertical line
                
                if y2m > y1m:
                    y1 = y1m + unit
                    y2 = y2m - unit
                elif y2m < y1m:
                    y1 = y1m - unit
                    y2 = y2m + unit
                
                p1 = [x1m + unit, y1]
                p2 = [x1m - unit, y1]
                p3 = [x2m - unit, y2]
                p4 = [x2m + unit, y2]

            elif m1 < 0:
                if x2m < x1m and y2m > y1m:
                    p1 = [x1m, y1m + (2 *unit)]
                    p2 = [x1m - (2 *unit), y1m]
                    p3 = [x2m, y2m - (2 *unit)]
                    p4 = [x2m + (2 *unit), y2m]
                   
                elif x2m > x1m and y2m < y1m:
                    p1 = [x1m, y1m - (2 *unit)]
                    p2 = [x1m + (2 *unit), y1m]
                    p3 = [x2m, y2m + (2 *unit)]
                    p4 = [x2m - (2 *unit), y2m]

            elif m1 > 0:
                if x2m > x1m and y2m > y1m:
                    p1 = [x1m, y1m + (2 *unit)]
                    p2 = [x1m + (2 *unit), y1m]
                    p3 = [x2m, y2m - (2 *unit)]
                    p4 = [x2m - (2 *unit), y2m]

                elif x2m < x1m and y2m < y1m:
                    p1 = [x1m, y1m - (2 *unit)]
                    p2 = [x1m - (2 *unit), y1m]
                    p3 = [x2m, y2m + (2 *unit)]
                    p4 = [x2m + (2 *unit), y2m]

        coord = [p1, p2, p3, p4]
        coord.append(coord[0])
        xs, ys = zip(*coord) #create lists of x and y values

        fill_color = (random.random(), random.random(), random.random())


        ax.plot([x1m, x2m], [y1m, y2m], 'o-',  c=node_color)
        ax.plot(xs, ys,  alpha=edge_alpha) # color = edge_color,

        ax.fill(xs, ys, color = fill_color)
     
        #ax.plot([x1m, x2m], [y1m, y2m], color=edge_color, linewidth=edge_linewidth, alpha=edge_alpha)
        
        # if 'geometry' in data:
        #     xs, ys = data['geometry'].xy
        #     ax.plot(xs, ys, color=edge_color, linewidth=edge_linewidth, alpha=edge_alpha)
        # else:
        #     x1, y1 = G.nodes[u]['x'], G.nodes[u]['y']
        #     x2, y2 = G.nodes[v]['x'], G.nodes[v]['y']
        #     ax.plot([x1, x2], [y1, y2], color=edge_color, linewidth=edge_linewidth, alpha=edge_alpha)

    node_Xs = [float(x) for _, x in G.nodes(data='x')]
    node_Ys = [float(y) for _, y in G.nodes(data='y')]

    # plot the nodes of the graph
    # ax.scatter(node_Xs, node_Ys, s=node_size, c=node_color, alpha=na,
    #            edgecolor=node_edgecolor, zorder = node_zorder)
    
    ax.set_aspect('equal')


    plt.show()

    return fig, ax



fig, ax = plot_graph2(graph, edge_linewidth = 0.02, node_zorder=2)
#fig, ax = ox.plot_graph(graph, edge_linewidth = 10)
    
ax.set_axis_on()