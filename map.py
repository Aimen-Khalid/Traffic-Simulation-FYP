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

area = ox.geocode_to_gdf(place_name)

nodes = graph.nodes
i = 0
for node in nodes:
    i += 1

print(i)
