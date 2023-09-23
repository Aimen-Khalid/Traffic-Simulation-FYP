import folium
from folium.plugins import Draw, Search, Geocoder
import json
from shapely import Polygon
import tkinter as tk
from tkinter import filedialog
import webbrowser
import osmnx
from RUSTIC.settings import scaling_factor


def select_map_region():
    m = folium.Map(location=[31.5633366, 74.3296802], zoom_start=16)
    Geocoder().add_to(m)
    draw = Draw(export=True, filename='coordinates.geojson')
    draw.add_to(m)
    m.save("map.html")
    # webbrowser.open("map.html")
    root = tk.Tk()
    root.withdraw()
    file_path = filedialog.askopenfilename()
    root.destroy()

    with open(file_path, 'r') as file:
        content = file.read()
        data = json.loads(content)

    coordinates = data['features'][0]['geometry']['coordinates'][0]
    return coordinates


def extract_roads_from_osm(coordinates):
    tags = {
        "highway": ["primary", "secondary", "tertiary", "residential", "living_street", "service", "roads"]
    }
    geom = Polygon(coordinates)
    gdf = osmnx.geometries_from_polygon(geom, tags=tags)
    if len(gdf) == 0:
        raise ValueError("Selected region has no roads")
    # Converting coordinates from lat/long to meters
    gdf = osmnx.projection.project_gdf(gdf, to_crs='EPSG:3857', to_latlong=False)
    gdf = gdf[['geometry']]
    segments = []
    vertices = []
    for index, row in gdf.iterrows():
        coordinates = row['geometry'].coords
        for i, coord in enumerate(coordinates[:-1]):
            coordinate_1 = (scaling_factor*coordinates[i][0], scaling_factor*coordinates[i][1])
            coordinate_2 = (scaling_factor*coordinates[i + 1][0], scaling_factor*coordinates[i + 1][1])
            segment = [coordinate_1, coordinate_2]
            vertices.extend((coordinate_1, coordinate_2))
            segments.append(segment)

    return vertices, segments


def main():
    coordinates = select_map_region()
    print("Coordinates picked from map: ", coordinates)


# main()
