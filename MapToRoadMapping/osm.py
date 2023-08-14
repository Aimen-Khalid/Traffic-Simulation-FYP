import folium
from folium.plugins import Draw, Search, Geocoder
import json
import tkinter as tk
from tkinter import filedialog
import webbrowser


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


def main():
    coordinates = select_map_region()
    print("Coordinates picked from map: ", coordinates)


# main()
