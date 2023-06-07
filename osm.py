import folium
from folium.plugins import Draw
import requests
import json
import pandas as pd
import os
import tkinter as tk
from tkinter import filedialog
import webbrowser
import time


def get_coordinates_from_map():
    m = folium.Map(location=[31.4756608, 74.3423600], zoom_start=16)

    draw = Draw(export=True, filename='coordinates.geojson')

    draw.add_to(m)

    m.save("map.html")

    webbrowser.open("map.html")

    root = tk.Tk()

    root.withdraw()

    file_path = filedialog.askopenfilename()
    root.destroy()

    with open(file_path, 'r') as file:
        content = file.read()
        data = json.loads(content)

    coordinates = data['features'][0]['geometry']['coordinates'][0]
    return coordinates

