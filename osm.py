import folium
from folium.plugins import Draw
import requests
import json
import pandas as pd
import os
import tkinter as tk
from tkinter import filedialog

# Create the Tkinter root window
root = tk.Tk()

# Hide the root window
root.withdraw()

# Open the file dialog
file_path = filedialog.askopenfilename()

# Check if a file was selected
if file_path:
    print("Selected file:", file_path)
else:
    print("No file selected")

# Destroy the root window
root.destroy()

m = folium.Map(location=[31.4756608, 74.3423600], zoom_start=16)

draw = Draw(export=True, filename='coordinates.geojson')

draw.add_to(m)

m.save("map.html")
