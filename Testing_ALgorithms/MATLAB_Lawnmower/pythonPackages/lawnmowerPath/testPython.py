import lawnmowerPath
import folium
from time import sleep
import numpy as np

import matlab

lmpath = lawnmowerPath.initialize()

# Reference GPS coordinate (London - Westminster)
lat_ref, lon_ref = 51.5074, -0.1278  

# Conversion factors (approximate for London)
meters_to_lat = 1 / 111000  # 1 meter in degrees latitude
meters_to_lon = 1 / (111320 * np.cos(np.radians(lat_ref)))  # 1 meter in degrees longitude

waypoints = lmpath.lawnmowerPath(600, 200, 30, 0, 0)
print(waypoints, sep='\n')

# Convert to GPS coordinates
gps_coordinates = [
    (lat_ref + y * meters_to_lat, lon_ref + x * meters_to_lon) for x, y in waypoints
]


m = folium.Map(location=gps_coordinates[0], zoom_start=15)

folium.PolyLine(
    locations=gps_coordinates,
    color="#FF0000",
    weight=2.5,
    tooltip="Lawnmower Path",
).add_to(m)

m.save("lawnmowerPath.html")


sleep(1000)
lawnmowerPath.terminate()
