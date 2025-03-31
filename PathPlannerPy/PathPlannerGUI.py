import numpy as np
import matplotlib.pyplot as plt
import folium
from PIL import Image, ImageTk
import tkinter as tk
from tkinter import ttk
import tkintermapview
from tkintermapview import TkinterMapView
from time import sleep
from geopy.geocoders import Nominatim
import PathPlannerDataStorage as ppds


class PathPlannerGUI:
    def __init__(self, root):
        # Initialise GUI
        self.root = root
        self.root.title("Path Planner Application")
        self.root.geometry("1080x720")  # Width x Height

        # Store marker positions
        self.markers = []
        self.marker_positions = []
        self.lines = []
        self.polygons = []

        # Buttons for Line/Polygon Mode
        self.mode = "null"  # Default mode is null

        self.main_frame = None
        self.left_frame = None
        self.right_top_frame = None
        self.right_bottom_frame = None

        self.setup_gui_frames()

        self.map_widget = None

        self.setup_map_widget()

        self.location_title_label = None
        self.location_search_var = tk.StringVar()
        self.location_search_box = None
        self.location_search_button = None

        self.setup_map_location_settings()

        self.tile_title_label = None
        self.select_tile_label = None
        self.tile_layer_var = tk.StringVar()
        self.tile_layer_combobox = None

        self.setup_tile_layer_settings()

        self.drawing_mode_title_label = None
        self.no_mode_button = None
        self.draw_line_mode_button = None
        self.draw_polygon_mode_button = None
        self.clear_map_button = None
        self.save_polygon_button = None

        self.setup_drawing_modes()

        # Data Storage
        self.data = ppds.PathPlannerDataStorage("waypoints.xml")

    def setup_gui_frames(self):
        # Create main frame
        self.main_frame = ttk.Frame(self.root, padding=5)
        self.main_frame.pack(expand=True, fill=tk.BOTH)

        # Configure row/column weights
        self.main_frame.columnconfigure(0, weight=1)
        self.main_frame.columnconfigure(1, weight=1)
        self.main_frame.rowconfigure(0, weight=1)
        self.main_frame.rowconfigure(1, weight=1)

        # Left Frame (50% Width)
        self.left_frame = ttk.Frame(self.main_frame, padding=5, relief=tk.RIDGE, borderwidth=2)
        self.left_frame.grid(row=0, column=0, rowspan=2, sticky="nsew")  # Takes up full height
        self.left_frame.grid_propagate(False)

        # Right Top Frame (50% Height)
        self.right_top_frame = ttk.Frame(self.main_frame, padding=5, relief=tk.RIDGE, borderwidth=2)
        self.right_top_frame.grid(row=0, column=1, sticky="nsew")

        # Right Bottom Frame (50% Height)
        self.right_bottom_frame = ttk.Frame(self.main_frame, padding=5, relief=tk.RIDGE, borderwidth=2)
        self.right_bottom_frame.grid(row=1, column=1, sticky="nsew")

    def setup_map_widget(self):
        # Add a map widget inside the top-right frame
        self.map_widget = tkintermapview.TkinterMapView(self.right_top_frame, width=200, height=250, corner_radius=0)
        self.map_widget.pack(expand=True, fill=tk.BOTH, padx=5, pady=5)

        # set current widget position and zoom
        self.map_widget.set_position(53.946555, -1.029298)  # York, Campus East
        self.map_widget.set_zoom(15)

        # Right click menu options
        self.map_widget.add_right_click_menu_command(label="Add Marker", command=self.add_marker_event,
                                                     pass_coords=True)

        self.map_widget.add_left_click_map_command(self.on_map_click)

    def setup_map_location_settings(self):
        # Add label for drawing mode section
        self.location_title_label = ttk.Label(self.left_frame, text="Location Search", font=("Arial", 12, "bold"))
        self.location_title_label.pack(pady=(20, 5), anchor="w")

        # Search Bar
        self.location_search_box = ttk.Entry(self.left_frame, textvariable=self.location_search_var, width=50)
        self.location_search_box.pack(pady=(10, 5), anchor="w")

        self.location_search_button = ttk.Button(self.left_frame, text="Search", command=self.perform_location_search)
        self.location_search_button.pack(pady=(10, 5), anchor="w")

    def setup_tile_layer_settings(self):
        # Add label for tile selection
        self.tile_title_label = ttk.Label(self.left_frame, text="Map Tile Options", font=("Arial", 12, "bold"))
        self.tile_title_label.pack(pady=(10, 5), anchor="w")

        # Add a tile layer selection dropdown to the left pane
        self.select_tile_label = ttk.Label(self.left_frame, text="Select Tile Layer:")
        self.select_tile_label.pack(pady=(10, 5), anchor="w")

        self.tile_layer_combobox = ttk.Combobox(
            self.left_frame,
            textvariable=self.tile_layer_var,
            values=["OSM", "Google", "Satellite", "Terrain"]
        )
        self.tile_layer_combobox.pack(fill=tk.X, padx=5)
        self.tile_layer_combobox.bind("<<ComboboxSelected>>", self.on_tile_layer_change)

    def setup_drawing_modes(self):
        # Add label for drawing mode section
        self.drawing_mode_title_label = ttk.Label(self.left_frame, text="Drawing Modes", font=("Arial", 12, "bold"))
        self.drawing_mode_title_label.pack(pady=(20, 5), anchor="w")

        self.no_mode_button = ttk.Button(self.left_frame, text="No Mode", command=self.set_mode_empty)
        self.no_mode_button.pack(side=tk.TOP, fill=tk.X, padx=5, pady=2)

        self.draw_line_mode_button = ttk.Button(self.left_frame, text="Draw Line", command=self.set_line_mode)
        self.draw_line_mode_button.pack(side=tk.TOP, fill=tk.X, padx=5, pady=2)

        self.draw_polygon_mode_button = ttk.Button(self.left_frame, text="Draw Polygon", command=self.set_polygon_mode)
        self.draw_polygon_mode_button.pack(side=tk.TOP, fill=tk.X, padx=5, pady=2)

        self.clear_map_button = ttk.Button(self.left_frame, text="Clear", command=self.clear_map)
        self.clear_map_button.pack(side=tk.TOP, fill=tk.X, padx=5, pady=2)

        self.save_polygon_button = ttk.Button(self.left_frame, text="Save Polygon", command=self.save_polygon)
        self.save_polygon_button.pack(side=tk.TOP, fill=tk.X, padx=5, pady=2)

    def perform_location_search(self):
        location_name = self.location_search_var.get()
        print(f"Searching for: {location_name}")

        geolocator = Nominatim(user_agent="Autonomous_UAV_Path_Planner_Application")
        location = geolocator.geocode(location_name)

        self.map_widget.set_position(location.latitude, location.longitude)
        self.map_widget.set_zoom(15)

    def on_map_click(self, coords):
        if self.mode == "line":
            marker = self.map_widget.set_marker(coords[0], coords[1], text=f"Marker {len(self.markers) + 1}")
            self.markers.append(marker)
            self.marker_positions.append((coords[0], coords[1]))

            # Draw path if more than one marker
            if len(self.marker_positions) > 1:
                self.lines.append(self.map_widget.set_path(self.marker_positions))
        elif self.mode == "polygon":
            marker = self.map_widget.set_marker(coords[0], coords[1], text=f"Marker {len(self.markers) + 1}")
            self.markers.append(marker)
            self.marker_positions.append((coords[0], coords[1]))
            if len(self.markers) > 2:
                polygon = self.map_widget.set_polygon(self.marker_positions, fill_color="red", outline_color="black")
                self.lines.append(polygon)
                self.polygons.append(list(self.marker_positions))
                print(f"Polygon Recorded: {self.marker_positions}")
        elif self.mode == "null":
            return

    def set_mode_empty(self):
        self.mode = "null"

    def set_line_mode(self):
        """Switch to line-drawing mode."""
        self.mode = "line"

    def set_polygon_mode(self):
        """Switch to polygon-drawing mode."""
        self.mode = "polygon"

    def clear_map(self):
        """Clears all markers and lines/polygons from the map."""
        print("All Recorded Polygons:", self.polygons)
        self.map_widget.delete_all_marker()
        self.map_widget.delete_all_path()
        self.map_widget.delete_all_polygon()
        for i, line in enumerate(self.lines):
            line.delete()
        self.marker_positions.clear()

    def save_polygon(self):
        self.data.add_multiple_waypoints(self.polygons[-1])
        self.data.save_xml()

    def add_marker_event(self, coords):
        print("Add marker:", coords)
        new_marker = self.map_widget.set_marker(coords[0], coords[1], text="new marker")

    def on_tile_layer_change(self, event):
        selected_tile = self.tile_layer_var.get()
        self.change_tile_layer(selected_tile)

    def change_tile_layer(self, tile_type):
        if tile_type == "OSM":
            self.map_widget.set_tile_server("https://a.tile.openstreetmap.org/{z}/{x}/{y}.png")
        elif tile_type == "Google":
            self.map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=m&hl=en&x={x}&y={y}&z={z}&s=Ga",
                                            max_zoom=22)
        elif tile_type == "Satellite":
            self.map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Ga",
                                            max_zoom=22)
        elif tile_type == "Terrain":
            self.map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=p&hl=en&x={x}&y={y}&z={z}&s=Ga",
                                            max_zoom=22)
        print(f"Tile layer switched to: {tile_type}")
