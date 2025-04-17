import numpy as np
import matplotlib.pyplot as plt
import folium
from PIL import Image, ImageTk
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import tkintermapview
from tkintermapview import TkinterMapView
from time import sleep
from geopy.geocoders import Nominatim
import PathPlannerDataStorage as ppds

from PathPlannerMATLAB2D import PathPlanner2D
from TSPSolver import TSPSolver
import matlab

import srtm


class PathPlannerGUI:
    def __init__(self, root, tsp_solver, data_handler):
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
        self.mode = "none"  # Default mode is none

        # Object for calling TSP Solver
        self.my_tsp_solver = tsp_solver

        # Data Storage
        self.data_handler = data_handler

        self.waypoint_data_file = None

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
        self.tile_button_frame = None
        # self.tile_layer_var = tk.StringVar()
        # self.tile_layer_combobox = None

        self.setup_tile_layer_settings()

        self.drawing_mode_frame = None
        self.drawing_mode_title_label = None
        self.current_drawing_mode_label = None
        self.no_mode_button = None
        self.draw_line_mode_button = None
        self.draw_polygon_mode_button = None
        self.clear_map_button = None
        self.save_polygon_button = None

        self.setup_drawing_modes()

        self.waypoint_marker_frame = None
        self.waypoint_marker_messagebox = None
        self.waypoint_message_var = tk.StringVar()
        self.setup_waypoint_marker_coordinate_output()

        self.elevation_profile_frame = None
        self.elevation_title_label = None
        self.setup_elevation_profile()

        self.algorithm_modes_frame = None
        self.algorithm_title_label = None
        self.calculate_shortest_path_button = None
        self.choose_waypoints_button = None
        self.plot_waypoints_button = None

        self.setup_algorithm_modes()

        # Zoom mapping based on type
        self.map_zoom_levels = {
            "hamlet": 15,
            "village": 14,
            "town": 13,
            "city": 12,
            "suburb": 14,
            "neighbourhood": 15,
            "county": 9,
            "state": 7,
            "country": 5,
            "administrative": 13
        }

    def setup_gui_frames(self):
        # Create main frame
        self.main_frame = ttk.Frame(self.root, padding=5)
        self.main_frame.pack(expand=True, fill=tk.BOTH)

        # Configure row/column weights
        self.main_frame.columnconfigure(0, weight=1)
        self.main_frame.columnconfigure(1, weight=3)
        self.main_frame.rowconfigure(0, weight=5)
        self.main_frame.rowconfigure(1, weight=2)

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
        self.right_bottom_frame.rowconfigure(0, weight=3)
        self.right_bottom_frame.rowconfigure(1, weight=2)

    def setup_map_widget(self):
        # Add a map widget inside the top-right frame
        self.map_widget = tkintermapview.TkinterMapView(self.right_top_frame, width=300, height=300, corner_radius=0)
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
        self.location_title_label.pack(pady=(5, 5), anchor="w")

        # Search Bar
        self.location_search_box = ttk.Entry(self.left_frame, textvariable=self.location_search_var, width=50)
        self.location_search_box.pack(pady=(10, 5), anchor="w")
        self.location_search_box.bind("<Return>", self.on_search_enter_pressed)

        self.location_search_button = ttk.Button(self.left_frame, text="Search", command=self.perform_location_search)
        self.location_search_button.pack(pady=(10, 5), anchor="w")

    def setup_tile_layer_settings(self):
        # Add label for tile selection
        self.tile_title_label = ttk.Label(self.left_frame, text="Map Tile Options", font=("Arial", 12, "bold"))
        self.tile_title_label.pack(pady=(10, 5), anchor="w")

        # Add a tile layer selection dropdown to the left pane
        self.select_tile_label = ttk.Label(self.left_frame, text="Select Tile Layer:")
        self.select_tile_label.pack(pady=(10, 5), anchor="w")

        # Frame to hold buttons side by side
        self.tile_button_frame = ttk.Frame(self.left_frame)
        self.tile_button_frame.pack(pady=(0, 10), fill=tk.X)

        for i in range(4):
            self.tile_button_frame.columnconfigure(i, weight=1)

        # Create tile layer buttons
        tiles = ["OSM", "Google", "Satellite", "Terrain"]
        for i, tile in enumerate(tiles):
            btn = ttk.Button(self.tile_button_frame, text=tile, command=lambda t=tile: self.change_tile_layer(t))
            btn.grid(row=0, column=i, sticky="ew", padx=2)

    def setup_drawing_modes(self):
        # Add label for drawing mode section
        self.drawing_mode_title_label = ttk.Label(self.left_frame, text="Drawing Modes", font=("Arial", 12, "bold"))
        self.drawing_mode_title_label.pack(pady=(20, 5), anchor="w")

        self.current_drawing_mode_label = ttk.Label(self.left_frame, text=f"Current Mode = {self.mode}",
                                                    font=("Arial", 11))
        self.current_drawing_mode_label.pack(pady=(5, 5), anchor="w")

        # Setup frame for mode buttons
        self.drawing_mode_frame = ttk.Frame(self.left_frame)
        self.drawing_mode_frame.pack(pady=(0, 10), fill=tk.X)

        for i in range(3):
            self.drawing_mode_frame.columnconfigure(i, weight=1)

        self.no_mode_button = ttk.Button(self.drawing_mode_frame, text="No Mode", command=self.set_mode_empty)
        self.no_mode_button.grid(row=0, column=0, sticky="ew", padx=2)

        self.draw_line_mode_button = ttk.Button(self.drawing_mode_frame, text="Draw Line", command=self.set_line_mode)
        self.draw_line_mode_button.grid(row=0, column=1, sticky="ew", padx=2)

        self.draw_polygon_mode_button = ttk.Button(self.drawing_mode_frame, text="Draw Polygon",
                                                   command=self.set_polygon_mode)
        self.draw_polygon_mode_button.grid(row=0, column=2, sticky="ew", padx=2)

        self.clear_map_button = ttk.Button(self.left_frame, text="Clear", command=self.clear_map)
        self.clear_map_button.pack(side=tk.TOP, fill=tk.X, padx=5, pady=2)

        self.save_polygon_button = ttk.Button(self.left_frame, text="Save Polygon", command=self.save_polygon)
        self.save_polygon_button.pack(side=tk.TOP, fill=tk.X, padx=5, pady=2)

    def setup_waypoint_marker_coordinate_output(self):
        self.waypoint_marker_frame = ttk.Frame(self.left_frame)
        self.waypoint_marker_frame.columnconfigure(0, weight=1)
        self.waypoint_marker_frame.pack(pady=(0, 10), fill=tk.X)

        # Bind resize event to update the message width
        self.waypoint_marker_frame.bind("<Configure>", self.update_message_box_width)

        self.waypoint_marker_messagebox = tk.Message(self.waypoint_marker_frame, textvariable=self.waypoint_message_var)
        self.waypoint_marker_messagebox.config(font=("Courier", 9))
        self.waypoint_marker_messagebox.pack(pady=(0, 10), fill=tk.X)

    def update_message_box_width(self, event):
        self.waypoint_marker_messagebox.config(width=event.width)

    def setup_elevation_profile(self):
        self.elevation_profile_frame = ttk.Frame(self.right_bottom_frame)
        self.elevation_profile_frame.rowconfigure(0, weight=1)
        self.elevation_profile_frame.columnconfigure(0, weight=1)
        self.elevation_profile_frame.pack(pady=(0, 10), fill=tk.X)

        self.elevation_title_label = ttk.Label(self.elevation_profile_frame, text="Elevation Profile",
                                               font=("Arial", 12, "bold"))
        self.elevation_title_label.pack(pady=(5, 5), anchor="w")

    def setup_algorithm_modes(self):
        self.calculate_shortest_path_button = None
        self.choose_waypoints_button = None
        self.plot_waypoints_button = None

        self.algorithm_modes_frame = ttk.Frame(self.right_bottom_frame)
        self.algorithm_modes_frame.rowconfigure(1, weight=1)
        self.algorithm_modes_frame.columnconfigure(0, weight=1)
        self.algorithm_modes_frame.pack(pady=(0, 10), fill=tk.X)

        self.algorithm_title_label = ttk.Label(self.algorithm_modes_frame, text="Algorithm Parameters",
                                               font=("Arial", 12, "bold"))
        self.algorithm_title_label.pack(pady=(5, 5), anchor="w")

        self.calculate_shortest_path_button = ttk.Button(self.algorithm_modes_frame, text="Calculate shortest path",
                                                         command=self.calculate_uav_flight_path)
        self.calculate_shortest_path_button.pack(side=tk.TOP, fill=tk.X, padx=5, pady=2)

        self.choose_waypoints_button = ttk.Button(self.algorithm_modes_frame, text="Import existing waypoints",
                                                  command=self.data_handler.import_waypoints)
        self.choose_waypoints_button.pack(side=tk.TOP, fill=tk.X, padx=5, pady=2)

        self.plot_waypoints_button = ttk.Button(self.algorithm_modes_frame, text="Plot waypoints",
                                                command=self.plot_waypoints)
        self.plot_waypoints_button.pack(side=tk.TOP, fill=tk.X, padx=5, pady=2)

    def on_search_enter_pressed(self, event):
        self.perform_location_search()

    def perform_location_search(self):
        location_name = self.location_search_var.get()
        if not location_name:
            messagebox.showwarning("Location Selection", "No location specified, please enter your location in the "
                                                         "search bar above.")
            return
        print(f"Searching for: {location_name}")

        geolocator = Nominatim(user_agent="Autonomous_UAV_Path_Planner_Application")
        location = geolocator.geocode(location_name, country_codes="gb", addressdetails=True)

        if not location:
            messagebox.showerror("Location Selection",
                                 "Invalid location specified, please try again or try the location's postcode.")
            return

        location_type = location.raw.get("type", "")
        map_zoom = self.map_zoom_levels.get(location_type, 13)  # default zoom
        print(f"Located: {location.address} ({location_type}) -> zoom {map_zoom}")

        self.map_widget.set_position(location.latitude, location.longitude)
        self.map_widget.set_zoom(map_zoom)

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
        elif self.mode == "none":
            return

        # Update coordinate list in message box
        coordinate_messagebox_text = ""
        for i, coord in enumerate(self.marker_positions):
            coordinate_messagebox_text = coordinate_messagebox_text + f"{i}: {coord}\n"

        self.waypoint_message_var.set(coordinate_messagebox_text)

    def set_mode_empty(self):
        self.mode = "none"
        self.current_drawing_mode_label.config(text=f"Current Mode = {self.mode}")

    def set_line_mode(self):
        """Switch to line-drawing mode."""
        self.mode = "line"
        self.current_drawing_mode_label.config(text=f"Current Mode = {self.mode}")

    def set_polygon_mode(self):
        """Switch to polygon-drawing mode."""
        self.mode = "polygon"
        self.current_drawing_mode_label.config(text=f"Current Mode = {self.mode}")

    def clear_map(self):
        """Clears all markers and lines/polygons from the map."""
        print("All Recorded Polygons:", self.polygons)
        self.map_widget.delete_all_marker()
        self.map_widget.delete_all_path()
        self.map_widget.delete_all_polygon()
        self.lines.clear()
        self.polygons.clear()
        self.markers.clear()
        self.marker_positions.clear()
        self.waypoint_message_var.set("")

    def save_polygon(self):
        if not self.polygons:
            messagebox.showinfo("Save Polygon Information",
                                "No polygon/area on map selected, please plot one first.")
            return
        self.data_handler.add_multiple_waypoints(self.polygons[-1])
        self.data_handler.save_xml()

    def add_marker_event(self, coords):
        print("Add marker:", coords)
        elevation_data = srtm.get_data()
        elevation_at_coord = elevation_data.get_elevation(coords[0], coords[1])
        new_marker = self.map_widget.set_marker(coords[0], coords[1], text=f'Elevation = {elevation_at_coord}m')
        image = elevation_data.get_image((1080, 1080), (coords[0]-0.1, coords[0]+0.1), (coords[1]-0.1, coords[1]+0.1), 1300)
        # the image is a standard PIL object, you can save or show it:
        image.show()

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

    def plot_waypoints(self):
        waypoints = self.data_handler.read_temp_coordinate_data()

        if not waypoints:
            print("Invalid waypoints retrieved")
            return

        index = 1
        for coord in waypoints:
            self.map_widget.set_marker(coord[0], coord[1], text=f"Marker {index}")
            index += 1

        self.map_widget.set_path(waypoints)

    def calculate_uav_flight_path(self):
        if not self.polygons:
            messagebox.showinfo("Shortest Path Area Selection",
                                "No polygon/area on map selected, please plot one first.")
            return

        latitudes = []
        longitudes = []
        altitudes = []

        resulting_markers = []
        resulting_markers_positions = []

        elevation_data = srtm.get_data()

        for coord in self.polygons[-1]:
            latitudes.append(coord[0])
            longitudes.append(coord[1])
            elevation_at_coord = elevation_data.get_elevation(coord[0], coord[1])
            altitudes.append(elevation_at_coord)

        polygon_vertices_waypoints = latitudes + longitudes + altitudes
        polygon_verticesIn = matlab.double(polygon_vertices_waypoints, size=(len(latitudes), 3))

        # polygon_verticesIn = matlab.double(
        #     [53.950974807525206, 53.946024591620926, 53.945898302917456, 53.95089904334211, 53.950974807525206,
        #      -1.0329672619323844, -1.033224753997814, -1.0243412777404899, -1.0251566692810172, -1.0329672619323844,
        #      50.0, 50.0, 50.0, 50.0, 50.0], size=(5, 3))
        coordinate_pathOut, dubins_path_waypointsOut = self.my_tsp_solver.setupTSP(polygon_verticesIn, nargout=2)
        # print(coordinate_pathOut, dubins_path_waypointsOut, sep='\n')

        for i, coord in enumerate(coordinate_pathOut):
            if i == 0:
                marker = self.map_widget.set_marker(coord[0], coord[1], text=f"Start Position", marker_color_circle="white", marker_color_outside="green")
            elif i == len(coordinate_pathOut) - 1:
                marker = self.map_widget.set_marker(coord[0], coord[1], text=f"End Position", marker_color_circle="white", marker_color_outside="red")
            else:
                marker = self.map_widget.set_marker(coord[0], coord[1], text=f"Marker {i}", marker_color_circle="white", marker_color_outside="blue")
            resulting_markers.append(marker)
            resulting_markers_positions.append((coord[0], coord[1]))
        # for i, waypoint in enumerate(dubins_path_waypointsOut):
        #     if i == 0:
        #         marker = self.map_widget.set_marker(waypoint[0], waypoint[1], text=f"Start Position", marker_color_circle="white", marker_color_outside="green")
        #     elif i == len(dubins_path_waypointsOut) - 1:
        #         marker = self.map_widget.set_marker(waypoint[0], waypoint[1], text=f"End Position", marker_color_circle="white", marker_color_outside="red")
        #     else:
        #         marker = self.map_widget.set_marker(waypoint[0], waypoint[1], text=f"Marker {i}", marker_color_circle="white", marker_color_outside="blue")
        #     resulting_markers.append(marker)
        #     resulting_markers_positions.append((waypoint[0], waypoint[1]))

        # Draw path if more than one marker
        if len(resulting_markers_positions) > 1:
            self.map_widget.set_path(resulting_markers_positions)



    def test_PathPlanner2D(self):
        if not self.polygons:
            messagebox.showinfo("Shortest Path Area Selection",
                                "No polygon/area on map selected, please plot one first.")
            return

        my_PathPlanner2D = PathPlanner2D.initialize()
        latitudes = []
        longitudes = []

        for coord in self.polygons[-1]:
            latitudes.append(coord[0])
            longitudes.append(coord[1])

        polygon_vertices_waypoints = latitudes + longitudes

        # polygon_verticesIn = matlab.double(
        #     [53.950974807525206, 53.946024591620926, 53.945898302917456, 53.95089904334211, 53.950974807525206,
        #      -1.0329672619323844, -1.033224753997814, -1.0243412777404899, -1.0251566692810172, -1.0329672619323844],
        #     size=(5, 2))
        polygon_verticesIn = matlab.double(polygon_vertices_waypoints, size=(len(latitudes), 2))

        waypoint_pathOut, square_cornersOut = my_PathPlanner2D.PathPlanner2D(polygon_verticesIn, nargout=2)
        print(waypoint_pathOut, square_cornersOut, sep='\n')

        sleep(60)

        my_PathPlanner2D.terminate()
