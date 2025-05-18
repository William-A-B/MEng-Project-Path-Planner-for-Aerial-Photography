import numpy as np
import matplotlib.pyplot as plt
import folium
from PIL import Image, ImageTk
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import tkintermapview
from time import sleep
from geopy.geocoders import Nominatim
import PathPlannerDataStorage as ppds
import PathPlannerDataStructures as ppstruct
import math
from geopy.distance import geodesic


from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
# Implement the default Matplotlib key bindings.
from matplotlib.backend_bases import key_press_handler
from matplotlib.figure import Figure

from Old_Scripts.PathPlannerMATLAB2D_OLD import PathPlanner2D
from PathPlannerTSPSolver_2D import setupTSP
import matlab

import srtm


class PathPlannerGUI:
    def __init__(self, root, tsp_solver_2d, tsp_solver_3d, data_handler):
        # Initialise GUI
        self.root = root
        self.root.title("Path Planner Application")
        self.root.geometry("1280x960")  # Width x Height

        self.elevation_data = srtm.get_data()

        # Store marker positions
        self.markers = []
        self.marker_positions = []
        self.lines = []
        self.polygons = []
        self.start_pos = None
        self.goal_pos = None

        self.wind_condition = ppstruct.WindConditions(0, 0)

        self.uav_altitude_limits = ppstruct.UAVAltitudeLimits(20, 100)

        self.uav_properties = ppstruct.UAVProperties(20, 30)

        # Buttons for Line/Polygon Mode
        self.mode = "none"  # Default mode is none

        # Object for calling TSP Solver
        self.my_tsp_solver_2d = tsp_solver_2d
        self.my_tsp_solver_3d = tsp_solver_3d

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

        self.flight_parameters_frame = None
        self.flight_parameters_title_label = None
        self.select_wind_direction_label = None
        self.wind_direction_menu = None
        self.altitude_limits_frame = None
        self.altitude_limits_label = None
        self.min_altitude_label = None
        self.max_altitude_label = None
        self.min_altitude_var = tk.StringVar(value=str(self.uav_altitude_limits.min_altitude))
        self.max_altitude_var = tk.StringVar(value=str(self.uav_altitude_limits.max_altitude))
        self.min_altitude_box = None
        self.max_altitude_box = None
        self.set_altitude_limits_button = None
        self.uav_properties_frame = None
        self.uav_properties_label = None
        self.uav_airspeed_label = None
        self.uav_airspeed_var = tk.StringVar(value=str(self.uav_properties.airspeed))
        self.uav_airspeed_box = None
        self.uav_turningradius_label = None
        self.uav_turningradius_var = tk.StringVar(value=str(self.uav_properties.turning_radius))
        self.uav_turningradius_box = None
        self.set_uav_properties_button = None

        self.setup_flight_parameter_settings()

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
        self.elevation_graph_frame = None
        self.elevation_profile_figure = None
        self.elevation_canvas_toolbar = None

        self.setup_elevation_profile()

        self.algorithm_modes_frame = None
        self.algorithm_title_label = None
        self.calculate_shortest_path_2d_button = None
        self.calculate_shortest_path_3d_button = None
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
        """
        Set up the main layout structure of the GUI, including left and right frames.
        """
        # Create main frame
        self.main_frame = ttk.Frame(self.root, padding=5)
        self.main_frame.pack(expand=True, fill=tk.BOTH)

        # Configure row/column weights
        self.main_frame.columnconfigure(0, weight=1)
        self.main_frame.columnconfigure(1, weight=3)
        self.main_frame.rowconfigure(0, weight=8)
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
        """
        Initialize the map widget in the top-right frame and bind click events and menu commands.
        """
        # Add a map widget inside the top-right frame
        self.map_widget = tkintermapview.TkinterMapView(self.right_top_frame, width=300, height=400, corner_radius=0)
        self.map_widget.pack(expand=True, fill=tk.BOTH, padx=5, pady=5)

        # set current widget position and zoom
        self.map_widget.set_position(53.946555, -1.029298)  # York, Campus East
        self.map_widget.set_zoom(15)

        # Right click menu options
        self.map_widget.add_right_click_menu_command(label="Add Marker", command=self.add_marker_event,
                                                     pass_coords=True)
        self.map_widget.add_right_click_menu_command(label="Set Start Position Marker", command=self.set_start_position,
                                                     pass_coords=True)
        self.map_widget.add_right_click_menu_command(label="Set Goal Position Marker", command=self.set_goal_position,
                                                     pass_coords=True)

        self.map_widget.add_left_click_map_command(self.on_map_click)

    def setup_map_location_settings(self):
        """
        Configure the location search section in the left panel, including search input and button.
        """
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
        """
        Add tile selection options for the map (e.g., OSM, Google, Satellite).
        """
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

    def setup_flight_parameter_settings(self):
        """
        Add input options for flight parameters including wind, altitude limits, and UAV properties.
        """
        self.flight_parameters_title_label = ttk.Label(self.left_frame, text="Flight Parameters",
                                                       font=("Arial", 12, "bold"))
        self.flight_parameters_title_label.pack(pady=(10, 5), anchor="w")

        # Frame to hold flight parameter settings
        self.flight_parameters_frame = ttk.Frame(self.left_frame)
        self.flight_parameters_frame.pack(pady=(0, 10), fill=tk.X)

        self.select_wind_direction_label = ttk.Label(self.flight_parameters_frame, text="Select Wind Direction:")
        self.select_wind_direction_label.pack(pady=(10, 5), anchor="w")

        wind_dir_options = [
            "N", "NNE", "NE", "ENE",
            "E", "ESE", "SE", "SSE",
            "S", "SSW", "SW", "WSW",
            "W", "WNW", "NW", "NNW"
        ]
        default_wind = tk.StringVar(value="N")
        self.wind_condition.wind_direction = 0

        self.wind_direction_menu = tk.OptionMenu(self.flight_parameters_frame, default_wind, *wind_dir_options,
                                                 command=self.on_wind_dir_change)
        self.wind_direction_menu.pack(pady=(0, 10), fill=tk.X)

        # Setup UAV altitude limits section
        self.altitude_limits_label = ttk.Label(self.flight_parameters_frame, text="UAV Altitude Limits (Metres):")
        self.altitude_limits_label.pack(pady=(5, 5), anchor="w")

        self.altitude_limits_frame = ttk.Frame(self.flight_parameters_frame)
        self.altitude_limits_frame.pack(pady=(0, 10), fill=tk.X)

        for i in range(3):
            self.altitude_limits_frame.rowconfigure(i, weight=1)
        for i in range(2):
            self.altitude_limits_frame.columnconfigure(i, weight=1)

        self.min_altitude_label = ttk.Label(self.altitude_limits_frame, text="Min Limit:")
        self.min_altitude_label.grid(row=0, column=0, sticky="ew", padx=2)

        self.max_altitude_label = ttk.Label(self.altitude_limits_frame, text="Max Limit:")
        self.max_altitude_label.grid(row=0, column=1, sticky="ew", padx=2)

        self.min_altitude_box = ttk.Entry(self.altitude_limits_frame, textvariable=self.min_altitude_var, width=15)
        self.min_altitude_box.grid(row=1, column=0, sticky="ew", padx=2)

        self.max_altitude_box = ttk.Entry(self.altitude_limits_frame, textvariable=self.max_altitude_var, width=15)
        self.max_altitude_box.grid(row=1, column=1, sticky="ew", padx=2)

        self.set_altitude_limits_button = ttk.Button(self.altitude_limits_frame, text="Set altitude limits",
                                                     command=self.set_altitude_limits)
        self.set_altitude_limits_button.grid(row=2, column=0, columnspan=2, sticky="ew", padx=2)

        # Setup UAV Properties inputs
        self.uav_properties_label = ttk.Label(self.flight_parameters_frame, text="UAV Properties:")
        self.uav_properties_label.pack(pady=(5, 5), anchor="w")

        self.uav_properties_frame = ttk.Frame(self.flight_parameters_frame)
        self.uav_properties_frame.pack(pady=(0, 10), fill=tk.X)

        for i in range(3):
            self.uav_properties_frame.rowconfigure(i, weight=1)
        for i in range(2):
            self.uav_properties_frame.columnconfigure(i, weight=1)

        self.uav_airspeed_label = ttk.Label(self.uav_properties_frame, text="UAV Airspeed (m/s):")
        self.uav_airspeed_label.grid(row=0, column=0, sticky="ew", padx=2)

        self.uav_turningradius_label = ttk.Label(self.uav_properties_frame, text="UAV Turning Radius (m):")
        self.uav_turningradius_label.grid(row=0, column=1, sticky="ew", padx=2)

        self.uav_airspeed_box = ttk.Entry(self.uav_properties_frame, textvariable=self.uav_airspeed_var, width=15)
        self.uav_airspeed_box.grid(row=1, column=0, sticky="ew", padx=2)

        self.uav_turningradius_box = ttk.Entry(self.uav_properties_frame, textvariable=self.uav_turningradius_var,
                                               width=15)
        self.uav_turningradius_box.grid(row=1, column=1, sticky="ew", padx=2)

        self.set_uav_properties_button = ttk.Button(self.uav_properties_frame, text="Set UAV properties",
                                                    command=self.set_uav_properties)
        self.set_uav_properties_button.grid(row=2, column=0, columnspan=2, sticky="ew", padx=2)

    def on_wind_dir_change(self, direction):
        """
        Changes the wind direction angle based on the selected direction by the user.

        :param direction: The direction selected by the user in the GUI
        """
        direction_map = {
            "N": 0,
            "NNE": math.pi / 8,
            "NE": math.pi / 4,
            "ENE": 3 * math.pi / 8,
            "E": math.pi / 2,
            "ESE": 5 * math.pi / 8,
            "SE": 3 * math.pi / 4,
            "SSE": 7 * math.pi / 8,
            "S": math.pi,
            "SSW": 9 * math.pi / 8,
            "SW": 5 * math.pi / 4,
            "WSW": 11 * math.pi / 8,
            "W": 3 * math.pi / 2,
            "WNW": 13 * math.pi / 8,
            "NW": 7 * math.pi / 4,
            "NNW": 15 * math.pi / 8,
        }
        wind_angle_rad = direction_map[direction]
        self.wind_condition.wind_direction = wind_angle_rad

    def set_altitude_limits(self):
        """
        Sets the min and max altitude limits for the UAV based on the user input
        """
        min_alt = self.min_altitude_var.get()
        max_alt = self.max_altitude_var.get()
        # Check for empty values
        if not min_alt or not max_alt:
            messagebox.showwarning("Altitude Limits Selection",
                                   "One or both altitude limits not set. Please enter a value.")
            return

        # Check for numeric values
        try:
            min_alt = float(min_alt)
            max_alt = float(max_alt)
        except ValueError:
            messagebox.showwarning("Altitude Limits Selection",
                                   "Altitude limits must be valid numbers.")
            return

        self.uav_altitude_limits.min_altitude = min_alt
        self.uav_altitude_limits.max_altitude = max_alt

    def set_uav_properties(self):
        """
        Sets the UAV properties including the airspeed and turning radius based on the user inputs
        """
        airspeed = self.uav_airspeed_var.get()
        turning_radius = self.uav_turningradius_var.get()
        # Check for empty values
        if not airspeed or not turning_radius:
            messagebox.showwarning("UAV Properties",
                                   "One or both UAV properties not set. Please enter a value.")
            return

        # Check for numeric values
        try:
            airspeed = float(airspeed)
            turning_radius = float(turning_radius)
        except ValueError:
            messagebox.showwarning("UAV Properties",
                                   "Properties must be valid numbers")
            return

        self.uav_properties.airspeed = airspeed
        self.uav_properties.turning_radius = turning_radius

    def setup_drawing_modes(self):
        """
        Draws the "drawing mode" options and buttons onto the GUI, setting up the event listeners
        """
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
        """
        Configures the waypoint marker coordinate list to display coordinates when polygons are plotted
        """
        self.waypoint_marker_frame = ttk.Frame(self.left_frame)
        self.waypoint_marker_frame.columnconfigure(0, weight=1)
        self.waypoint_marker_frame.pack(pady=(0, 10), fill=tk.X)

        # Bind resize event to update the message width
        self.waypoint_marker_frame.bind("<Configure>", self.update_message_box_width)

        self.waypoint_marker_messagebox = tk.Message(self.waypoint_marker_frame, textvariable=self.waypoint_message_var)
        self.waypoint_marker_messagebox.config(font=("Courier", 9))
        self.waypoint_marker_messagebox.pack(pady=(0, 10), fill=tk.X)

    def update_message_box_width(self, event):
        """
        Resizes the waypoint marker coordinate list when the GUI window is resized.
        :param event: Event when the GUI window is resized, contains the updated frame width.
        """
        self.waypoint_marker_messagebox.config(width=event.width)

    def setup_elevation_profile(self):
        """
        Configures the elevation profile frame and widget, so that the UAV elevations can be plotted onto the GUI.
        """
        self.elevation_profile_frame = ttk.Frame(self.right_bottom_frame)
        self.elevation_profile_frame.rowconfigure(0, weight=1)
        self.elevation_profile_frame.columnconfigure(0, weight=1)
        self.elevation_profile_frame.pack(pady=(0, 10), fill=tk.X)

        self.elevation_title_label = ttk.Label(self.elevation_profile_frame, text="Elevation Profile",
                                               font=("Arial", 12, "bold"))
        self.elevation_title_label.pack(pady=(5, 5), anchor="w")
        self.elevation_graph_frame = ttk.Frame(self.elevation_profile_frame)
        self.elevation_graph_frame.pack(pady=(0, 10), fill=tk.X)

        self.elevation_profile_figure = Figure(figsize=(6, 2.5), dpi=50)

        self.elevation_canvas = FigureCanvasTkAgg(self.elevation_profile_figure, master=self.elevation_graph_frame)
        self.elevation_canvas.draw()

        self.elevation_canvas_toolbar = NavigationToolbar2Tk(self.elevation_canvas, self.elevation_graph_frame)
        self.elevation_canvas_toolbar.update()

        self.elevation_canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

    def setup_algorithm_modes(self):
        """
        Set up the algorithm controls and user buttons
        """
        self.calculate_shortest_path_2d_button = None
        self.choose_waypoints_button = None
        self.plot_waypoints_button = None

        self.algorithm_modes_frame = ttk.Frame(self.right_bottom_frame)
        self.algorithm_modes_frame.rowconfigure(1, weight=1)
        self.algorithm_modes_frame.columnconfigure(0, weight=1)
        self.algorithm_modes_frame.pack(pady=(0, 10), fill=tk.X)

        self.algorithm_title_label = ttk.Label(self.algorithm_modes_frame, text="Algorithm Parameters",
                                               font=("Arial", 12, "bold"))
        self.algorithm_title_label.pack(pady=(5, 5), anchor="w")

        self.calculate_shortest_path_2d_button = ttk.Button(self.algorithm_modes_frame,
                                                            text="Calculate shortest path 2D",
                                                            command=self.calculate_uav_flight_path_2d)
        self.calculate_shortest_path_2d_button.pack(side=tk.TOP, fill=tk.X, padx=5, pady=2)

        self.calculate_shortest_path_3d_button = ttk.Button(self.algorithm_modes_frame,
                                                            text="Calculate shortest path 3D",
                                                            command=self.calculate_uav_flight_path_3d)
        self.calculate_shortest_path_3d_button.pack(side=tk.TOP, fill=tk.X, padx=5, pady=2)

        self.choose_waypoints_button = ttk.Button(self.algorithm_modes_frame, text="Import existing waypoints",
                                                  command=self.data_handler.import_waypoints)
        self.choose_waypoints_button.pack(side=tk.TOP, fill=tk.X, padx=5, pady=2)

        self.plot_waypoints_button = ttk.Button(self.algorithm_modes_frame, text="Plot waypoints",
                                                command=self.plot_waypoints)
        self.plot_waypoints_button.pack(side=tk.TOP, fill=tk.X, padx=5, pady=2)

    def on_search_enter_pressed(self, event):
        """
        Listener for the enter key press event
        Calls the function to search for the location input by the user.
        :param event: Data from enter key not used here.
        """
        self.perform_location_search()

    def perform_location_search(self):
        """
        Searches for the address/location entered into the search bar and loads this location onto the map
        """
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

    def set_start_position(self, coords):
        """
        Sets the starting position for the flight path
        :param coords: The coordinates of the starting position
        """
        elevation_at_coord = self.elevation_data.get_elevation(coords[0], coords[1])
        new_marker = self.map_widget.set_marker(coords[0], coords[1], text=f'Start Position',
                                                marker_color_circle="white", marker_color_outside="green")
        self.start_pos = ppstruct.Marker(new_marker, coords[0], coords[1], elevation_at_coord)

    def set_goal_position(self, coords):
        """
        Sets the goal position for the flight path
        :param coords: The coordinates of the goal position
        """
        elevation_at_coord = self.elevation_data.get_elevation(coords[0], coords[1])
        new_marker = self.map_widget.set_marker(coords[0], coords[1], text=f'Goal Position',
                                                marker_color_circle="white", marker_color_outside="red")
        self.goal_pos = ppstruct.Marker(new_marker, coords[0], coords[1], elevation_at_coord)

    def on_map_click(self, coords):
        """
        Left click map listener. Performs actions on the map depending on the mode the user has selected in the left menu
        :param coords: The coordinates of the location the user clicked at
        """
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
        """
        Sets the drawing mode on the map to nothing, so no accidental interactions occur
        """
        self.mode = "none"
        self.current_drawing_mode_label.config(text=f"Current Mode = {self.mode}")

    def set_line_mode(self):
        """
        Switch to line-drawing mode. Allowing lines to be drawn onto the map
        """
        self.mode = "line"
        self.current_drawing_mode_label.config(text=f"Current Mode = {self.mode}")

    def set_polygon_mode(self):
        """
        Switch to polygon-drawing mode. Allowing polygons representing a surveillance area to be drawn onto the map
        """
        self.mode = "polygon"
        self.current_drawing_mode_label.config(text=f"Current Mode = {self.mode}")

    def clear_map(self):
        """
        Clears all markers and lines/polygons from the map.
        """
        print("All Recorded Polygons:", self.polygons)
        self.map_widget.delete_all_marker()
        self.map_widget.delete_all_path()
        self.map_widget.delete_all_polygon()
        self.lines.clear()
        self.polygons.clear()
        self.markers.clear()
        self.marker_positions.clear()
        self.waypoint_message_var.set("")
        self.elevation_profile_figure.clf()
        self.elevation_canvas.draw()
        self.elevation_canvas_toolbar.update()

    def save_polygon(self):
        """
        Save the polygon coordinates into an XML file
        """
        if not self.polygons:
            messagebox.showinfo("Save Polygon Information",
                                "No polygon/area on map selected, please plot one first.")
            return
        self.data_handler.add_multiple_waypoints(self.polygons[-1])
        self.data_handler.save_xml()

    def add_marker_event(self, coords):
        """
        Adds a marker onto the map with the current elevation at that position.
        Constructs an elevation height map and displays it as an image of the surrounding area
        :param coords: The coordinates of the marker placed
        """
        print("Add marker:", coords)
        elevation_at_coord = self.elevation_data.get_elevation(coords[0], coords[1])
        new_marker = self.map_widget.set_marker(coords[0], coords[1], text=f'Elevation = {elevation_at_coord}m')
        image = self.elevation_data.get_image((1080, 1080), (coords[0] - 0.1, coords[0] + 0.1),
                                              (coords[1] - 0.1, coords[1] + 0.1), 1300)
        # the image is a standard PIL object, you can save or show it:
        image.show()

    def change_tile_layer(self, tile_type):
        """
        Switches the map layer displayed between OSM, Google Maps, Satellite view and Terrain view.
        :param tile_type: The type of map tile selected by the user
        """
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
        """
        Plots a set of waypoints uploaded to the program by the user.
        """
        waypoints = self.data_handler.read_temp_coordinate_data()

        if not waypoints:
            print("Invalid waypoints retrieved")
            return

        index = 1
        for coord in waypoints:
            self.map_widget.set_marker(coord[0], coord[1], text=f"Marker {index}")
            index += 1

        self.map_widget.set_path(waypoints)

    def calculate_uav_flight_path_2d(self):
        """
        Calculates and draws the 2D shortest path based on current start and goal positions, using the drawn polygon as
        the surveillance area. Runs the algorithm via the exported MATLAB package.
        """
        if self.start_pos is None:
            messagebox.showwarning("Start Position", "No start position set, please set it by right clicking.")
            return
        if self.goal_pos is None:
            messagebox.showwarning("Goal Position", "No goal position set, please set it by right clicking.")
            return

        start_posIn = matlab.double([self.start_pos.latitude, self.start_pos.longitude, self.start_pos.altitude],
                                    size=(1, 3))
        goal_posIn = matlab.double([self.goal_pos.latitude, self.goal_pos.longitude, self.goal_pos.altitude],
                                   size=(1, 3))

        latitudes = []
        longitudes = []
        altitudes = []

        resulting_markers = []
        resulting_markers_positions = []
        resulting_altitudes = []

        # Create transparent image
        transparent_img = Image.new("RGBA", (1, 1), (255, 255, 255, 0))
        transparent_tk = ImageTk.PhotoImage(transparent_img)

        for coord in self.polygons[-1]:
            latitudes.append(coord[0])
            longitudes.append(coord[1])
            elevation_at_coord = self.elevation_data.get_elevation(coord[0], coord[1])
            altitudes.append(elevation_at_coord)
        # Close the polygon loop
        coord = self.polygons[-1][0]
        latitudes.append(coord[0])
        longitudes.append(coord[1])
        elevation_at_coord = self.elevation_data.get_elevation(coord[0], coord[1])
        altitudes.append(elevation_at_coord)

        polygon_vertices_waypoints = latitudes + longitudes + altitudes
        polygon_verticesIn = matlab.double(polygon_vertices_waypoints, size=(len(latitudes), 3))
        wind_directionIn = matlab.double([self.wind_condition.wind_direction], size=(1, 1))
        altitude_limitsIn = {"min": matlab.double([self.uav_altitude_limits.min_altitude], size=(1, 1)),
                             "max": matlab.double([self.uav_altitude_limits.max_altitude], size=(1, 1))}
        uav_turning_radiusIn = matlab.double([30.0], size=(1, 1))
        num_divisionsIn = {"x": matlab.double([12.0], size=(1, 1)), "y": matlab.double([12.0], size=(1, 1)),
                           "z": matlab.double([3.0], size=(1, 1))}
        plot_resultsIn = matlab.logical([True], size=(1, 1))
        coordinate_pathOut, dubins_path_waypointsOut = self.my_tsp_solver_2d.setupTSP(start_posIn, goal_posIn,
                                                                                      polygon_verticesIn,
                                                                                      wind_directionIn,
                                                                                      altitude_limitsIn,
                                                                                      uav_turning_radiusIn,
                                                                                      num_divisionsIn,
                                                                                      plot_resultsIn, nargout=2)

        for i, coord in enumerate(coordinate_pathOut):
            if i == 0:
                marker = self.map_widget.set_marker(coord[0], coord[1], text=f"Start Position",
                                                    marker_color_circle="white", marker_color_outside="green")
            elif i == len(coordinate_pathOut) - 1:
                marker = self.map_widget.set_marker(coord[0], coord[1], text=f"End Position",
                                                    marker_color_circle="white", marker_color_outside="red")
            else:
                marker = self.map_widget.set_marker(coord[0], coord[1], text=f"{i}", marker_color_circle="white",
                                                    marker_color_outside="blue")
            # resulting_markers.append(marker)
            # resulting_markers_positions.append((coord[0], coord[1]))
            # elevation_at_coord = self.elevation_data.get_elevation(coord[0], coord[1])
            # resulting_altitudes.append(elevation_at_coord)

        for i, waypoint in enumerate(dubins_path_waypointsOut):
            resulting_markers_positions.append((waypoint[0], waypoint[1]))
            elevation_at_coord = self.elevation_data.get_elevation(waypoint[0], waypoint[1])
            resulting_altitudes.append(elevation_at_coord)

        # Draw path if more than one marker
        if len(resulting_markers_positions) > 1:
            self.map_widget.set_path(resulting_markers_positions)
            self.plot_elevation_profile(resulting_altitudes, [], False)

    def calculate_uav_flight_path_3d(self):
        """
        Calculates and draws the 3D shortest path based on current start and goal positions, accounting for elevation,
        using the drawn polygon as the surveillance area. Runs the algorithm via the exported MATLAB package.
        """
        if self.start_pos is None:
            messagebox.showwarning("Start Position", "No start position set, please set it by right clicking.")
            return
        if self.goal_pos is None:
            messagebox.showwarning("Goal Position", "No goal position set, please set it by right clicking.")
            return

        start_posIn = matlab.double([self.start_pos.latitude, self.start_pos.longitude, self.start_pos.altitude],
                                    size=(1, 3))
        goal_posIn = matlab.double([self.goal_pos.latitude, self.goal_pos.longitude, self.goal_pos.altitude],
                                   size=(1, 3))

        latitudes = []
        longitudes = []
        altitudes = []

        resulting_markers_positions = []
        resulting_altitudes = []
        uav_altitudes = []

        # Create transparent image
        transparent_img = Image.new("RGBA", (1, 1), (255, 255, 255, 0))
        transparent_tk = ImageTk.PhotoImage(transparent_img)

        for coord in self.polygons[-1]:
            latitudes.append(coord[0])
            longitudes.append(coord[1])
            elevation_at_coord = self.elevation_data.get_elevation(coord[0], coord[1])
            altitudes.append(elevation_at_coord)
        # Close the polygon loop
        coord = self.polygons[-1][0]
        latitudes.append(coord[0])
        longitudes.append(coord[1])
        elevation_at_coord = self.elevation_data.get_elevation(coord[0], coord[1])
        altitudes.append(elevation_at_coord)

        num_div_x, num_div_y, num_div_z = self.calculate_num_divisions(latitudes, longitudes, altitudes)

        polygon_vertices_waypoints = latitudes + longitudes + altitudes
        polygon_verticesIn = matlab.double(polygon_vertices_waypoints, size=(len(latitudes), 3))
        wind_directionIn = matlab.double([self.wind_condition.wind_direction], size=(1, 1))
        altitude_limitsIn = {"min": matlab.double([self.uav_altitude_limits.min_altitude], size=(1, 1)),
                             "max": matlab.double([self.uav_altitude_limits.max_altitude], size=(1, 1))}
        uav_turning_radiusIn = matlab.double([self.uav_properties.turning_radius], size=(1, 1))
        uav_airspeedIn = matlab.double([self.uav_properties.airspeed], size=(1, 1))
        num_divisionsIn = {"x": matlab.double([num_div_x], size=(1, 1)), "y": matlab.double([num_div_y], size=(1, 1)),
                           "z": matlab.double([num_div_z], size=(1, 1))}
        plot_resultsIn = matlab.logical([True], size=(1, 1))
        coordinate_pathOut, dubins_path_waypointsOut, total_path_costOut = self.my_tsp_solver_3d.setupTSP(
            start_posIn, goal_posIn, polygon_verticesIn, wind_directionIn, altitude_limitsIn, uav_turning_radiusIn,
            uav_airspeedIn, num_divisionsIn, plot_resultsIn, nargout=3)

        for i, coord in enumerate(coordinate_pathOut):
            if i == 0:
                marker = self.map_widget.set_marker(coord[0], coord[1], text=f"Start Position",
                                                    marker_color_circle="white", marker_color_outside="green")
            elif i == len(coordinate_pathOut) - 1:
                marker = self.map_widget.set_marker(coord[0], coord[1], text=f"End Position",
                                                    marker_color_circle="white", marker_color_outside="red")
            else:
                marker = self.map_widget.set_marker(coord[0], coord[1], text=f"{i}", marker_color_circle="white",
                                                    marker_color_outside="blue")

        for i, waypoint in enumerate(dubins_path_waypointsOut):
            resulting_markers_positions.append((waypoint[0], waypoint[1]))
            elevation_at_coord = self.elevation_data.get_elevation(waypoint[0], waypoint[1])
            resulting_altitudes.append(elevation_at_coord)
            uav_altitudes.append(waypoint[2])

        # Draw path if more than one marker
        if len(resulting_markers_positions) > 1:
            self.map_widget.set_path(resulting_markers_positions)
            self.plot_elevation_profile(resulting_altitudes, uav_altitudes, True)

        messagebox.showinfo("UAV Path Cost", f"The total UAV path cost from start to goal is {total_path_costOut}")

    def plot_elevation_profile(self, altitudes, uav_altitudes, plot_uav_altitudes):
        """
        Plots the elevation profile of a 3D flight path on the elevation graph.
        :param altitudes: The list of ground level altitudes traversed by the UAV
        :param uav_altitudes: The list of altitudes the UAV was flying at
        :param plot_uav_altitudes: Bool to indicate whether to plot UAV altitudes or just ground level altitudes only
        """
        self.elevation_profile_figure.clf()
        subplot = self.elevation_profile_figure.add_subplot(111)
        alt_x = list(range(len(altitudes)))  # X-axis: index or distance
        subplot.plot(alt_x, altitudes, label='Elevation', marker='o', linestyle='-', color='blue', markersize=4)
        if plot_uav_altitudes:
            uav_alt_x = list(range(len(uav_altitudes)))  # X-axis: index or distance
            subplot.plot(uav_alt_x, uav_altitudes, label='UAV Altitude', marker='o', linestyle='-', color='red',
                         markersize=4)
            min_uav_alts = [i + self.uav_altitude_limits.min_altitude for i in altitudes]
            max_uav_alts = [i + self.uav_altitude_limits.max_altitude for i in altitudes]

            # Plot the min and max height above ground level UAV altitude limits
            subplot.plot(alt_x, min_uav_alts, label='Min UAV Limit', linestyle='-', color='green',
                         markersize=4)
            subplot.plot(alt_x, max_uav_alts, label='Max UAV Limit', linestyle='-', color='green',
                         markersize=4)

            # Shade the region between min and max height above ground level UAV altitude limits
            subplot.fill_between(alt_x, min_uav_alts, max_uav_alts, color='green', alpha=0.2, label='Safe UAV Altitude')

        subplot.set_title("Elevation Profile")
        subplot.set_xlabel("Distance")
        subplot.set_ylabel("Altitude (m)")
        subplot.grid(True)
        subplot.legend()

        self.elevation_canvas.draw()

        self.elevation_canvas_toolbar.update()

    def calculate_num_divisions(self, latitudes, longitudes, altitudes):
        """
        Calculates the number of divsions across the surveillance area polygon which determines the number of imaging
        locations to calculate the shortest path through.
        :param latitudes: List of latitude coordinates of the polygon vertices
        :param longitudes: List of longitude coordinates of the polygon vertices
        :param altitudes: List of altitudes of the polygon vertices
        :return num_div_x, num_div_y, num_div_z: Number of divisions across the polygon vertices in the x, y, z plane
        """
        # Bounding box
        min_lat, max_lat = min(latitudes), max(latitudes)
        min_lon, max_lon = min(longitudes), max(longitudes)

        # Approximate widths
        x_plane_m = geodesic((min_lat, min_lon), (min_lat, max_lon)).meters
        y_plane_m = geodesic((min_lat, min_lon), (max_lat, min_lon)).meters
        z_plane_m = self.uav_altitude_limits.max_altitude - self.uav_altitude_limits.min_altitude

        # Choose resolution (e.g., one division per 100m)
        resolution_m = 100.0
        vertical_resolution = 10.0
        num_div_x = max(1, int(x_plane_m / resolution_m))
        num_div_y = max(1, int(y_plane_m / resolution_m))
        num_div_z = max(1, int(z_plane_m / vertical_resolution))
        # Limit number of altitude divisions between 4 and 10 to reduce complexity
        if num_div_z < 4:
            num_div_z = 4
        elif num_div_z > 10:
            num_div_z = 10

        return num_div_x, num_div_y, num_div_z
