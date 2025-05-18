import xml.etree.ElementTree as ET
from xml.dom import minidom
import csv
from tkinter import filedialog
from tkinter import messagebox


class PathPlannerDataStorage:
    def __init__(self, filename):
        # Create the root element of the XML
        self.root = ET.Element("PathPlannerDataStorage")
        self.waypoints_element = ET.SubElement(self.root, "waypoints")
        self.filename = filename
        self.tree = None
        self.index = 1

        self.waypoints_filename = None

    def add_waypoint(self, latitude, longitude):
        waypoint_element = ET.SubElement(self.waypoints_element, "waypoint-" + str(self.index))
        waypoint_latitude = ET.SubElement(waypoint_element, "latitude")
        waypoint_longitude = ET.SubElement(waypoint_element, "longitude")
        waypoint_latitude.text = str(latitude)
        waypoint_longitude.text = str(longitude)
        self.index += 1

    def add_multiple_waypoints(self, waypoints):
        for waypoint_item in waypoints:
            waypoint_element = ET.SubElement(self.waypoints_element, "waypoint-" + str(self.index))
            waypoint_latitude = ET.SubElement(waypoint_element, "latitude")
            waypoint_longitude = ET.SubElement(waypoint_element, "longitude")
            waypoint_latitude.text = str(waypoint_item[0])
            waypoint_longitude.text = str(waypoint_item[1])
            self.index += 1

    def save_xml(self):
        self.tree = ET.ElementTree(self.root)
        # Generate the XML string with the declaration
        xml_str = ET.tostring(self.root, encoding="utf-8", method="xml")
        # Use minidom to parse and pretty print the XML
        xml_str_pretty = minidom.parseString(xml_str).toprettyxml(indent="  ")

        # Write the pretty-printed XML to a file
        with open(self.filename, "w", encoding="utf-8") as f:
            f.write(xml_str_pretty)

    def read_temp_coordinate_data(self):
        coordinates = []

        if self.waypoints_filename is None:
            messagebox.showwarning("Plot Waypoints Warning",
                                   "No valid waypoints file selected, please import a valid csv file.")
            return
        elif self.waypoints_filename == '':
            messagebox.showwarning("Plot Waypoints Warning",
                                   "No valid waypoints file selected, please import a valid csv file.")
            return

        with open(self.waypoints_filename, newline='', encoding='utf-8') as csvfile:
            reader = csv.reader(csvfile)
            next(reader)  # Skip the header
            for row in reader:
                if len(row) >= 2 and row[0] and row[1]:  # Ensure latitude and longitude exist
                    try:
                        latitude = float(row[0])
                        longitude = float(row[1])
                        coordinates.append((latitude, longitude))
                    except ValueError:
                        pass  # Ignore rows with invalid numerical data
        return coordinates

    def import_waypoints(self):
        self.waypoints_filename = filedialog.askopenfilename()
