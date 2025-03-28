import xml.etree.ElementTree as ET
from xml.dom import minidom



class PathPlannerDataStorage:
    def __init__(self, filename):
        # Create the root element of the XML
        self.root = ET.Element("PathPlannerDataStorage")
        self.waypoints_element = ET.SubElement(self.root, "waypoints")
        self.filename = filename
        self.tree = None
        self.index = 1

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
