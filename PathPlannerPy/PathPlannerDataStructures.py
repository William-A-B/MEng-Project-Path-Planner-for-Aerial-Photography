from dataclasses import dataclass
from tkintermapview import TkinterMapView


@dataclass
class Marker:
    marker: object
    latitude: float
    longitude: float
    altitude: float


@dataclass
class WindConditions:
    wind_direction: float
    wind_speed: float


@dataclass
class UAVAltitudeLimits:
    min_altitude: float
    max_altitude: float

@dataclass
class UAVProperties:
    airspeed: float
    turning_radius: float
