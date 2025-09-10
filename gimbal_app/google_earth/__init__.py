"""
Google Earth integration module for gimbal tracking system.

This module provides functionality to:
- Parse KML/KMZ files for waypoint extraction
- Manage multi-target waypoint tracking
- Generate real-time KML feeds for Google Earth
- Integrate with the existing gimbal tracking system
"""

from .kml_parser import KMLParser
from .waypoint_manager import WaypointManager
from .telemetry_feed import TelemetryKMLFeed
from .controller import GoogleEarthController

__all__ = [
    'KMLParser',
    'WaypointManager', 
    'TelemetryKMLFeed',
    'GoogleEarthController'
]