"""
KML/KMZ parser for extracting waypoints from Google Earth files.
"""

import zipfile
import xml.etree.ElementTree as ET
from typing import List, Tuple, Optional
import os
from dataclasses import dataclass


@dataclass
class Waypoint:
    """Represents a waypoint with coordinates and metadata."""
    name: str
    latitude: float
    longitude: float
    altitude: float = 0.0
    description: str = ""


class KMLParser:
    """Parser for KML and KMZ files to extract waypoint coordinates."""
    
    def __init__(self):
        self.namespaces = {
            'kml': 'http://www.opengis.net/kml/2.2',
            'gx': 'http://www.google.com/kml/ext/2.2'
        }
    
    def parse_file(self, file_path: str) -> List[Waypoint]:
        """
        Parse a KML or KMZ file and extract waypoints.
        
        Args:
            file_path: Path to the KML or KMZ file
            
        Returns:
            List of Waypoint objects
        """
        if not os.path.exists(file_path):
            raise FileNotFoundError(f"File not found: {file_path}")
        
        if file_path.lower().endswith('.kmz'):
            return self._parse_kmz(file_path)
        elif file_path.lower().endswith('.kml'):
            return self._parse_kml(file_path)
        else:
            raise ValueError("File must be KML or KMZ format")
    
    def _parse_kmz(self, kmz_path: str) -> List[Waypoint]:
        """Parse KMZ (zipped KML) file."""
        waypoints = []
        
        with zipfile.ZipFile(kmz_path, 'r') as kmz:
            # Look for KML files in the archive
            kml_files = [f for f in kmz.namelist() if f.lower().endswith('.kml')]
            
            if not kml_files:
                raise ValueError("No KML files found in KMZ archive")
            
            # Parse all KML files in the archive
            for kml_file in kml_files:
                with kmz.open(kml_file) as kml_content:
                    waypoints.extend(self._parse_kml_content(kml_content.read()))
        
        return waypoints
    
    def _parse_kml(self, kml_path: str) -> List[Waypoint]:
        """Parse KML file."""
        with open(kml_path, 'r', encoding='utf-8') as file:
            content = file.read()
        
        return self._parse_kml_content(content)
    
    def _parse_kml_content(self, kml_content: bytes) -> List[Waypoint]:
        """Parse KML content and extract waypoints."""
        waypoints = []
        
        try:
            root = ET.fromstring(kml_content)
            
            # Find all Placemark elements
            placemarks = root.findall('.//kml:Placemark', self.namespaces)
            if not placemarks:
                # Try without namespace
                placemarks = root.findall('.//Placemark')
            
            for placemark in placemarks:
                waypoint = self._extract_waypoint_from_placemark(placemark)
                if waypoint:
                    waypoints.append(waypoint)
        
        except ET.ParseError as e:
            raise ValueError(f"Invalid KML format: {e}")
        
        return waypoints
    
    def _extract_waypoint_from_placemark(self, placemark: ET.Element) -> Optional[Waypoint]:
        """Extract waypoint data from a Placemark element."""
        # Get name
        name_elem = placemark.find('.//kml:name', self.namespaces)
        if name_elem is None:
            name_elem = placemark.find('.//name')
        name = name_elem.text if name_elem is not None else "Unnamed"
        
        # Get description
        desc_elem = placemark.find('.//kml:description', self.namespaces)
        if desc_elem is None:
            desc_elem = placemark.find('.//description')
        description = desc_elem.text if desc_elem is not None else ""
        
        # Find Point element
        point_elem = placemark.find('.//kml:Point', self.namespaces)
        if point_elem is None:
            point_elem = placemark.find('.//Point')
        
        if point_elem is None:
            # Try to find coordinates in other geometry types
            coords_elem = placemark.find('.//kml:coordinates', self.namespaces)
            if coords_elem is None:
                coords_elem = placemark.find('.//coordinates')
        else:
            coords_elem = point_elem.find('.//kml:coordinates', self.namespaces)
            if coords_elem is None:
                coords_elem = point_elem.find('.//coordinates')
        
        if coords_elem is None or not coords_elem.text:
            return None
        
        # Parse coordinates (format: longitude,latitude,altitude)
        coords_text = coords_elem.text.strip()
        coord_parts = coords_text.split(',')
        
        if len(coord_parts) < 2:
            return None
        
        try:
            longitude = float(coord_parts[0])
            latitude = float(coord_parts[1])
            altitude = float(coord_parts[2]) if len(coord_parts) > 2 else 0.0
            
            return Waypoint(
                name=name,
                latitude=latitude,
                longitude=longitude,
                altitude=altitude,
                description=description
            )
        
        except ValueError:
            return None
    
    def parse_coordinates_only(self, file_path: str) -> List[Tuple[float, float, float]]:
        """
        Parse file and return only coordinates as tuples.
        
        Args:
            file_path: Path to KML or KMZ file
            
        Returns:
            List of (latitude, longitude, altitude) tuples
        """
        waypoints = self.parse_file(file_path)
        return [(wp.latitude, wp.longitude, wp.altitude) for wp in waypoints]