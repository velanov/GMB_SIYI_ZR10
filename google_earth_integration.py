"""
Google Earth Integration Module for Gimbal GPS System
Handles KML file reading and coordinate management from Google Earth Pro
"""

import xml.etree.ElementTree as ET
import os
import time
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass
import threading
import logging

@dataclass
class Target:
    """Represents a target coordinate from Google Earth"""
    name: str
    lat: float
    lon: float
    alt: float = 0.0
    description: str = ""

class GoogleEarthIntegration:
    """Handles Google Earth Pro integration for gimbal targeting"""
    
    def __init__(self, kml_file_path: str = None):
        self.kml_file_path = kml_file_path or "gimbal_targets.kml"
        self.targets: List[Target] = []
        self.current_target_index = 0
        self.auto_refresh = False
        self.refresh_interval = 2.0  # seconds
        self.last_modified = 0
        self._stop_event = threading.Event()
        self._watch_thread = None
        
        # Callbacks for target changes
        self.on_targets_updated = None  # Callback when targets list changes
        self.on_target_changed = None   # Callback when current target changes
        
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
    def set_kml_file_path(self, file_path: str):
        """Set the path to the KML file to monitor"""
        self.kml_file_path = file_path
        self.last_modified = 0
        
    def parse_kml_file(self) -> bool:
        """Parse KML file and extract coordinates"""
        try:
            if not os.path.exists(self.kml_file_path):
                self.logger.warning(f"KML file not found: {self.kml_file_path}")
                return False
                
            tree = ET.parse(self.kml_file_path)
            root = tree.getroot()
            
            # Handle KML namespace
            ns = {'kml': 'http://www.opengis.net/kml/2.2'}
            if not root.tag.endswith('kml'):
                # Try without namespace
                ns = {}
            
            new_targets = []
            
            # Find all Placemark elements
            placemarks = root.findall('.//kml:Placemark' if ns else './/Placemark', ns)
            if not placemarks:
                placemarks = root.findall('.//Placemark')
            
            for placemark in placemarks:
                try:
                    # Get name
                    name_elem = placemark.find('kml:name' if ns else 'name', ns)
                    if name_elem is None:
                        name_elem = placemark.find('name')
                    name = name_elem.text if name_elem is not None else "Unnamed"
                    
                    # Get description
                    desc_elem = placemark.find('kml:description' if ns else 'description', ns)
                    if desc_elem is None:
                        desc_elem = placemark.find('description')
                    description = desc_elem.text if desc_elem is not None else ""
                    
                    # Find Point coordinates
                    point = placemark.find('.//kml:Point' if ns else './/Point', ns)
                    if point is None:
                        point = placemark.find('.//Point')
                        
                    if point is not None:
                        coords_elem = point.find('kml:coordinates' if ns else 'coordinates', ns)
                        if coords_elem is None:
                            coords_elem = point.find('coordinates')
                            
                        if coords_elem is not None:
                            coords_text = coords_elem.text.strip()
                            # Coordinates format: longitude,latitude,altitude
                            coords = coords_text.split(',')
                            if len(coords) >= 2:
                                lon = float(coords[0])
                                lat = float(coords[1])
                                alt = float(coords[2]) if len(coords) > 2 else 0.0
                                
                                target = Target(
                                    name=name,
                                    lat=lat,
                                    lon=lon,
                                    alt=alt,
                                    description=description
                                )
                                new_targets.append(target)
                                
                except (ValueError, AttributeError) as e:
                    self.logger.warning(f"Failed to parse placemark: {e}")
                    continue
            
            # Update targets list
            old_count = len(self.targets)
            self.targets = new_targets
            
            # Reset current index if needed
            if self.current_target_index >= len(self.targets):
                self.current_target_index = 0
            
            self.logger.info(f"Loaded {len(self.targets)} targets from KML file")
            
            # Notify callbacks
            if self.on_targets_updated and len(self.targets) != old_count:
                self.on_targets_updated(self.targets)
            
            return True
            
        except Exception as e:
            self.logger.error(f"Error parsing KML file: {e}")
            return False
    
    def get_targets(self) -> List[Target]:
        """Get all targets"""
        return self.targets.copy()
    
    def get_current_target(self) -> Optional[Target]:
        """Get the current target"""
        if 0 <= self.current_target_index < len(self.targets):
            return self.targets[self.current_target_index]
        return None
    
    def get_current_coordinates(self) -> Optional[Tuple[float, float, float]]:
        """Get current target coordinates as (lat, lon, alt)"""
        target = self.get_current_target()
        if target:
            return (target.lat, target.lon, target.alt)
        return None
    
    def next_target(self) -> Optional[Target]:
        """Move to next target in the list"""
        if not self.targets:
            return None
            
        self.current_target_index = (self.current_target_index + 1) % len(self.targets)
        target = self.get_current_target()
        
        if self.on_target_changed and target:
            self.on_target_changed(target, self.current_target_index)
            
        return target
    
    def previous_target(self) -> Optional[Target]:
        """Move to previous target in the list"""
        if not self.targets:
            return None
            
        self.current_target_index = (self.current_target_index - 1) % len(self.targets)
        target = self.get_current_target()
        
        if self.on_target_changed and target:
            self.on_target_changed(target, self.current_target_index)
            
        return target
    
    def set_current_target_index(self, index: int) -> Optional[Target]:
        """Set current target by index"""
        if not self.targets or not (0 <= index < len(self.targets)):
            return None
            
        self.current_target_index = index
        target = self.get_current_target()
        
        if self.on_target_changed and target:
            self.on_target_changed(target, self.current_target_index)
            
        return target
    
    def get_target_info(self) -> Dict:
        """Get information about current target and list"""
        target = self.get_current_target()
        return {
            'current_target': target,
            'current_index': self.current_target_index,
            'total_targets': len(self.targets),
            'has_targets': len(self.targets) > 0
        }
    
    def start_auto_refresh(self, interval: float = 2.0):
        """Start automatically monitoring the KML file for changes"""
        self.refresh_interval = interval
        self.auto_refresh = True
        self._stop_event.clear()
        
        if self._watch_thread is None or not self._watch_thread.is_alive():
            self._watch_thread = threading.Thread(target=self._watch_kml_file, daemon=True)
            self._watch_thread.start()
            self.logger.info(f"Started auto-refresh for KML file: {self.kml_file_path}")
    
    def stop_auto_refresh(self):
        """Stop automatically monitoring the KML file"""
        self.auto_refresh = False
        self._stop_event.set()
        if self._watch_thread and self._watch_thread.is_alive():
            self._watch_thread.join(timeout=5.0)
        self.logger.info("Stopped auto-refresh")
    
    def _watch_kml_file(self):
        """Watch KML file for changes in a separate thread"""
        while self.auto_refresh and not self._stop_event.is_set():
            try:
                if os.path.exists(self.kml_file_path):
                    current_modified = os.path.getmtime(self.kml_file_path)
                    if current_modified > self.last_modified:
                        self.last_modified = current_modified
                        self.logger.info("KML file changed, reloading targets...")
                        self.parse_kml_file()
                        
            except Exception as e:
                self.logger.error(f"Error watching KML file: {e}")
            
            # Wait for the refresh interval or stop event
            self._stop_event.wait(self.refresh_interval)
    
    def create_sample_kml(self, file_path: str = None) -> str:
        """Create a sample KML file for testing"""
        if file_path is None:
            file_path = self.kml_file_path
            
        sample_kml = """<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>Gimbal Targets</name>
    <description>Sample targets for gimbal tracking</description>
    
    <Placemark>
      <name>Target 1</name>
      <description>First target location</description>
      <Point>
        <coordinates>-122.4194,37.7749,100</coordinates>
      </Point>
    </Placemark>
    
    <Placemark>
      <name>Target 2</name>
      <description>Second target location</description>
      <Point>
        <coordinates>-122.4094,37.7849,150</coordinates>
      </Point>
    </Placemark>
    
    <Placemark>
      <name>Target 3</name>
      <description>Third target location</description>
      <Point>
        <coordinates>-122.4294,37.7649,200</coordinates>
      </Point>
    </Placemark>
    
  </Document>
</kml>"""
        
        try:
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(sample_kml)
            self.logger.info(f"Created sample KML file: {file_path}")
            return file_path
        except Exception as e:
            self.logger.error(f"Error creating sample KML: {e}")
            return ""

class GimbalGoogleEarthInterface:
    """High-level interface for integrating Google Earth with gimbal system"""
    
    def __init__(self, gimbal_app):
        self.gimbal_app = gimbal_app
        self.ge_integration = GoogleEarthIntegration()
        self.active = False
        
        # Setup callbacks
        self.ge_integration.on_target_changed = self._on_target_changed
        self.ge_integration.on_targets_updated = self._on_targets_updated
    
    def setup(self, kml_file_path: str, auto_refresh: bool = True):
        """Setup Google Earth integration"""
        self.ge_integration.set_kml_file_path(kml_file_path)
        
        # Initial load
        if self.ge_integration.parse_kml_file():
            if auto_refresh:
                self.ge_integration.start_auto_refresh()
            self.active = True
            return True
        return False
    
    def shutdown(self):
        """Shutdown Google Earth integration"""
        self.active = False
        self.ge_integration.stop_auto_refresh()
    
    def next_target(self):
        """Switch to next target and update gimbal"""
        if not self.active:
            return
            
        target = self.ge_integration.next_target()
        if target:
            self._update_gimbal_target(target)
    
    def previous_target(self):
        """Switch to previous target and update gimbal"""
        if not self.active:
            return
            
        target = self.ge_integration.previous_target()
        if target:
            self._update_gimbal_target(target)
    
    def get_current_target_info(self) -> Dict:
        """Get current target information"""
        return self.ge_integration.get_target_info()
    
    def _update_gimbal_target(self, target: Target):
        """Update the gimbal application with new target coordinates"""
        try:
            # Update fixed target in the gimbal app
            self.gimbal_app.fixed_lat_var.set(str(target.lat))
            self.gimbal_app.fixed_lon_var.set(str(target.lon))
            self.gimbal_app.fixed_alt_var.set(str(target.alt))
            
            # Set to fixed target mode
            self.gimbal_app.target_mode_var.set("fixed")
            self.gimbal_app._on_target_mode_change()
            
            # Apply the target
            self.gimbal_app._set_fixed_target()
            
            logging.info(f"Updated gimbal target to: {target.name} ({target.lat}, {target.lon}, {target.alt})")
            
        except Exception as e:
            logging.error(f"Error updating gimbal target: {e}")
    
    def _on_target_changed(self, target: Target, index: int):
        """Callback when target changes"""
        logging.info(f"Target changed to: {target.name} (index: {index})")
    
    def _on_targets_updated(self, targets: List[Target]):
        """Callback when targets list is updated"""
        logging.info(f"Targets updated: {len(targets)} targets available")

# Example usage and testing
if __name__ == "__main__":
    # Create sample KML file for testing
    ge = GoogleEarthIntegration("test_targets.kml")
    ge.create_sample_kml()
    
    # Test parsing
    if ge.parse_kml_file():
        print(f"Loaded {len(ge.targets)} targets:")
        for i, target in enumerate(ge.targets):
            print(f"  {i+1}. {target.name}: {target.lat}, {target.lon}, {target.alt}")
        
        # Test navigation
        print(f"\nCurrent target: {ge.get_current_target().name}")
        print(f"Next target: {ge.next_target().name}")
        print(f"Next target: {ge.next_target().name}")
        print(f"Previous target: {ge.previous_target().name}")
    
    # Clean up test file
    if os.path.exists("test_targets.kml"):
        os.remove("test_targets.kml")