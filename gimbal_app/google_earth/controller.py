"""
Google Earth integration controller.
Main controller that integrates KML parsing, waypoint management, and telemetry feeds
with the existing gimbal tracking system.
"""

import os
import time
import threading
from typing import Optional, Callable, Dict, Any, List
from dataclasses import dataclass

from .kml_parser import KMLParser, Waypoint
from .waypoint_manager import WaypointManager, TrackingMode
from .telemetry_feed import TelemetryKMLFeed, TelemetryData


@dataclass
class GoogleEarthConfig:
    """Configuration for Google Earth integration."""
    kml_output_dir: str = "kml_output"
    telemetry_update_interval: float = 1.0
    auto_generate_flight_path: bool = True
    enable_real_time_feed: bool = True


class GoogleEarthController:
    """Main controller for Google Earth integration."""
    
    def __init__(self, config: GoogleEarthConfig = None):
        self.config = config or GoogleEarthConfig()
        
        # Initialize components
        self.kml_parser = KMLParser()
        self.waypoint_manager = WaypointManager()
        self.telemetry_feed = TelemetryKMLFeed(self.config.kml_output_dir)
        
        # State
        self._is_running = False
        self._current_mission_file: Optional[str] = None
        
        # Callbacks for integration with existing gimbal system
        self._on_target_changed: Optional[Callable[[float, float, float], None]] = None
        self._telemetry_source: Optional[Callable[[], Dict[str, Any]]] = None
        
        # Flight path tracking
        self._flight_path_points: List[tuple] = []
        self._max_path_points = 1000
        
        # Setup waypoint manager callbacks
        self.waypoint_manager.set_waypoint_changed_callback(self._on_waypoint_changed)
        self.waypoint_manager.set_mission_completed_callback(self._on_mission_completed)
        
        # Setup telemetry feed data source
        self.telemetry_feed.set_data_source(self._get_telemetry_data)
    
    def load_mission_from_kml(self, kml_file_path: str) -> Dict[str, Any]:
        """
        Load mission waypoints from KML/KMZ file.
        
        Args:
            kml_file_path: Path to KML/KMZ file
            
        Returns:
            Dictionary with mission load results
        """
        try:
            # Parse KML file
            waypoints = self.kml_parser.parse_file(kml_file_path)
            
            if not waypoints:
                return {
                    'success': False,
                    'error': 'No waypoints found in KML file',
                    'waypoint_count': 0
                }
            
            # Load waypoints into manager
            self.waypoint_manager.clear_waypoints()
            self.waypoint_manager.add_waypoints(waypoints)
            
            self._current_mission_file = kml_file_path
            
            return {
                'success': True,
                'waypoint_count': len(waypoints),
                'waypoints': [
                    {
                        'name': wp.name,
                        'latitude': wp.latitude,
                        'longitude': wp.longitude,
                        'altitude': wp.altitude,
                        'description': wp.description
                    }
                    for wp in waypoints
                ]
            }
        
        except Exception as e:
            return {
                'success': False,
                'error': str(e),
                'waypoint_count': 0
            }
    
    def start_mission(self, tracking_mode: TrackingMode = TrackingMode.SEQUENTIAL) -> bool:
        """
        Start the waypoint tracking mission.
        
        Args:
            tracking_mode: How to navigate through waypoints
            
        Returns:
            True if mission started successfully
        """
        try:
            # Set tracking mode
            self.waypoint_manager.set_tracking_mode(tracking_mode)
            
            # Start waypoint mission
            if not self.waypoint_manager.start_mission():
                return False
            
            # Start telemetry feed if enabled
            if self.config.enable_real_time_feed:
                self.telemetry_feed.start_auto_update(self.config.telemetry_update_interval)
            
            self._is_running = True
            
            return True
        
        except Exception as e:
            print(f"Error starting mission: {e}")
            return False
    
    def stop_mission(self) -> None:
        """Stop the current mission."""
        self._is_running = False
        self.waypoint_manager.stop_mission()
        self.telemetry_feed.stop_auto_update()
        
        # Generate final flight path KML if enabled
        if self.config.auto_generate_flight_path and self._flight_path_points:
            self.generate_flight_path_kml()
    
    def next_waypoint(self) -> Optional[Dict[str, Any]]:
        """
        Move to the next waypoint in the mission.
        
        Returns:
            Dictionary with next waypoint info or None if mission complete
        """
        next_wp = self.waypoint_manager.next_waypoint()
        
        if next_wp:
            return {
                'name': next_wp.name,
                'latitude': next_wp.latitude,
                'longitude': next_wp.longitude,
                'altitude': next_wp.altitude,
                'description': next_wp.description
            }
        
        return None
    
    def select_waypoint(self, index: int) -> Optional[Dict[str, Any]]:
        """
        Manually select a waypoint by index.
        
        Args:
            index: Waypoint index to select
            
        Returns:
            Selected waypoint info or None if invalid
        """
        selected_wp = self.waypoint_manager.select_waypoint(index)
        
        if selected_wp:
            return {
                'name': selected_wp.name,
                'latitude': selected_wp.latitude,
                'longitude': selected_wp.longitude,
                'altitude': selected_wp.altitude,
                'description': selected_wp.description
            }
        
        return None
    
    def get_current_target_coordinates(self) -> Optional[tuple]:
        """
        Get current target coordinates for gimbal tracking.
        
        Returns:
            (latitude, longitude, altitude) tuple or None
        """
        return self.waypoint_manager.get_current_coordinates()
    
    def set_target_changed_callback(self, callback: Callable[[float, float, float], None]) -> None:
        """
        Set callback for when target coordinates change.
        This integrates with the existing gimbal tracking system.
        """
        self._on_target_changed = callback
    
    def set_telemetry_source(self, callback: Callable[[], Dict[str, Any]]) -> None:
        """
        Set callback to get telemetry data from MAVLink connection.
        
        Expected telemetry format:
        {
            'latitude': float,
            'longitude': float,
            'altitude': float,
            'heading': float,
            'speed': float,
            'battery_voltage': float,
            'flight_mode': str
        }
        """
        self._telemetry_source = callback
    
    def _on_waypoint_changed(self, waypoint: Waypoint) -> None:
        """Handle waypoint change event."""
        # Notify gimbal system of new target coordinates
        if self._on_target_changed:
            self._on_target_changed(waypoint.latitude, waypoint.longitude, waypoint.altitude)
    
    def _on_mission_completed(self) -> None:
        """Handle mission completion."""
        print("Mission completed!")
        # Could add callback for mission completion notification
    
    def _get_telemetry_data(self) -> Optional[TelemetryData]:
        """Get current telemetry data for KML generation."""
        if not self._telemetry_source:
            return None
        
        try:
            tel_data = self._telemetry_source()
            
            if not tel_data:
                return None
            
            # Convert to TelemetryData format
            telemetry = TelemetryData(
                latitude=tel_data.get('latitude', 0.0),
                longitude=tel_data.get('longitude', 0.0),
                altitude=tel_data.get('altitude', 0.0),
                heading=tel_data.get('heading', 0.0),
                speed=tel_data.get('speed', 0.0),
                timestamp=time.time(),
                battery_voltage=tel_data.get('battery_voltage', 0.0),
                flight_mode=tel_data.get('flight_mode', 'UNKNOWN'),
                gimbal_yaw=tel_data.get('gimbal_yaw', 0.0),
                gimbal_pitch=tel_data.get('gimbal_pitch', 0.0),
                gimbal_roll=tel_data.get('gimbal_roll', 0.0)
            )
            
            # Add to flight path if enabled
            if self.config.auto_generate_flight_path:
                self._add_to_flight_path(telemetry.latitude, telemetry.longitude, telemetry.altitude)
            
            return telemetry
        
        except Exception as e:
            print(f"Error getting telemetry data: {e}")
            return None
    
    def _add_to_flight_path(self, lat: float, lon: float, alt: float) -> None:
        """Add point to flight path tracking."""
        self._flight_path_points.append((lat, lon, alt))
        
        # Limit path points to avoid memory issues
        if len(self._flight_path_points) > self._max_path_points:
            self._flight_path_points = self._flight_path_points[-self._max_path_points//2:]
    
    def get_mission_status(self) -> Dict[str, Any]:
        """Get comprehensive mission status."""
        waypoint_status = self.waypoint_manager.get_mission_status()
        
        return {
            'controller_running': self._is_running,
            'mission_file': self._current_mission_file,
            'telemetry_feed_active': self.config.enable_real_time_feed,
            'kml_output_dir': self.config.kml_output_dir,
            'network_link_path': self.telemetry_feed.get_network_link_path(),
            'flight_path_points': len(self._flight_path_points),
            **waypoint_status
        }
    
    def get_waypoints_list(self) -> List[Dict[str, Any]]:
        """Get list of all waypoints with status."""
        waypoints = self.waypoint_manager.get_waypoints()
        
        return [
            {
                'index': i,
                'name': wp.waypoint.name,
                'latitude': wp.waypoint.latitude,
                'longitude': wp.waypoint.longitude,
                'altitude': wp.waypoint.altitude,
                'description': wp.waypoint.description,
                'visited': wp.visited,
                'currently_tracking': wp.currently_tracking,
                'visit_count': wp.visit_count,
                'last_visited': wp.last_visited
            }
            for i, wp in enumerate(waypoints)
        ]
    
    def generate_flight_path_kml(self, output_file: str = None) -> str:
        """
        Generate KML file with recorded flight path.
        
        Args:
            output_file: Optional output file path
            
        Returns:
            Path to generated KML file
        """
        if not self._flight_path_points:
            raise ValueError("No flight path data available")
        
        return self.telemetry_feed.generate_flight_path_kml(self._flight_path_points, output_file)
    
    def export_waypoints_to_kml(self, output_file: str) -> None:
        """Export current waypoints to a KML file."""
        waypoints = self.waypoint_manager.get_waypoints()
        
        if not waypoints:
            raise ValueError("No waypoints to export")
        
        # Use the KML parser to generate a simple waypoint KML
        # This is a simplified version - could be enhanced
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
            f.write('<kml xmlns="http://www.opengis.net/kml/2.2">\n')
            f.write('  <Document>\n')
            f.write('    <name>Exported Waypoints</name>\n')
            
            for i, wp_status in enumerate(waypoints):
                wp = wp_status.waypoint
                f.write('    <Placemark>\n')
                f.write(f'      <name>{wp.name}</name>\n')
                f.write(f'      <description>{wp.description}</description>\n')
                f.write('      <Point>\n')
                f.write(f'        <coordinates>{wp.longitude},{wp.latitude},{wp.altitude}</coordinates>\n')
                f.write('      </Point>\n')
                f.write('    </Placemark>\n')
            
            f.write('  </Document>\n')
            f.write('</kml>\n')
    
    def cleanup(self) -> None:
        """Clean up resources."""
        self.stop_mission()
        self.telemetry_feed.cleanup()