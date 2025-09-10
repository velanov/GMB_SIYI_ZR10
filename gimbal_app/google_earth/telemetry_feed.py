"""
Real-time KML telemetry feed for Google Earth integration.
Generates dynamic KML files that update with UAV position and status.
"""

import os
import time
import threading
from typing import Optional, Callable
from dataclasses import dataclass
from xml.etree.ElementTree import Element, SubElement, tostring
from xml.dom import minidom


@dataclass
class TelemetryData:
    """Telemetry data structure for KML generation."""
    latitude: float
    longitude: float
    altitude: float
    heading: float = 0.0
    speed: float = 0.0
    timestamp: float = 0.0
    battery_voltage: float = 0.0
    flight_mode: str = "UNKNOWN"
    # Gimbal orientation data for camera view cone
    gimbal_yaw: float = 0.0
    gimbal_pitch: float = 0.0
    gimbal_roll: float = 0.0


class TelemetryKMLFeed:
    """Generates real-time KML feeds for Google Earth visualization."""
    
    def __init__(self, output_dir: str = "kml_output"):
        self.output_dir = output_dir
        # Delay directory creation to avoid blocking during initialization
        # self._ensure_output_directory()
        
        # Current telemetry data
        self._current_data: Optional[TelemetryData] = None
        self._lock = threading.Lock()
        
        # Auto-update settings
        self._auto_update = False
        self._update_interval = 0.5  # seconds - faster updates for smoother following
        self._update_thread: Optional[threading.Thread] = None
        
        # KML file paths (will be set when directory is created)
        self.uav_kml_path = None
        self.network_link_path = None
        
        # Callbacks
        self._data_source: Optional[Callable[[], TelemetryData]] = None
    
    def _ensure_output_directory(self):
        """Ensure the output directory exists."""
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir, exist_ok=True)
        
        # Set file paths once directory is created
        if self.uav_kml_path is None:
            self.uav_kml_path = os.path.join(self.output_dir, "uav_position.kml")
            self.network_link_path = os.path.join(self.output_dir, "live_feed.kml")
    
    def set_data_source(self, callback: Callable[[], TelemetryData]) -> None:
        """Set callback function to get telemetry data."""
        self._data_source = callback
    
    def update_telemetry(self, data: TelemetryData) -> None:
        """Update telemetry data manually."""
        with self._lock:
            self._current_data = data
        
        if not self._auto_update:
            self._generate_uav_kml()
    
    def start_auto_update(self, interval: float = 1.0) -> None:
        """
        Start automatic KML generation.
        
        Args:
            interval: Update interval in seconds
        """
        if self._auto_update:
            return
        
        # Ensure directory exists before starting
        self._ensure_output_directory()
        
        self._auto_update = True
        self._update_interval = interval
        self._update_thread = threading.Thread(target=self._auto_update_loop, daemon=True)
        self._update_thread.start()
        
        # Generate initial network link KML
        self._generate_network_link_kml()
    
    def stop_auto_update(self) -> None:
        """Stop automatic KML generation."""
        self._auto_update = False
        if self._update_thread:
            self._update_thread.join(timeout=2.0)
    
    def _auto_update_loop(self) -> None:
        """Auto-update loop for continuous KML generation."""
        while self._auto_update:
            try:
                # Get data from source if available
                if self._data_source:
                    with self._lock:
                        self._current_data = self._data_source()
                
                # Generate KML if we have data
                if self._current_data:
                    self._generate_uav_kml()
                
                time.sleep(self._update_interval)
            
            except Exception as e:
                print(f"Error in telemetry update loop: {e}")
                time.sleep(self._update_interval)
    
    def _generate_network_link_kml(self) -> None:
        """Generate the main KML file with NetworkLink for live updates."""
        kml = Element('kml', xmlns="http://www.opengis.net/kml/2.2")
        document = SubElement(kml, 'Document')
        
        # Document info
        name = SubElement(document, 'name')
        name.text = "UAV Live Telemetry Feed"
        
        description = SubElement(document, 'description')
        description.text = "Real-time UAV position and telemetry data"
        
        # NetworkLink for live updates
        network_link = SubElement(document, 'NetworkLink')
        
        link_name = SubElement(network_link, 'name')
        link_name.text = "UAV Position"
        
        # Refresh settings for automatic updates
        link = SubElement(network_link, 'Link')
        href = SubElement(link, 'href')
        href.text = os.path.abspath(self.uav_kml_path)
        
        refresh_mode = SubElement(link, 'refreshMode')
        refresh_mode.text = 'onInterval'
        
        refresh_interval = SubElement(link, 'refreshInterval')
        refresh_interval.text = str(max(1, int(self._update_interval)))  # Minimum 1 second
        
        # Force view refresh on updates
        view_refresh_mode = SubElement(link, 'viewRefreshMode')
        view_refresh_mode.text = 'onStop'
        
        view_refresh_time = SubElement(link, 'viewRefreshTime')
        view_refresh_time.text = '1'
        
        # Write to file
        self._write_kml_file(kml, self.network_link_path)
    
    def _generate_uav_kml(self) -> None:
        """Generate KML file with current UAV position and status."""
        if not self._current_data:
            return
        
        data = self._current_data
        
        kml = Element('kml', xmlns="http://www.opengis.net/kml/2.2")
        
        # Add NetworkLinkControl for camera control
        if data.gimbal_yaw is not None and data.gimbal_pitch is not None:
            self._add_network_link_control(kml, data)
        
        document = SubElement(kml, 'Document')
        
        # Document info
        name = SubElement(document, 'name')
        name.text = "UAV Current Position"
        
        # Style for UAV icon
        style = SubElement(document, 'Style', id='uav_style')
        icon_style = SubElement(style, 'IconStyle')
        
        scale = SubElement(icon_style, 'scale')
        scale.text = '1.2'
        
        heading = SubElement(icon_style, 'heading')
        heading.text = str(data.heading)
        
        icon = SubElement(icon_style, 'Icon')
        href = SubElement(icon, 'href')
        # Use a simple airplane icon or custom UAV icon
        href.text = 'http://maps.google.com/mapfiles/kml/shapes/airports.png'
        
        # Label style
        label_style = SubElement(style, 'LabelStyle')
        label_scale = SubElement(label_style, 'scale')
        label_scale.text = '0.8'
        
        # UAV placemark
        placemark = SubElement(document, 'Placemark')
        
        pm_name = SubElement(placemark, 'name')
        pm_name.text = f"UAV - {data.flight_mode}"
        
        # Description with telemetry data
        pm_description = SubElement(placemark, 'description')
        description_text = f"""
        <![CDATA[
        <table border="1" cellpadding="5">
        <tr><td>Latitude:</td><td>{data.latitude:.6f}°</td></tr>
        <tr><td>Longitude:</td><td>{data.longitude:.6f}°</td></tr>
        <tr><td>Altitude:</td><td>{data.altitude:.1f} m</td></tr>
        <tr><td>Heading:</td><td>{data.heading:.1f}°</td></tr>
        <tr><td>Speed:</td><td>{data.speed:.1f} m/s</td></tr>
        <tr><td>Flight Mode:</td><td>{data.flight_mode}</td></tr>
        <tr><td>Battery:</td><td>{data.battery_voltage:.1f} V</td></tr>
        <tr><td>Updated:</td><td>{time.strftime('%H:%M:%S', time.localtime(data.timestamp))}</td></tr>
        </table>
        ]]>
        """
        pm_description.text = description_text
        
        # Style reference
        style_url = SubElement(placemark, 'styleUrl')
        style_url.text = '#uav_style'
        
        # Point coordinates
        point = SubElement(placemark, 'Point')
        coordinates = SubElement(point, 'coordinates')
        coordinates.text = f"{data.longitude},{data.latitude},{data.altitude}"
        
        # Altitude mode
        altitude_mode = SubElement(point, 'altitudeMode')
        altitude_mode.text = 'absolute'
        
        # Add camera view cone if gimbal data available
        if data.gimbal_yaw is not None and data.gimbal_pitch is not None:
            self._add_camera_view_cone(document, data)
            # Add FPV camera view
            self._add_fpv_camera_view(document, data)
        
        # Write to file
        self._write_kml_file(kml, self.uav_kml_path)
    
    def _add_network_link_control(self, kml: Element, data: TelemetryData) -> None:
        """Add NetworkLinkControl to force Google Earth camera to follow airplane."""
        # Calculate absolute gimbal direction
        absolute_gimbal_heading = (data.heading + data.gimbal_yaw) % 360.0
        
        # NetworkLinkControl forces camera updates
        network_control = SubElement(kml, 'NetworkLinkControl')
        
        # Add Update element to force camera movement
        update = SubElement(network_control, 'Update')
        
        # Camera that Google Earth will automatically switch to
        camera = SubElement(network_control, 'Camera')
        
        # Camera position (at the airplane)
        longitude = SubElement(camera, 'longitude')
        longitude.text = str(data.longitude)
        
        latitude = SubElement(camera, 'latitude')
        latitude.text = str(data.latitude)
        
        altitude = SubElement(camera, 'altitude')
        altitude.text = str(data.altitude)  # Exactly at airplane altitude for true cockpit view
        
        # Camera orientation (gimbal direction)  
        heading = SubElement(camera, 'heading')
        heading.text = str(absolute_gimbal_heading)
        
        tilt = SubElement(camera, 'tilt')
        # Convert gimbal pitch to Google Earth tilt for aerial view
        # Gimbal pitch: -90 (looking down) to +90 (looking up)
        # Google Earth tilt: 0 (looking straight down) to 90 (looking at horizon)
        # For aerial view, we want: gimbal -90° -> GE tilt 0°, gimbal 0° -> GE tilt 90°
        ge_tilt = 90 + data.gimbal_pitch  # This gives us the aerial perspective
        ge_tilt = max(0, min(90, ge_tilt))  # Clamp to 0-90 for downward looking view
        tilt.text = str(ge_tilt)
        
        roll = SubElement(camera, 'roll') 
        roll.text = str(data.gimbal_roll)
        
        altitude_mode = SubElement(camera, 'altitudeMode')
        altitude_mode.text = 'absolute'
        
        # Range/distance from camera position
        range_elem = SubElement(camera, 'range')
        range_elem.text = '50'  # Very close for true first-person aerial view
        
        # Add smooth animation
        fly_to = SubElement(network_control, 'flyTo')
        duration = SubElement(fly_to, 'duration')
        duration.text = '0.1'  # Very fast transition for responsive following
        
        fly_mode = SubElement(fly_to, 'flyToMode')
        fly_mode.text = 'smooth'
        
        # Copy camera to flyTo for animation
        fly_camera = SubElement(fly_to, 'Camera')
        for child in camera:
            new_elem = SubElement(fly_camera, child.tag)
            new_elem.text = child.text
            new_elem.attrib = child.attrib
    
    def _add_camera_view_cone(self, document: Element, data: TelemetryData) -> None:
        """Add camera view cone/frustum to show what the gimbal camera is seeing."""
        import math
        
        # Camera/gimbal parameters
        fov_horizontal = 60.0  # Field of view in degrees (adjust for your camera)
        fov_vertical = 45.0
        view_distance = 1000.0  # How far to project the view cone in meters
        
        # Convert gimbal angles to absolute directions
        # Aircraft heading + gimbal yaw gives absolute gimbal direction
        absolute_gimbal_yaw = (data.heading + data.gimbal_yaw) % 360.0
        gimbal_pitch = data.gimbal_pitch
        
        # Calculate view cone corners
        uav_lat = math.radians(data.latitude)
        uav_lon = math.radians(data.longitude)
        uav_alt = data.altitude
        
        # Earth radius in meters
        R = 6378137.0
        
        def calculate_point_at_distance(bearing_deg, pitch_deg, distance):
            """Calculate lat/lon/alt of a point at given bearing, pitch, and distance from UAV."""
            bearing_rad = math.radians(bearing_deg)
            pitch_rad = math.radians(pitch_deg)
            
            # Horizontal distance accounting for pitch
            horizontal_dist = distance * math.cos(pitch_rad)
            
            # Altitude change
            alt_change = distance * math.sin(pitch_rad)
            
            # Calculate new lat/lon using great circle formulas
            lat2 = math.asin(math.sin(uav_lat) * math.cos(horizontal_dist/R) + 
                           math.cos(uav_lat) * math.sin(horizontal_dist/R) * math.cos(bearing_rad))
            
            lon2 = uav_lon + math.atan2(math.sin(bearing_rad) * math.sin(horizontal_dist/R) * math.cos(uav_lat),
                                      math.cos(horizontal_dist/R) - math.sin(uav_lat) * math.sin(lat2))
            
            return math.degrees(lat2), math.degrees(lon2), uav_alt + alt_change
        
        # Calculate the four corners of the view frustum
        corners = []
        for yaw_offset in [-fov_horizontal/2, fov_horizontal/2, fov_horizontal/2, -fov_horizontal/2]:
            for pitch_offset in [-fov_vertical/2, -fov_vertical/2, fov_vertical/2, fov_vertical/2]:
                bearing = absolute_gimbal_yaw + yaw_offset
                pitch = gimbal_pitch + pitch_offset
                lat, lon, alt = calculate_point_at_distance(bearing, pitch, view_distance)
                corners.append((lat, lon, max(0, alt)))  # Ensure altitude >= 0
                break  # Only one pitch per yaw for a rectangular frustum
        
        # Actually, let's create a simpler triangular view cone
        corners = []
        # Center of view cone
        center_lat, center_lon, center_alt = calculate_point_at_distance(absolute_gimbal_yaw, gimbal_pitch, view_distance)
        
        # Left and right edges
        left_lat, left_lon, left_alt = calculate_point_at_distance(absolute_gimbal_yaw - fov_horizontal/2, gimbal_pitch, view_distance)
        right_lat, right_lon, right_alt = calculate_point_at_distance(absolute_gimbal_yaw + fov_horizontal/2, gimbal_pitch, view_distance)
        
        # Style for view cone
        style = SubElement(document, 'Style', id='camera_view_style')
        poly_style = SubElement(style, 'PolyStyle')
        color = SubElement(poly_style, 'color')
        color.text = '4400FF00'  # Semi-transparent green
        fill = SubElement(poly_style, 'fill')
        fill.text = '1'
        outline = SubElement(poly_style, 'outline')
        outline.text = '1'
        
        line_style = SubElement(style, 'LineStyle')
        line_color = SubElement(line_style, 'color')
        line_color.text = 'FF00FF00'  # Solid green line
        width = SubElement(line_style, 'width')
        width.text = '2'
        
        # Camera view cone placemark
        placemark = SubElement(document, 'Placemark')
        
        pm_name = SubElement(placemark, 'name')
        pm_name.text = "Camera View"
        
        pm_description = SubElement(placemark, 'description')
        pm_description.text = f"Gimbal viewing direction\nYaw: {data.gimbal_yaw:.1f}°\nPitch: {data.gimbal_pitch:.1f}°"
        
        style_url = SubElement(placemark, 'styleUrl')
        style_url.text = '#camera_view_style'
        
        # Create polygon for view cone
        polygon = SubElement(placemark, 'Polygon')
        
        extrude = SubElement(polygon, 'extrude')
        extrude.text = '0'
        
        altitude_mode = SubElement(polygon, 'altitudeMode')
        altitude_mode.text = 'absolute'
        
        outer_boundary = SubElement(polygon, 'outerBoundaryIs')
        linear_ring = SubElement(outer_boundary, 'LinearRing')
        coordinates = SubElement(linear_ring, 'coordinates')
        
        # Create triangular view cone: UAV -> left -> center -> right -> back to UAV
        coord_text = f"{data.longitude},{data.latitude},{uav_alt} "
        coord_text += f"{left_lon},{left_lat},{max(0, left_alt)} "
        coord_text += f"{center_lon},{center_lat},{max(0, center_alt)} "
        coord_text += f"{right_lon},{right_lat},{max(0, right_alt)} "
        coord_text += f"{data.longitude},{data.latitude},{uav_alt}"
        
        coordinates.text = coord_text
    
    def _add_fpv_camera_view(self, document: Element, data: TelemetryData) -> None:
        """Add First Person View camera that positions Google Earth viewpoint at airplane looking through gimbal."""
        # Calculate absolute gimbal direction
        absolute_gimbal_heading = (data.heading + data.gimbal_yaw) % 360.0
        
        # Camera positioned at airplane location, looking in gimbal direction
        camera = SubElement(document, 'Camera')
        
        # Camera position (at the airplane)
        longitude = SubElement(camera, 'longitude')
        longitude.text = str(data.longitude)
        
        latitude = SubElement(camera, 'latitude')
        latitude.text = str(data.latitude)
        
        altitude = SubElement(camera, 'altitude')
        altitude.text = str(data.altitude + 2)  # Slightly above airplane for better view
        
        # Camera orientation (gimbal direction)
        heading = SubElement(camera, 'heading')
        heading.text = str(absolute_gimbal_heading)
        
        tilt = SubElement(camera, 'tilt')
        # Convert gimbal pitch to Google Earth tilt
        # Gimbal pitch: -90 (down) to +90 (up)
        # Google Earth tilt: 0 (looking down) to 90 (looking at horizon) to 180 (looking up)
        ge_tilt = 90 + data.gimbal_pitch  # Convert to Google Earth tilt system
        ge_tilt = max(0, min(180, ge_tilt))  # Clamp to valid range
        tilt.text = str(ge_tilt)
        
        roll = SubElement(camera, 'roll')
        roll.text = str(data.gimbal_roll)
        
        altitude_mode = SubElement(camera, 'altitudeMode')
        altitude_mode.text = 'absolute'
        
        # Range/distance from camera position (how far to look ahead)
        range_elem = SubElement(camera, 'range')
        range_elem.text = '500'  # Look 500m ahead - adjust as needed
    
    def generate_flight_path_kml(self, path_points: list, output_file: str = None) -> str:
        """
        Generate KML with flight path visualization.
        
        Args:
            path_points: List of (lat, lon, alt) tuples representing flight path
            output_file: Output file path (optional)
            
        Returns:
            Path to generated KML file
        """
        if output_file is None:
            output_file = os.path.join(self.output_dir, "flight_path.kml")
        
        kml = Element('kml', xmlns="http://www.opengis.net/kml/2.2")
        document = SubElement(kml, 'Document')
        
        # Document info
        name = SubElement(document, 'name')
        name.text = "Flight Path"
        
        # Style for path line
        style = SubElement(document, 'Style', id='path_style')
        line_style = SubElement(style, 'LineStyle')
        
        color = SubElement(line_style, 'color')
        color.text = 'ff0000ff'  # Red line
        
        width = SubElement(line_style, 'width')
        width.text = '3'
        
        # Path placemark
        placemark = SubElement(document, 'Placemark')
        
        pm_name = SubElement(placemark, 'name')
        pm_name.text = "Flight Path"
        
        style_url = SubElement(placemark, 'styleUrl')
        style_url.text = '#path_style'
        
        # LineString
        line_string = SubElement(placemark, 'LineString')
        
        tessellate = SubElement(line_string, 'tessellate')
        tessellate.text = '1'
        
        altitude_mode = SubElement(line_string, 'altitudeMode')
        altitude_mode.text = 'absolute'
        
        coordinates = SubElement(line_string, 'coordinates')
        coord_text = ' '.join([f"{lon},{lat},{alt}" for lat, lon, alt in path_points])
        coordinates.text = coord_text
        
        # Write to file
        self._write_kml_file(kml, output_file)
        
        return output_file
    
    def _write_kml_file(self, kml_element: Element, file_path: str) -> None:
        """Write KML element to file with proper formatting."""
        # Convert to string
        rough_string = tostring(kml_element, 'unicode')
        
        # Pretty print
        reparsed = minidom.parseString(rough_string)
        pretty_string = reparsed.toprettyxml(indent="  ")
        
        # Remove empty lines and write to file
        with open(file_path, 'w', encoding='utf-8') as f:
            for line in pretty_string.split('\n'):
                if line.strip():
                    f.write(line + '\n')
    
    def get_network_link_path(self) -> str:
        """Get the path to the main network link KML file."""
        self._ensure_output_directory()
        return os.path.abspath(self.network_link_path)
    
    def cleanup(self) -> None:
        """Clean up resources and stop auto-update."""
        self.stop_auto_update()
    
    def __del__(self):
        """Cleanup when object is destroyed."""
        self.cleanup()