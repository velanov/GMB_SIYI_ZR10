from ..shared import *
import numpy as np
from typing import Protocol
from dataclasses import dataclass

@dataclass
class Position:
    """Position in latitude, longitude, altitude (MSL) format."""
    lat: float  # degrees
    lon: float  # degrees
    alt: float  # meters MSL

@dataclass
class NEDPosition:
    """Position in North-East-Down coordinate system."""
    north: float  # meters
    east: float   # meters
    down: float   # meters

@dataclass
class EulerAngles:
    """Euler angles in radians (roll, pitch, yaw)."""
    roll: float   # rotation about x-axis
    pitch: float  # rotation about y-axis  
    yaw: float    # rotation about z-axis

@dataclass
class GimbalAngles:
    """Gimbal pointing angles."""
    pitch: float  # radians, negative = down
    yaw: float    # radians, positive = right

@dataclass
class TargetingResult:
    """Result of 3D targeting calculation."""
    target_position: Optional[Position]
    raw_estimate: Position  # Flat earth assumption
    iterations: int
    converged: bool
    final_error: float  # meters
    processing_time: float  # seconds

class TerrainService(Protocol):
    """Protocol for terrain elevation services."""
    def get_elevation(self, lat: float, lon: float) -> float:
        """Get terrain elevation at given coordinates."""
        return 0.0  # Default to sea level

class TargetCalculator:
    """Enhanced target position calculation with 3D corrections and terrain awareness"""
    
    # Earth radius in meters (WGS84)
    EARTH_RADIUS = 6378137.0
    
    def __init__(self, terrain_service: Optional[TerrainService] = None, 
                 max_iterations: int = 10, tolerance_meters: float = 0.5):
        """Initialize with optional terrain service."""
        self.terrain_service = terrain_service or self._default_terrain_service()
        self.max_iterations = max_iterations
        self.tolerance = tolerance_meters
    
    def _default_terrain_service(self):
        """Default terrain service using offline SRTM tiles."""
        try:
            from ..elevation import OfflineSRTMService
            return OfflineSRTMService()
        except ImportError as e:
            print(f"[TERRAIN] Could not import offline SRTM service: {e}")
            # Fallback to simple sea-level service
            class FallbackTerrain:
                def get_elevation(self, lat: float, lon: float) -> float:
                    print(f"[TERRAIN] Using fallback sea-level for {lat:.6f}, {lon:.6f}")
                    return 0.0
            return FallbackTerrain()
    
    def calculate_target_3d(self, aircraft_lat: float, aircraft_lon: float, aircraft_alt_agl: float,
                           pitch_deg: float, yaw_deg: float, 
                           aircraft_roll_deg: float = 0.0, aircraft_pitch_deg: float = 0.0, 
                           aircraft_yaw_deg: float = 0.0) -> TargetingResult:
        """
        Calculate target location using 3D ray-terrain intersection with UAV attitude correction.
        
        Args:
            aircraft_lat, aircraft_lon, aircraft_alt_agl: UAV position
            pitch_deg, yaw_deg: Gimbal angles in degrees
            aircraft_roll_deg, aircraft_pitch_deg, aircraft_yaw_deg: UAV attitude in degrees
        
        Returns:
            TargetingResult: Complete 3D targeting result
        """
        import time
        start_time = time.time()
        
        # Convert to proper data structures
        uav_position = Position(lat=aircraft_lat, lon=aircraft_lon, alt=aircraft_alt_agl)
        gimbal_angles = GimbalAngles(pitch=math.radians(pitch_deg), yaw=math.radians(yaw_deg))
        uav_attitude = EulerAngles(
            roll=math.radians(aircraft_roll_deg),
            pitch=math.radians(aircraft_pitch_deg), 
            yaw=math.radians(aircraft_yaw_deg)
        )
        
        # Get 3D pointing vector in NED frame
        pointing_vector = self._gimbal_to_ned_rotation(gimbal_angles, uav_attitude)
        
        # Check if pointing down
        if pointing_vector[2] <= 0:
            # Looking up or horizontal - use legacy logic
            processing_time = time.time() - start_time
            raw_estimate = self._calculate_raw_estimate(uav_position, pointing_vector)
            return TargetingResult(
                target_position=None,
                raw_estimate=raw_estimate,
                iterations=0,
                converged=False,
                final_error=float('inf'),
                processing_time=processing_time
            )
        
        # Calculate raw estimate (flat earth assumption)
        raw_estimate = self._calculate_raw_estimate(uav_position, pointing_vector)
        
        # Iterative refinement with terrain
        target_position, iterations, final_error = self._iterative_intersection(
            uav_position, pointing_vector
        )
        
        processing_time = time.time() - start_time
        converged = target_position is not None
        
        return TargetingResult(
            target_position=target_position,
            raw_estimate=raw_estimate,
            iterations=iterations,
            converged=converged,
            final_error=final_error,
            processing_time=processing_time
        )
    
    @staticmethod
    def calculate_target_basic(aircraft_lat: float, aircraft_lon: float, aircraft_alt_agl: float,
                              pitch_deg: float, yaw_deg: float, aircraft_heading_deg: float = 0.0) -> Optional[Dict[str, Any]]:
        """Calculate target using basic gimbal math with heading but without full 3D attitude corrections"""
        if pitch_deg <= 0:  # Not pointing down
            return None
            
        # Calculate horizontal distance using basic trigonometry
        horizontal_distance = aircraft_alt_agl / math.tan(math.radians(pitch_deg))
        
        # Convert gimbal yaw to absolute bearing (add aircraft heading)
        absolute_bearing_deg = (aircraft_heading_deg + yaw_deg) % 360.0
        bearing_rad = math.radians(absolute_bearing_deg)
        
        # Calculate lat/lon offsets using spherical earth approximation
        lat_rad = math.radians(aircraft_lat)
        
        # Distance calculations
        lat_offset = horizontal_distance * math.cos(bearing_rad) / 111320.0  # meters to degrees lat
        lon_offset = horizontal_distance * math.sin(bearing_rad) / (111320.0 * math.cos(lat_rad))  # meters to degrees lon
        
        target_lat = aircraft_lat + lat_offset
        target_lon = aircraft_lon + lon_offset
        
        return {
            'lat': target_lat,
            'lon': target_lon,
            'alt': 0.0,  # Basic calculation assumes ground level
            'distance': horizontal_distance,
            'pitch': pitch_deg,
            'yaw': yaw_deg,
            'bearing': absolute_bearing_deg,
            'note': "Basic-Heading-Only"
        }

    @staticmethod
    def calculate_target(aircraft_lat: float, aircraft_lon: float, aircraft_alt_agl: float,
                        pitch_deg: float, yaw_deg: float, 
                        aircraft_roll_deg: float = 0.0, aircraft_pitch_deg: float = 0.0, 
                        aircraft_yaw_deg: float = 0.0) -> Optional[Dict[str, Any]]:
        """Legacy interface - creates calculator instance and returns simplified result."""
        calculator = TargetCalculator()
        result = calculator.calculate_target_3d(
            aircraft_lat, aircraft_lon, aircraft_alt_agl,
            pitch_deg, yaw_deg, aircraft_roll_deg, aircraft_pitch_deg, aircraft_yaw_deg
        )
        
        # Return legacy format for backward compatibility
        target = result.target_position or result.raw_estimate
        if target is None:
            return None
            
        # Calculate distance for compatibility
        distance = calculator._distance_2d(
            Position(aircraft_lat, aircraft_lon, aircraft_alt_agl), target
        )
        
        return {
            'lat': target.lat, 
            'lon': target.lon, 
            'alt': target.alt,  # Include the terrain-corrected altitude
            'distance': distance,
            'pitch': pitch_deg, 
            'yaw': yaw_deg,
            'note': f"3D-Corrected ({'Converged' if result.converged else 'Raw'})",
            'iterations': result.iterations,
            'error_m': result.final_error,
            'processing_time_ms': result.processing_time * 1000
        }

    @staticmethod
    def calculate_gimbal_angles(aircraft_lat: float, aircraft_lon: float, aircraft_alt_agl: float,
                               aircraft_heading: float, target_lat: float, target_lon: float, 
                               target_alt: float = 0.0) -> Optional[Dict[str, Any]]:
        """Calculate gimbal angles needed to point at target coordinates"""
        if None in [aircraft_lat, aircraft_lon, target_lat, target_lon]:
            return None
        
        # Calculate distance and bearing to target
        distance_2d = calculate_distance(aircraft_lat, aircraft_lon, target_lat, target_lon)
        if distance_2d == 0:
            return None
        
        # Calculate bearing from aircraft to target (true north = 0°)
        lat1, lon1, lat2, lon2 = map(math.radians, [aircraft_lat, aircraft_lon, target_lat, target_lon])
        dlon = lon2 - lon1
        
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        bearing_rad = math.atan2(y, x)
        bearing_deg = math.degrees(bearing_rad) % 360.0
        
        # Calculate pitch angle (vertical angle to target)
        # Positive pitch = looking down, negative pitch = looking up
        altitude_diff = aircraft_alt_agl - target_alt  # Height above target
        pitch_rad = math.atan2(altitude_diff, distance_2d)
        pitch_deg = math.degrees(pitch_rad)
        
        # Calculate yaw angle relative to aircraft heading
        # Convert true bearing to relative bearing
        relative_bearing = (bearing_deg - aircraft_heading) % 360.0
        if relative_bearing > 180:
            relative_bearing -= 360.0
        
        # Convert to gimbal yaw (accounting for gimbal coordinate system)
        gimbal_yaw = relative_bearing
        
        return {
            'pitch': pitch_deg,
            'yaw': gimbal_yaw,
            'distance_2d': distance_2d,
            'bearing': bearing_deg,
            'relative_bearing': relative_bearing,
            'altitude_diff': altitude_diff
        }
    
    # ====================================================================
    # 3D MATH INTEGRATION - Supporting Methods
    # ====================================================================
    
    def _euler_to_rotation_matrix(self, angles: EulerAngles) -> np.ndarray:
        """Convert Euler angles to rotation matrix (ZYX sequence)."""
        roll, pitch, yaw = angles.roll, angles.pitch, angles.yaw
        
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        
        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr           ]
        ])
        
        return R
    
    def _gimbal_to_ned_rotation(self, gimbal_angles: GimbalAngles, 
                               uav_attitude: EulerAngles) -> np.ndarray:
        """Convert gimbal angles to NED pointing vector through UAV attitude."""
        # UAV attitude rotation matrix (body to NED)
        R_uav = self._euler_to_rotation_matrix(uav_attitude)
        
        # Gimbal rotation matrix (gimbal to body)
        gimbal_euler = EulerAngles(roll=0, pitch=gimbal_angles.pitch, yaw=gimbal_angles.yaw)
        R_gimbal = self._euler_to_rotation_matrix(gimbal_euler)
        
        # Combined rotation (gimbal to NED)
        R_total = R_uav @ R_gimbal
        
        # Gimbal points along +X axis in its own frame
        gimbal_vector = np.array([1.0, 0.0, 0.0])
        
        # Transform to NED frame
        return R_total @ gimbal_vector
    
    def _lla_to_ned(self, current_pos: Position, reference_pos: Position) -> NEDPosition:
        """Convert LLA position to NED relative to reference position."""
        lat1 = math.radians(reference_pos.lat)
        lon1 = math.radians(reference_pos.lon)
        lat2 = math.radians(current_pos.lat)
        lon2 = math.radians(current_pos.lon)
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        north = dlat * self.EARTH_RADIUS
        east = dlon * self.EARTH_RADIUS * math.cos(lat1)
        down = reference_pos.alt - current_pos.alt
        
        return NEDPosition(north=north, east=east, down=down)
    
    def _ned_to_lla(self, ned_pos: NEDPosition, reference_pos: Position) -> Position:
        """Convert NED position to LLA using reference position."""
        ref_lat_rad = math.radians(reference_pos.lat)
        ref_lon_rad = math.radians(reference_pos.lon)
        
        dlat = ned_pos.north / self.EARTH_RADIUS
        dlon = ned_pos.east / (self.EARTH_RADIUS * math.cos(ref_lat_rad))
        
        new_lat = math.degrees(ref_lat_rad + dlat)
        new_lon = math.degrees(ref_lon_rad + dlon)
        new_alt = reference_pos.alt - ned_pos.down
        
        return Position(lat=new_lat, lon=new_lon, alt=new_alt)
    
    def _distance_2d(self, pos1: Position, pos2: Position) -> float:
        """Calculate 2D distance between two LLA positions."""
        lat1 = math.radians(pos1.lat)
        lon1 = math.radians(pos1.lon)
        lat2 = math.radians(pos2.lat)
        lon2 = math.radians(pos2.lon)
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = (math.sin(dlat/2)**2 + 
             math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return self.EARTH_RADIUS * c
    
    def _calculate_raw_estimate(self, uav_position: Position, 
                              pointing_vector: np.ndarray) -> Position:
        """Calculate target position assuming flat earth at sea level."""
        if pointing_vector[2] <= 0:
            # Default to position below UAV if not pointing down
            return Position(lat=uav_position.lat, lon=uav_position.lon, alt=0.0)
        
        t = uav_position.alt / pointing_vector[2]
        
        # Calculate impact point in NED
        impact_ned = t * pointing_vector
        
        # Convert to LLA
        ned_position = NEDPosition(
            north=impact_ned[0],
            east=impact_ned[1], 
            down=impact_ned[2]
        )
        
        return self._ned_to_lla(ned_position, uav_position)
    
    def _iterative_intersection(self, uav_position: Position,
                              pointing_vector: np.ndarray) -> tuple[Optional[Position], int, float]:
        """Perform iterative ray-terrain intersection."""
        # Initial estimate (flat earth assumption)
        t = uav_position.alt / pointing_vector[2]
        
        for iteration in range(self.max_iterations):
            # Calculate impact point in NED
            impact_ned = t * pointing_vector
            
            # Convert to lat/lon
            ned_position = NEDPosition(
                north=impact_ned[0],
                east=impact_ned[1],
                down=impact_ned[2]
            )
            current_position = self._ned_to_lla(ned_position, uav_position)
            
            # Get terrain elevation at this point
            try:
                terrain_alt = self.terrain_service.get_elevation(
                    current_position.lat, current_position.lon
                )
            except Exception:
                # Fallback to sea level if terrain service fails
                terrain_alt = 0.0
            
            # FIXED: The ray altitude is already computed correctly in current_position.alt
            ray_alt = current_position.alt
            
            # Check convergence
            error = abs(ray_alt - terrain_alt)
            if error < self.tolerance:
                final_position = Position(
                    lat=current_position.lat,
                    lon=current_position.lon,
                    alt=terrain_alt
                )
                return final_position, iteration + 1, error
            
            # Refine estimate
            altitude_error = ray_alt - terrain_alt
            t = t + (altitude_error / pointing_vector[2])
            
            # Safety check - don't let t go negative
            if t < 0:
                break
        
        # Failed to converge
        return None, self.max_iterations, float('inf')
