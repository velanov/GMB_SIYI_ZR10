"""
Gimbal Fixed Target System - Extracted functionality from gimbal_gps_ui_v2.py
Handles fixed coordinate targeting, gimbal locking, and target mode management
"""

import math
import time
import threading
from typing import Optional, Tuple, Dict, Any
from pymavlink import mavutil

# Constants
EARTH_RADIUS = 6378137.0

def calculate_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Distance between two GPS points in meters"""
    if None in [lat1, lat2, lon1, lon2]:
        return float('inf')
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat, dlon = lat2 - lat1, lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    return EARTH_RADIUS * 2 * math.asin(math.sqrt(a))

def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calculate bearing from point 1 to point 2"""
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlon = lon2 - lon1
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    bearing = math.atan2(y, x)
    return (math.degrees(bearing) + 360) % 360

class TargetCalculator:
    """Extended target calculator with gimbal angle calculation"""
    
    @staticmethod
    def calculate_gimbal_angles(aircraft_lat: float, aircraft_lon: float, aircraft_alt_agl: float,
                               aircraft_heading: float, target_lat: float, target_lon: float, 
                               target_alt: float = 0.0) -> Optional[Dict[str, Any]]:
        """Calculate required gimbal angles to point at target coordinates"""
        try:
            # Calculate horizontal distance and bearing
            distance_2d = calculate_distance(aircraft_lat, aircraft_lon, target_lat, target_lon)
            if distance_2d < 1.0:  # Too close
                return None
            
            bearing = calculate_bearing(aircraft_lat, aircraft_lon, target_lat, target_lon)
            
            # Calculate relative yaw (bearing relative to aircraft heading)
            relative_yaw = (bearing - aircraft_heading + 360) % 360
            if relative_yaw > 180:
                relative_yaw -= 360
            
            # Calculate pitch angle (negative for downward)
            alt_diff = target_alt - aircraft_alt_agl  # Negative if target below aircraft
            pitch_rad = math.atan2(-alt_diff, distance_2d)  # Negative for downward look
            pitch_deg = math.degrees(pitch_rad)
            
            # Convert relative yaw to gimbal yaw (0-360)
            gimbal_yaw = relative_yaw % 360
            
            return {
                'yaw': gimbal_yaw,
                'pitch': pitch_deg,
                'bearing': bearing,
                'relative_yaw': relative_yaw,
                'distance_2d': distance_2d,
                'altitude_diff': alt_diff,
                'target_above': alt_diff > 0
            }
        except Exception:
            return None

class GimbalLocker:
    """Gimbal lock system that continuously points gimbal at target coordinates"""
    
    def __init__(self, gimbal):
        self.gimbal = gimbal
        self.active = False
        self.target_lat = None
        self.target_lon = None
        self.target_alt = None
        self.aircraft_state = None
        self.update_interval = 0.2  # Update gimbal angles every 200ms
        self.last_update = 0
        self.angle_threshold = 3.0  # Only move if angle diff > 3 degrees
        self.last_commanded_pitch = None
        self.last_commanded_yaw = None
        self._stop = False
        self.last_state_log_time = 0
        self._worker_thread = threading.Thread(target=self._gimbal_worker, daemon=True)
        self._worker_thread.start()
    
    def start_locking(self, target_lat: float, target_lon: float, target_alt: float = 0.0):
        """Start gimbal lock on target coordinates"""
        self.target_lat = target_lat
        self.target_lon = target_lon
        self.target_alt = target_alt
        self.active = True
        self.last_update = 0  # Force immediate update
        
        # Log target setting if logger exists
        if hasattr(self.gimbal, 'logger'):
            self.gimbal.logger.log_target_set(target_lat, target_lon, target_alt, "gimbal_lock")
        
    def stop_locking(self):
        """Stop gimbal lock"""
        self.active = False
        
    def update_aircraft_state(self, aircraft_state: Dict[str, float]):
        """Update current aircraft position for gimbal calculations"""
        self.aircraft_state = aircraft_state
        
    def update_target(self, target_lat: float, target_lon: float, target_alt: float = 0.0):
        """Update target coordinates during active lock"""
        self.target_lat = target_lat
        self.target_lon = target_lon
        self.target_alt = target_alt
        
    def _gimbal_worker(self):
        """Background worker that continuously updates gimbal angles"""
        while not self._stop:
            try:
                if (self.active and 
                    self.aircraft_state and
                    self.target_lat is not None and
                    time.time() - self.last_update >= self.update_interval):
                    
                    # Calculate required gimbal angles
                    angles = TargetCalculator.calculate_gimbal_angles(
                        self.aircraft_state['lat'], self.aircraft_state['lon'], 
                        self.aircraft_state['alt_agl'], self.aircraft_state['heading'],
                        self.target_lat, self.target_lon, self.target_alt
                    )
                    
                    if angles:
                        # Log calculated angles and aircraft state if logger exists
                        if hasattr(self.gimbal, 'logger'):
                            distance_2d = angles.get('distance_2d', 0)
                            self.gimbal.logger.log_gimbal_angles(angles, distance_2d)
                            self.gimbal.logger.log_aircraft_state(
                                self.aircraft_state['lat'], self.aircraft_state['lon'],
                                self.aircraft_state['alt_agl'], self.aircraft_state['heading']
                            )
                        
                        # Clamp pitch to safe limits
                        required_pitch = max(min(angles['pitch'], 89.0), -89.0)
                        required_yaw = angles['yaw']
                        
                        # Only update gimbal if angles changed significantly
                        should_update = False
                        if self.last_commanded_pitch is None or self.last_commanded_yaw is None:
                            should_update = True
                        else:
                            # Compare required angles vs CURRENT gimbal position
                            current_yaw = self.gimbal.yaw_abs if self.gimbal.yaw_abs is not None else 0
                            current_pitch = self.gimbal.pitch_norm if self.gimbal.pitch_norm is not None else 0
                            
                            pitch_diff = abs(required_pitch - current_pitch)
                            yaw_diff = abs(required_yaw - current_yaw)
                            if yaw_diff > 180:  # Handle wraparound
                                yaw_diff = 360 - yaw_diff
                            
                            if pitch_diff > self.angle_threshold or yaw_diff > self.angle_threshold:
                                should_update = True
                        
                        if should_update and self.gimbal.is_connected:
                            print(f"[GIMBAL LOCK] Commanding gimbal: P:{required_pitch:.1f}° Y:{required_yaw:.1f}°")
                            # Use set_angle method if available, otherwise use jog
                            if hasattr(self.gimbal, 'set_angle'):
                                self.gimbal.set_angle(required_yaw, required_pitch, speed=80)
                            else:
                                # Calculate jog speeds based on angle differences
                                yaw_speed = max(-100, min(100, (required_yaw - current_yaw) * 2))
                                pitch_speed = max(-100, min(100, (required_pitch - current_pitch) * 2))
                                self.gimbal.jog(int(yaw_speed), int(pitch_speed))
                            
                            self.last_commanded_pitch = required_pitch
                            self.last_commanded_yaw = required_yaw
                        elif self.gimbal.is_connected and not should_update:
                            # Stop movement when close to target
                            if hasattr(self.gimbal, 'stop_movement'):
                                self.gimbal.stop_movement()
                            else:
                                self.gimbal.jog(0, 0)
                            
                    self.last_update = time.time()
                    
                # Log gimbal state every 2 seconds
                if hasattr(self.gimbal, 'logger') and time.time() - self.last_state_log_time > 2.0:
                    if self.gimbal.yaw_abs is not None and self.gimbal.pitch_norm is not None:
                        self.gimbal.logger.log_gimbal_state(
                            self.gimbal.yaw_abs, self.gimbal.pitch_norm, self.gimbal.is_connected
                        )
                    self.last_state_log_time = time.time()
                    
                time.sleep(0.1)
            except Exception:
                time.sleep(1.0)
    
    def get_lock_info(self) -> Dict[str, Any]:
        """Get current gimbal lock status information"""
        if not self.active or not self.aircraft_state or self.target_lat is None:
            return {'active': False}
            
        angles = TargetCalculator.calculate_gimbal_angles(
            self.aircraft_state['lat'], self.aircraft_state['lon'],
            self.aircraft_state['alt_agl'], self.aircraft_state['heading'],
            self.target_lat, self.target_lon, self.target_alt
        )
        
        return {
            'active': True,
            'target_lat': self.target_lat,
            'target_lon': self.target_lon,
            'target_alt': self.target_alt,
            'required_angles': angles,
            'last_commanded': {
                'pitch': self.last_commanded_pitch,
                'yaw': self.last_commanded_yaw
            } if self.last_commanded_pitch is not None else None
        }
    
    def cleanup(self):
        """Stop gimbal lock worker thread"""
        self._stop = True
        self.active = False

class FixedTargetManager:
    """Manages fixed target coordinates and target mode switching"""
    
    def __init__(self):
        self.target_mode = "gimbal"  # "gimbal" or "fixed"
        self.fixed_target_state = {
            'lat': None,
            'lon': None,
            'alt': 0.0
        }
        self.gimbal_target_state = {
            'lat': None,
            'lon': None,
            'distance': 0.0,
            'calculation_result': None
        }
    
    def set_target_mode(self, mode: str):
        """Set target mode: 'gimbal' or 'fixed'"""
        if mode in ["gimbal", "fixed"]:
            self.target_mode = mode
    
    def set_fixed_target(self, lat: float, lon: float, alt: float = 0.0) -> bool:
        """Set fixed coordinate target"""
        try:
            # Validate coordinates
            if not (-90 <= lat <= 90):
                return False, "Latitude must be between -90 and 90 degrees"
            if not (-180 <= lon <= 180):
                return False, "Longitude must be between -180 and 180 degrees"
            if alt < -500 or alt > 10000:
                return False, "Altitude must be between -500 and 10000 meters"
            
            self.fixed_target_state['lat'] = lat
            self.fixed_target_state['lon'] = lon
            self.fixed_target_state['alt'] = alt
            return True, "Fixed target set successfully"
            
        except ValueError:
            return False, "Invalid coordinate format"
    
    def set_fixed_target_from_aircraft(self, aircraft_state: Dict[str, float]) -> bool:
        """Use current aircraft position as fixed target"""
        if aircraft_state and aircraft_state.get('lat') is not None:
            return self.set_fixed_target(
                aircraft_state['lat'],
                aircraft_state['lon'],
                aircraft_state.get('alt_agl', 0.0)
            )
        return False, "No aircraft position available"
    
    def get_current_target(self) -> Tuple[Optional[float], Optional[float], Optional[float]]:
        """Get current target coordinates based on selected mode"""
        if self.target_mode == "fixed":
            return (self.fixed_target_state['lat'], 
                    self.fixed_target_state['lon'],
                    self.fixed_target_state['alt'])
        else:
            # Gimbal mode
            if self.gimbal_target_state['lat'] is not None:
                return (self.gimbal_target_state['lat'],
                        self.gimbal_target_state['lon'], 
                        0.0)  # Default altitude for gimbal mode
        return (None, None, None)
    
    def update_gimbal_target(self, lat: Optional[float], lon: Optional[float], 
                           distance: float = 0.0, calculation_result: Dict = None):
        """Update gimbal-calculated target"""
        self.gimbal_target_state['lat'] = lat
        self.gimbal_target_state['lon'] = lon
        self.gimbal_target_state['distance'] = distance
        self.gimbal_target_state['calculation_result'] = calculation_result
    
    def get_target_info(self, aircraft_state: Dict[str, float] = None) -> Dict[str, Any]:
        """Get comprehensive target information"""
        info = {
            'mode': self.target_mode,
            'has_target': False,
            'target_lat': None,
            'target_lon': None,
            'target_alt': None,
            'distance_to_target': None
        }
        
        target_lat, target_lon, target_alt = self.get_current_target()
        
        if target_lat is not None:
            info.update({
                'has_target': True,
                'target_lat': target_lat,
                'target_lon': target_lon,
                'target_alt': target_alt
            })
            
            # Calculate distance to target if aircraft position available
            if aircraft_state and aircraft_state.get('lat') is not None:
                info['distance_to_target'] = calculate_distance(
                    aircraft_state['lat'], aircraft_state['lon'],
                    target_lat, target_lon
                )
        
        # Add mode-specific information
        if self.target_mode == "fixed":
            info['fixed_target'] = self.fixed_target_state.copy()
        else:
            info['gimbal_target'] = self.gimbal_target_state.copy()
        
        return info

class EnhancedDynamicTracker:
    """Enhanced dynamic tracker that works with both gimbal and fixed coordinates"""
    
    def __init__(self, mavlink_handler, target_manager: FixedTargetManager):
        self.mavlink = mavlink_handler
        self.target_manager = target_manager
        self.active = False
        self.last_update = 0
        self.update_interval = 1.0  # Default update interval
        self.min_movement = 10.0    # Default minimum movement threshold
        self.loiter_radius = 500.0  # Default loiter radius
        self.start_time = None
        self.update_count = 0
        self.last_center_lat = None
        self.last_center_lon = None
        self._stop = False
        self._worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker_thread.start()
    
    def start_tracking(self, radius: float, update_interval: float, min_movement: float) -> Tuple[bool, str]:
        """Start tracking current target"""
        if not self.mavlink.connected:
            return False, "No MAVLink connection"
        
        target_lat, target_lon, target_alt = self.target_manager.get_current_target()
        if target_lat is None:
            if self.target_manager.target_mode == "gimbal":
                return False, "No gimbal target calculated - point gimbal downward first"
            else:
                return False, "No fixed target set - set coordinates first"
        
        # Use aircraft altitude for loiter
        # This would need to be passed from the main application
        alt = 100.0  # Default altitude, should be set from aircraft state
        
        self.loiter_radius = radius
        self.update_interval = update_interval
        self.min_movement = min_movement
        
        if not self.mavlink.set_loiter_mode(target_lat, target_lon, alt, radius):
            return False, "Failed to set loiter mode"
        
        self.active = True
        self.start_time = time.time()
        self.update_count = 0
        self.last_center_lat = target_lat
        self.last_center_lon = target_lon
        self.last_update = time.time()
        return True, "Tracking started successfully"
    
    def stop_tracking(self):
        """Stop tracking"""
        self.active = False
    
    def _worker_loop(self):
        """Worker loop for dynamic tracking"""
        while not self._stop:
            try:
                if (self.active and 
                    time.time() - self.last_update >= self.update_interval):
                    
                    target_lat, target_lon, target_alt = self.target_manager.get_current_target()
                    
                    if target_lat is not None and self.last_center_lat and self.last_center_lon:
                        distance_moved = calculate_distance(
                            self.last_center_lat, self.last_center_lon,
                            target_lat, target_lon
                        )
                        if distance_moved >= self.min_movement:
                            # Use aircraft altitude for repositioning
                            alt = target_alt if target_alt is not None else 100.0
                            if self.mavlink.reposition(target_lat, target_lon, alt):
                                self.last_center_lat = target_lat
                                self.last_center_lon = target_lon
                                self.update_count += 1
                    
                    self.last_update = time.time()
                
                time.sleep(0.1)
            except Exception:
                time.sleep(1.0)
    
    def get_stats(self) -> Dict[str, Any]:
        """Get tracking statistics"""
        duration = (time.time() - self.start_time) if self.start_time else 0
        return {
            'active': self.active,
            'duration': duration,
            'updates': self.update_count,
            'rate_per_min': (self.update_count / max(duration, 1)) * 60,
            'next_update': max(0, self.update_interval - (time.time() - self.last_update))
        }
    
    def cleanup(self):
        """Cleanup tracker"""
        self._stop = True
        self.active = False

# Example integration class that shows how to use these components together
class FixedTargetSystem:
    """Complete fixed target system integration"""
    
    def __init__(self, gimbal, mavlink_handler):
        self.gimbal = gimbal
        self.mavlink = mavlink_handler
        
        # Core components
        self.target_manager = FixedTargetManager()
        self.gimbal_locker = GimbalLocker(gimbal)
        self.tracker = EnhancedDynamicTracker(mavlink_handler, self.target_manager)
        
        # State
        self.aircraft_state = {}
    
    def update_aircraft_state(self, state: Dict[str, float]):
        """Update aircraft state across all components"""
        self.aircraft_state = state
        self.gimbal_locker.update_aircraft_state(state)
    
    def set_fixed_target(self, lat: float, lon: float, alt: float = 0.0) -> Tuple[bool, str]:
        """Set fixed target and switch to fixed mode"""
        success, message = self.target_manager.set_fixed_target(lat, lon, alt)
        if success:
            self.target_manager.set_target_mode("fixed")
        return success, message
    
    def set_gimbal_mode(self):
        """Switch to gimbal mode"""
        self.target_manager.set_target_mode("gimbal")
    
    def start_gimbal_lock(self) -> Tuple[bool, str]:
        """Start gimbal lock on current target"""
        target_lat, target_lon, target_alt = self.target_manager.get_current_target()
        if target_lat is None:
            return False, "No target available"
        
        # Use fixed target altitude or default to ground level
        lock_target_alt = target_alt if target_alt is not None else 0.0
        self.gimbal_locker.start_locking(target_lat, target_lon, lock_target_alt)
        return True, "Gimbal lock started"
    
    def stop_gimbal_lock(self):
        """Stop gimbal lock"""
        self.gimbal_locker.stop_locking()
    
    def start_dynamic_tracking(self, radius: float = 500.0, update_interval: float = 1.0, 
                             min_movement: float = 10.0) -> Tuple[bool, str]:
        """Start dynamic tracking"""
        return self.tracker.start_tracking(radius, update_interval, min_movement)
    
    def stop_dynamic_tracking(self):
        """Stop dynamic tracking"""
        self.tracker.stop_tracking()
    
    def update_gimbal_target(self, result):
        """Update gimbal calculated target"""
        if result:
            self.target_manager.update_gimbal_target(
                result['lat'], result['lon'], result['distance'], result
            )
            
            # Update gimbal lock target if active
            if self.gimbal_locker.active:
                lock_target_alt = 0.0  # Ground level for gimbal targets
                self.gimbal_locker.update_target(result['lat'], result['lon'], lock_target_alt)
    
    def get_system_status(self) -> Dict[str, Any]:
        """Get complete system status"""
        return {
            'target_info': self.target_manager.get_target_info(self.aircraft_state),
            'gimbal_lock_info': self.gimbal_locker.get_lock_info(),
            'tracking_stats': self.tracker.get_stats()
        }
    
    def cleanup(self):
        """Cleanup all components"""
        self.gimbal_locker.cleanup()
        self.tracker.cleanup()

# For backward compatibility
def calculate_target(aircraft_lat: float, aircraft_lon: float, aircraft_alt_agl: float,
                    pitch_deg: float, yaw_deg: float) -> Optional[Dict[str, Any]]:
    """Legacy target calculation function for compatibility"""
    if pitch_deg < -90:
        effective_yaw = (yaw_deg + 180.0) % 360.0
        note = "Backward-Sky"
    elif pitch_deg <= 0:
        effective_yaw = yaw_deg
        note = "Sky/Horizon"
    else:
        effective_yaw = yaw_deg
        note = "Ground"
    
    if pitch_deg <= 0:
        distance = 5000  # 5km fixed distance
        a = math.radians(effective_yaw)
        dN = math.cos(a) * distance
        dE = math.sin(a) * distance
        lat0 = math.radians(aircraft_lat)
        dlat = dN / EARTH_RADIUS
        dlon = dE / (EARTH_RADIUS * max(1e-9, math.cos(lat0)))
        lat = aircraft_lat + math.degrees(dlat)
        lon = aircraft_lon + math.degrees(dlon)
        return {
            'lat': lat, 'lon': lon, 'distance': distance,
            'pitch': pitch_deg, 'yaw': yaw_deg, 'effective_yaw': effective_yaw,
            'note': f'{note} - Fixed 5km'
        }
    
    if aircraft_alt_agl <= 0:
        return None
        
    eps = math.radians(pitch_deg)
    a = math.radians(yaw_deg)
    vN = math.cos(eps) * math.cos(a)
    vE = math.cos(eps) * math.sin(a)
    vD = math.sin(eps)
    if vD <= 1e-6:
        return None
    t = aircraft_alt_agl / vD
    dN, dE = vN * t, vE * t
    distance = math.hypot(dN, dE)
    max_distance = 5000
    if distance > max_distance:
        scale = max_distance / distance
        dN *= scale; dE *= scale; distance = max_distance
        note += " - Capped"
    else:
        note += " - Normal"
    
    lat0 = math.radians(aircraft_lat)
    dlat = dN / EARTH_RADIUS
    dlon = dE / (EARTH_RADIUS * max(1e-9, math.cos(lat0)))
    lat = aircraft_lat + math.degrees(dlat)
    lon = aircraft_lon + math.degrees(dlon)
    return {
        'lat': lat, 'lon': lon, 'distance': distance,
        'pitch': pitch_deg, 'yaw': yaw_deg,
        'note': note
    }