from ..shared import *
from ..calc.target_calculator import TargetCalculator
from .siyi_gimbal import SiyiGimbal
from ..session_logging.session_logger import get_session_logger  

class GimbalLocker:
    """Gimbal lock system that continuously points gimbal at target coordinates"""
    
    def __init__(self, gimbal: SiyiGimbal):
        self.gimbal = gimbal
        self.active = False
        self.target_lat = None
        self.target_lon = None
        self.target_alt = None
        self.aircraft_state = None
        self.update_interval = 0.2  # Update gimbal angles every 200ms (faster for smoother control)
        self.last_update = 0
        self.angle_threshold = 3.0  # Only move if angle diff > 3 degrees (wider deadband)
        self.last_commanded_pitch = None
        self.last_commanded_yaw = None
        self._stop = False
        self.last_state_log_time = 0  # For periodic gimbal state logging
        self._worker_thread = threading.Thread(target=self._gimbal_worker, daemon=True)
        self._worker_thread.start()
    
    def start_locking(self, target_lat: float, target_lon: float, target_alt: float = 0.0):
        """Start gimbal lock on target coordinates"""
        self.target_lat = target_lat
        self.target_lon = target_lon
        self.target_alt = target_alt
        self.active = True
        self.last_update = 0  # Force immediate update
        
        # Log target setting
        self.gimbal.logger.log_target_set(target_lat, target_lon, target_alt, "gimbal_lock")
        
    def start_position_lock(self):
        """Start gimbal lock at current position - don't move, just hold position"""
        if not self.gimbal.is_connected:
            print("[GIMBAL LOCK] Cannot start position lock - gimbal not connected")
            return False
            
        # Get current gimbal angles
        current_yaw = self.gimbal.yaw_abs if self.gimbal.yaw_abs is not None else 0
        current_pitch = self.gimbal.pitch_norm if self.gimbal.pitch_norm is not None else 0
        
        # Store current angles as target angles (don't calculate geographic target)
        self.target_angles = {
            'yaw': current_yaw,
            'pitch': current_pitch
        }
        self.position_lock_mode = True  # Flag to use angle-based locking instead of geographic
        self.active = True
        self.last_update = 0  # Force immediate update
        
        print(f"[GIMBAL LOCK] Position lock started at Y:{current_yaw:.1f}° P:{current_pitch:.1f}°")
        return True
        
    def stop_locking(self):
        """Stop gimbal lock"""
        self.active = False
        self.position_lock_mode = False
        self.lock_start_time = 0  # Reset grace period timer
        
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
                        # Log calculated angles and aircraft state
                        distance_2d = angles.get('distance_2d', 0)
                        self.gimbal.logger.log_gimbal_angles(angles, distance_2d)
                        self.gimbal.logger.log_aircraft_state(
                            self.aircraft_state['lat'], self.aircraft_state['lon'],
                            self.aircraft_state['alt_agl'], self.aircraft_state['heading']
                        )
                        
                        # Clamp pitch to safe limits to avoid gimbal lock at extremes
                        required_pitch = max(min(angles['pitch'], 89.0), -89.0)
                        required_yaw = angles['yaw']
                        
                        # Only update gimbal if angles changed significantly
                        should_update = False
                        if self.last_commanded_pitch is None or self.last_commanded_yaw is None:
                            should_update = True
                        else:
                            # CRITICAL FIX: Compare required angles vs CURRENT gimbal position, not last commanded!
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
                            self.gimbal.set_angle(required_yaw, required_pitch, speed=80)
                            self.last_commanded_pitch = required_pitch
                            self.last_commanded_yaw = required_yaw
                        
                    self.last_update = time.time()
                    
                # Log gimbal state every 2 seconds (less frequent to avoid spam)
                if time.time() - self.last_state_log_time > 2.0:
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
