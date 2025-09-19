"""
Advanced Session Logging System for Gimbal Application
Creates detailed logs and automatic analysis for each flight session
"""

import os
import json
import csv
import time
from datetime import datetime
from typing import Dict, List, Any, Optional
from dataclasses import dataclass, asdict
import threading

@dataclass
class GimbalLogEntry:
    """Single gimbal log entry"""
    timestamp: float
    commanded_pitch: float
    commanded_yaw: float
    actual_pitch: float
    actual_yaw: float
    pitch_error: float
    yaw_error: float
    gimbal_connected: bool
    operation_mode: str  # "manual", "tracking", "centering", "idle"

@dataclass
class CoordinateLogEntry:
    """Single coordinate calculation log entry"""
    timestamp: float
    aircraft_lat: float
    aircraft_lon: float
    aircraft_alt: float
    aircraft_heading: float
    gimbal_pitch: float
    gimbal_yaw: float
    coord_before_euler_lat: float
    coord_before_euler_lon: float
    coord_after_euler_lat: float
    coord_after_euler_lon: float
    coordinate_diff_meters: float
    terrain_elevation: float

@dataclass
class TargetSelectionLogEntry:
    """Target selection event log entry"""
    timestamp: float
    target_lat: float
    target_lon: float
    target_alt: float
    aircraft_lat: float
    aircraft_lon: float
    aircraft_alt_agl: float
    aircraft_heading: float
    gimbal_pitch: float
    gimbal_yaw: float
    coord_before_euler_lat: float
    coord_before_euler_lon: float
    coord_after_euler_lat: float
    coord_after_euler_lon: float
    transformation_impact_meters: float
    target_distance_2d: float
    selection_mode: str  # "gimbal_pointing", "fixed_coordinate", "waypoint"

@dataclass
class SessionMetadata:
    """Session metadata"""
    session_id: str
    start_time: datetime
    end_time: Optional[datetime]
    total_duration_seconds: float
    aircraft_type: str
    gimbal_model: str
    total_gimbal_commands: int
    total_coordinate_calculations: int
    max_altitude: float
    flight_distance_km: float

class SessionLogger:
    """Advanced session logging with automatic analysis"""
    
    def __init__(self, base_logs_dir: str = None):
        # Create logs directory in Documents
        if base_logs_dir is None:
            home_dir = os.path.expanduser("~")
            base_logs_dir = os.path.join(home_dir, "Documents", "Gimbal_Flight_Logs")
        
        self.base_logs_dir = base_logs_dir
        
        # Create session folder
        self.session_id = self._generate_session_id()
        self.session_dir = os.path.join(base_logs_dir, self.session_id)
        os.makedirs(self.session_dir, exist_ok=True)
        
        # Initialize log files
        self.gimbal_log_file = os.path.join(self.session_dir, "gimbal_performance.csv")
        self.coordinate_log_file = os.path.join(self.session_dir, "coordinate_calculations.csv")
        self.target_selection_log_file = os.path.join(self.session_dir, "target_selections.csv")
        self.session_log_file = os.path.join(self.session_dir, "session_summary.json")
        self.raw_log_file = os.path.join(self.session_dir, "raw_application.log")
        
        # Data storage
        self.gimbal_entries: List[GimbalLogEntry] = []
        self.coordinate_entries: List[CoordinateLogEntry] = []
        self.target_selection_entries: List[TargetSelectionLogEntry] = []
        
        # Session tracking
        self.session_start = datetime.now()
        self.metadata = SessionMetadata(
            session_id=self.session_id,
            start_time=self.session_start,
            end_time=None,
            total_duration_seconds=0,
            aircraft_type="Unknown",
            gimbal_model="SIYI ZR10",
            total_gimbal_commands=0,
            total_coordinate_calculations=0,
            max_altitude=0,
            flight_distance_km=0
        )
        
        # Thread safety
        self._lock = threading.Lock()
        
        # Initialize CSV files with headers
        self._initialize_csv_files()
        
        print(f"[SESSION] New flight session started: {self.session_id}")
        print(f"[SESSION] Logs directory: {self.session_dir}")
    
    def _generate_session_id(self) -> str:
        """Generate unique session ID based on timestamp"""
        now = datetime.now()
        return f"Flight_{now.strftime('%Y%m%d_%H%M%S')}"
    
    def _initialize_csv_files(self):
        """Initialize CSV files with headers"""
        # Gimbal performance CSV
        with open(self.gimbal_log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'commanded_pitch', 'commanded_yaw', 
                'actual_pitch', 'actual_yaw', 'pitch_error', 'yaw_error',
                'gimbal_connected', 'operation_mode'
            ])
        
        # Coordinate calculations CSV
        with open(self.coordinate_log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'aircraft_lat', 'aircraft_lon', 'aircraft_alt', 'aircraft_heading',
                'gimbal_pitch', 'gimbal_yaw', 'coord_before_euler_lat', 'coord_before_euler_lon',
                'coord_after_euler_lat', 'coord_after_euler_lon', 'coordinate_diff_meters', 'terrain_elevation'
            ])
        
        # Target selections CSV
        with open(self.target_selection_log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'target_lat', 'target_lon', 'target_alt', 'aircraft_lat', 'aircraft_lon',
                'aircraft_alt_agl', 'aircraft_heading', 'gimbal_pitch', 'gimbal_yaw',
                'coord_before_euler_lat', 'coord_before_euler_lon', 'coord_after_euler_lat', 'coord_after_euler_lon',
                'transformation_impact_meters', 'target_distance_2d', 'selection_mode'
            ])
    
    def log_gimbal_performance(self, commanded_pitch: float, commanded_yaw: float,
                             actual_pitch: float, actual_yaw: float, 
                             gimbal_connected: bool, operation_mode: str = "unknown"):
        """Log gimbal performance data"""
        with self._lock:
            pitch_error = abs(commanded_pitch - actual_pitch)
            yaw_error = abs(commanded_yaw - actual_yaw)
            
            entry = GimbalLogEntry(
                timestamp=time.time(),
                commanded_pitch=commanded_pitch,
                commanded_yaw=commanded_yaw,
                actual_pitch=actual_pitch,
                actual_yaw=actual_yaw,
                pitch_error=pitch_error,
                yaw_error=yaw_error,
                gimbal_connected=gimbal_connected,
                operation_mode=operation_mode
            )
            
            self.gimbal_entries.append(entry)
            self.metadata.total_gimbal_commands += 1
            
            # Write to CSV immediately
            with open(self.gimbal_log_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    entry.timestamp, entry.commanded_pitch, entry.commanded_yaw,
                    entry.actual_pitch, entry.actual_yaw, entry.pitch_error, entry.yaw_error,
                    entry.gimbal_connected, entry.operation_mode
                ])
    
    def log_coordinate_calculation(self, aircraft_lat: float, aircraft_lon: float, aircraft_alt: float,
                                 aircraft_heading: float, gimbal_pitch: float, gimbal_yaw: float,
                                 coord_before_lat: float, coord_before_lon: float,
                                 coord_after_lat: float, coord_after_lon: float,
                                 coordinate_diff_meters: float, terrain_elevation: float = 0.0):
        """Log coordinate calculation data"""
        with self._lock:
            entry = CoordinateLogEntry(
                timestamp=time.time(),
                aircraft_lat=aircraft_lat,
                aircraft_lon=aircraft_lon,
                aircraft_alt=aircraft_alt,
                aircraft_heading=aircraft_heading,
                gimbal_pitch=gimbal_pitch,
                gimbal_yaw=gimbal_yaw,
                coord_before_euler_lat=coord_before_lat,
                coord_before_euler_lon=coord_before_lon,
                coord_after_euler_lat=coord_after_lat,
                coord_after_euler_lon=coord_after_lon,
                coordinate_diff_meters=coordinate_diff_meters,
                terrain_elevation=terrain_elevation
            )
            
            self.coordinate_entries.append(entry)
            self.metadata.total_coordinate_calculations += 1
            self.metadata.max_altitude = max(self.metadata.max_altitude, aircraft_alt)
            
            # Write to CSV immediately
            with open(self.coordinate_log_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    entry.timestamp, entry.aircraft_lat, entry.aircraft_lon, entry.aircraft_alt,
                    entry.aircraft_heading, entry.gimbal_pitch, entry.gimbal_yaw,
                    entry.coord_before_euler_lat, entry.coord_before_euler_lon,
                    entry.coord_after_euler_lat, entry.coord_after_euler_lon,
                    entry.coordinate_diff_meters, entry.terrain_elevation
                ])
    
    def log_target_selection(self, target_lat: float, target_lon: float, target_alt: float,
                           aircraft_lat: float, aircraft_lon: float, aircraft_alt_agl: float,
                           aircraft_heading: float, gimbal_pitch: float, gimbal_yaw: float,
                           coord_before_euler_lat: float, coord_before_euler_lon: float,
                           coord_after_euler_lat: float, coord_after_euler_lon: float,
                           transformation_impact_meters: float, target_distance_2d: float,
                           selection_mode: str = "unknown"):
        """Log target selection event with complete 3D transformation data"""
        with self._lock:
            entry = TargetSelectionLogEntry(
                timestamp=time.time(),
                target_lat=target_lat,
                target_lon=target_lon,
                target_alt=target_alt,
                aircraft_lat=aircraft_lat,
                aircraft_lon=aircraft_lon,
                aircraft_alt_agl=aircraft_alt_agl,
                aircraft_heading=aircraft_heading,
                gimbal_pitch=gimbal_pitch,
                gimbal_yaw=gimbal_yaw,
                coord_before_euler_lat=coord_before_euler_lat,
                coord_before_euler_lon=coord_before_euler_lon,
                coord_after_euler_lat=coord_after_euler_lat,
                coord_after_euler_lon=coord_after_euler_lon,
                transformation_impact_meters=transformation_impact_meters,
                target_distance_2d=target_distance_2d,
                selection_mode=selection_mode
            )
            
            self.target_selection_entries.append(entry)
            
            # Write to CSV immediately
            with open(self.target_selection_log_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    entry.timestamp, entry.target_lat, entry.target_lon, entry.target_alt,
                    entry.aircraft_lat, entry.aircraft_lon, entry.aircraft_alt_agl, entry.aircraft_heading,
                    entry.gimbal_pitch, entry.gimbal_yaw, entry.coord_before_euler_lat, entry.coord_before_euler_lon,
                    entry.coord_after_euler_lat, entry.coord_after_euler_lon, entry.transformation_impact_meters,
                    entry.target_distance_2d, entry.selection_mode
                ])
            
            # Log to console with detailed information
            dt = datetime.fromtimestamp(entry.timestamp)
            print(f"\n{'='*80}")
            print(f"[TARGET LOG] {dt.strftime('%H:%M:%S')} - Target Selected ({selection_mode.upper()})")
            print(f"[TARGET LOG] Target: {target_lat:.6f},{target_lon:.6f} @ {target_alt:.1f}m")
            print(f"[TARGET LOG] Aircraft: {aircraft_lat:.6f},{aircraft_lon:.6f} @ {aircraft_alt_agl:.1f}m AGL, HDG:{aircraft_heading:.1f}°")
            print(f"[TARGET LOG] Gimbal: Pitch={gimbal_pitch:.1f}° Yaw={gimbal_yaw:.1f}°")
            print(f"[TARGET LOG] BEFORE 3D: {coord_before_euler_lat:.6f},{coord_before_euler_lon:.6f}")
            print(f"[TARGET LOG] AFTER 3D:  {coord_after_euler_lat:.6f},{coord_after_euler_lon:.6f}")
            print(f"[TARGET LOG] 3D TRANSFORMATION IMPACT: {transformation_impact_meters:.2f} meters")
            print(f"[TARGET LOG] Target Distance: {target_distance_2d:.1f}m")
            print(f"{'='*80}\n")
    
    def log_raw_message(self, message: str, level: str = "INFO"):
        """Log raw application messages"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        log_line = f"[{timestamp}] [{level}] {message}\n"
        
        with open(self.raw_log_file, 'a') as f:
            f.write(log_line)
    
    def finalize_session(self):
        """Finalize session and trigger analysis"""
        with self._lock:
            self.metadata.end_time = datetime.now()
            self.metadata.total_duration_seconds = (self.metadata.end_time - self.metadata.start_time).total_seconds()
            
            # Save session metadata
            with open(self.session_log_file, 'w') as f:
                # Convert dataclass to dict, handling datetime serialization
                metadata_dict = asdict(self.metadata)
                metadata_dict['start_time'] = self.metadata.start_time.isoformat()
                metadata_dict['end_time'] = self.metadata.end_time.isoformat()
                json.dump(metadata_dict, f, indent=2)
            
            print(f"[SESSION] Flight session completed: {self.session_id}")
            print(f"[SESSION] Duration: {self.metadata.total_duration_seconds:.1f} seconds")
            print(f"[SESSION] Gimbal commands logged: {self.metadata.total_gimbal_commands}")
            print(f"[SESSION] Coordinate calculations logged: {self.metadata.total_coordinate_calculations}")
            print(f"[SESSION] Max altitude: {self.metadata.max_altitude:.1f}m")
            
            return self.session_dir
    
    def get_session_stats(self) -> Dict[str, Any]:
        """Get current session statistics"""
        with self._lock:
            current_duration = (datetime.now() - self.session_start).total_seconds()
            
            gimbal_accuracy = 0.0
            if self.gimbal_entries:
                total_error = sum(entry.pitch_error + entry.yaw_error for entry in self.gimbal_entries)
                avg_error = total_error / (len(self.gimbal_entries) * 2)  # 2 for pitch + yaw
                gimbal_accuracy = max(0, 100 - avg_error)  # Convert error to accuracy percentage
            
            return {
                'session_id': self.session_id,
                'duration_seconds': current_duration,
                'gimbal_commands': len(self.gimbal_entries),
                'coordinate_calculations': len(self.coordinate_entries),
                'gimbal_accuracy_percent': gimbal_accuracy,
                'session_dir': self.session_dir
            }

# Global session logger instance
_session_logger: Optional[SessionLogger] = None

def get_session_logger() -> SessionLogger:
    """Get or create global session logger"""
    global _session_logger
    if _session_logger is None:
        _session_logger = SessionLogger()
    return _session_logger

def finalize_current_session() -> Optional[str]:
    """Finalize current session and return session directory"""
    global _session_logger
    if _session_logger is not None:
        session_dir = _session_logger.finalize_session()
        _session_logger = None
        return session_dir
    return None