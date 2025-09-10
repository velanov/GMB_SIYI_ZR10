# Auto-refactor (exact) from gimbal_gps_ui_v2.py
# All common imports + Config, SettingsStore, GimbalLogger + constants + utilities.
import tkinter as tk
from tkinter import ttk, messagebox
import math, time, threading, json, os, logging
from datetime import datetime
from pymavlink import mavutil
import socket, struct
from typing import Optional, Tuple, Dict, Any


EARTH_RADIUS = 6378137.0

class Config:
    """Centralized configuration (runtime-updatable)"""
    # Network (will be loaded/saved from JSON)
    SIYI_IP = "192.168.144.25"
    SIYI_PORT = 37260
    SBS_BIND = "0.0.0.0"
    SBS_PORT = 30003
    # RX=listen telemetry, TX=send commands (kept separate for QGC forwarding scenario)
    MAVLINK_ADDRESS = 'udp:127.0.0.1:14540'   # Backward compatibility (RX)
    MAVLINK_TX_ADDRESS = ''                   # Empty = use RX link. For QGC: 'udpout:127.0.0.1:14550'
    
    # Timing
    GUI_UPDATE_MS = 50
    SBS_UPDATE_S = 0.2
    ATTITUDE_REQUEST_MS = 100
    TRACKING_UPDATE_S = 1.0
    
    # Tracking
    DEFAULT_LOITER_RADIUS = 500.0
    MIN_MOVEMENT_THRESHOLD = 10.0
    MAX_DISTANCE_KM = 5.0
    
    # Performance
    CALC_THROTTLE_S = 0.1
    ANGLE_CHANGE_THRESHOLD = 0.5

class GimbalLogger:
    """Comprehensive gimbal command and state logging system"""
    
    def __init__(self, log_file="gimbal_log.txt"):
        self.log_file = log_file
        self.logger = self._setup_logger()
        self.session_start = datetime.now()
        self.log_session_start()
    
    def _setup_logger(self):
        """Setup file logger with timestamps"""
        logger = logging.getLogger('gimbal')
        logger.setLevel(logging.INFO)
        
        # Remove existing handlers
        for handler in logger.handlers[:]:
            logger.removeHandler(handler)
        
        # File handler
        file_handler = logging.FileHandler(self.log_file, mode='a', encoding='utf-8')
        file_handler.setLevel(logging.INFO)
        
        # Format: timestamp | level | message  
        formatter = logging.Formatter('%(asctime)s | %(levelname)s | %(message)s', 
                                    datefmt='%Y-%m-%d %H:%M:%S')
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
        
        return logger
    
    def log_session_start(self):
        """Log session start marker"""
        self.logger.info("="*80)
        self.logger.info(f"GIMBAL SESSION START - {self.session_start.strftime('%Y-%m-%d %H:%M:%S')}")
        self.logger.info("="*80)
    
    def log_target_set(self, lat: float, lon: float, alt: float, mode: str):
        """Log when target coordinates are set"""
        self.logger.info(f"TARGET_SET | Mode: {mode} | Lat: {lat:.6f}° | Lon: {lon:.6f}° | Alt: {alt:.1f}m")
    
    def log_gimbal_command(self, yaw_target: float, pitch_target: float, yaw_current: float, 
                          pitch_current: float, yaw_speed: int, pitch_speed: int):
        """Log gimbal movement commands"""
        self.logger.info(f"GIMBAL_CMD | Target: Y={yaw_target:.1f}° P={pitch_target:.1f}° | "
                        f"Current: Y={yaw_current:.1f}° P={pitch_current:.1f}° | "
                        f"Speed: yaw={yaw_speed} pitch={pitch_speed}")
    
    def log_gimbal_angles(self, angles_dict: dict, distance_2d: float):
        """Log calculated gimbal angles from coordinates"""
        self.logger.info(f"ANGLES_CALC | Required: Y={angles_dict.get('yaw', 0):.1f}° "
                        f"P={angles_dict.get('pitch', 0):.1f}° | Distance: {distance_2d:.1f}m")
    
    def log_aircraft_state(self, lat: float, lon: float, alt: float, heading: float):
        """Log current aircraft position and attitude"""
        self.logger.info(f"AIRCRAFT | Lat: {lat:.6f}° | Lon: {lon:.6f}° | "
                        f"Alt: {alt:.1f}m | Heading: {heading:.1f}°")
    
    def log_gimbal_state(self, yaw: float, pitch: float, is_connected: bool):
        """Log current gimbal state"""
        self.logger.info(f"GIMBAL_STATE | Y={yaw:.1f}° P={pitch:.1f}° | Connected: {is_connected}")
    
    def log_recovery_attempt(self, reason: str, pitch_current: float, pitch_diff: float):
        """Log gimbal recovery attempts"""
        self.logger.warning(f"RECOVERY | Reason: {reason} | Current: {pitch_current:.1f}° | Diff: {pitch_diff:.1f}°")
    
    def log_warning(self, message: str):
        """Log warnings and issues"""
        self.logger.warning(f"WARNING | {message}")
    
    def log_error(self, message: str, exception: Exception = None):
        """Log errors"""
        error_msg = f"ERROR | {message}"
        if exception:
            error_msg += f" | Exception: {str(exception)}"
        self.logger.error(error_msg)

class SettingsStore:
    """Load/Save settings to a JSON file in app directory."""
    BASE_DIR = os.path.abspath(os.path.dirname(__file__))
    PATH = os.path.join(BASE_DIR, "gimbal_gps_settings_v2.json")

    KEYS = [
        "SIYI_IP","SIYI_PORT","SBS_BIND","SBS_PORT",
        "MAVLINK_ADDRESS","MAVLINK_TX_ADDRESS"
    ]

    @staticmethod
    def load_into():
        try:
            with open(SettingsStore.PATH, "r", encoding="utf-8") as f:
                data = json.load(f)
            for k in SettingsStore.KEYS:
                if k in data:
                    setattr(Config, k, data[k])
        except Exception:
            pass

    @staticmethod
    def save_from():
        data = {k: getattr(Config, k) for k in SettingsStore.KEYS}
        try:
            with open(SettingsStore.PATH, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
        except Exception as e:
            print(f"Settings save error: {e}")

# ---- Top-level utility functions (verbatim) ----

def crc16_ccitt(data: bytes) -> int:
    """CRC16 CCITT calculation for SIYI protocol"""
    crc = 0
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def ned_to_geodetic(lat0_deg: float, lon0_deg: float, dN: float, dE: float) -> Tuple[float, float]:
    """Convert NED offset to geodetic coordinates"""
    lat0 = math.radians(lat0_deg)
    dlat = dN / EARTH_RADIUS
    dlon = dE / (EARTH_RADIUS * max(1e-9, math.cos(lat0)))
    return (lat0_deg + math.degrees(dlat), lon0_deg + math.degrees(dlon))

def calculate_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Distance between two GPS points in meters"""
    if None in [lat1, lat2, lon1, lon2]:
        return float('inf')
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat, dlon = lat2 - lat1, lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    return EARTH_RADIUS * 2 * math.asin(math.sqrt(a))

def wrap_360(deg: float) -> float:
    deg = deg % 360.0
    return deg + 360.0 if deg < 0 else deg

def clamp(value: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(max_val, value))
