# gimbal_gps_ui.py
# -*- coding: utf-8 -*-
"""
UI Fixed Version + Settings Panel (persistent JSON) + SIYI stream keepalive
+ MAVLink dual-link (RX/TX) so commands work when using QGC forwarding
"""

import tkinter as tk
from tkinter import ttk, messagebox
import math, time, threading, json, os
from pymavlink import mavutil
import socket, struct
from typing import Optional, Tuple, Dict, Any

# =============================================================================
# CONFIGURATION & CONSTANTS
# =============================================================================

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

class SettingsStore:
    """Load/Save settings to a JSON file in app directory."""
    BASE_DIR = os.path.abspath(os.path.dirname(__file__))
    PATH = os.path.join(BASE_DIR, "gimbal_gps_settings.json")

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

# Constants
EARTH_RADIUS = 6378137.0

# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================

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

# =============================================================================
# CORE SYSTEM CLASSES
# =============================================================================

class SiyiGimbal:
    """SIYI ZR10 gimbal communication handler"""
    
    def __init__(self, ip: str = Config.SIYI_IP, port: int = Config.SIYI_PORT):
        self.ip, self.port = ip, port
        self.sock = None
        self.seq = 1
        self._rx_alive = False
        self._rx_thread = None
        
        # State
        self.yaw_abs = None
        self.pitch_norm = None
        self.roll = None
        self.last_update = 0
        self.mount_dir = None
        self.motion_mode = None

        # Stream keepalive
        self._last_enable = 0.0
        self._stream_hz = 10
    
    def _create_frame(self, cmd: int, payload: bytes = b"") -> bytes:
        stx = b"\x55\x66"
        ctrl = b"\x00"
        dlen = struct.pack("<H", len(payload))
        seq = struct.pack("<H", self.seq & 0xFFFF); self.seq = (self.seq + 1) & 0xFFFF
        body = stx + ctrl + dlen + seq + bytes([cmd]) + payload
        return body + struct.pack("<H", crc16_ccitt(body))

    def _enable_stream(self, hz: Optional[int] = None):
        if not self.sock:
            return
        try:
            hz = hz or self._stream_hz
            freq_map = {0:0, 2:1, 4:2, 5:3, 10:4, 20:5, 50:6, 100:7}
            req = min(freq_map.keys(), key=lambda f: abs(f - hz))
            payload = bytes([1, freq_map[req]])  # data_type=1 (attitude)
            self.sock.sendto(self._create_frame(0x25, payload), (self.ip, self.port))
            self._last_enable = time.time()
        except Exception:
            pass

    def _probe_attitude(self):
        if not self.sock:
            return
        try:
            self.sock.sendto(self._create_frame(0x0D), (self.ip, self.port))
        except Exception:
            pass
    
    def start(self) -> bool:
        if self.sock:
            return True
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.settimeout(0.2)
            self._rx_alive = True
            self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._rx_thread.start()
            self.request_config()
            self._enable_stream(self._stream_hz)
            return True
        except Exception:
            return False
    
    def stop(self):
        self._rx_alive = False
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
    
    def request_attitude(self):
        if self.sock:
            try:
                self.sock.sendto(self._create_frame(0x0D), (self.ip, self.port))
            except Exception:
                pass
    
    def request_config(self):
        if self.sock:
            try:
                self.sock.sendto(self._create_frame(0x0A), (self.ip, self.port))
            except Exception:
                pass
    
    def jog(self, yaw_speed: int, pitch_speed: int):
        if self.sock:
            try:
                y = clamp(yaw_speed, -100, 100)
                p = clamp(pitch_speed, -100, 100)
                payload = struct.pack("<bb", int(y), int(p))
                self.sock.sendto(self._create_frame(0x07, payload), (self.ip, self.port))
            except Exception:
                pass
    
    def center(self):
        if self.sock:
            try:
                self.sock.sendto(self._create_frame(0x08, b"\x01"), (self.ip, self.port))
            except Exception:
                pass
    
    def _parse_packet(self, packet: bytes):
        if len(packet) < 12 or packet[:2] != b"\x55\x66":
            return
        dlen = struct.unpack_from("<H", packet, 3)[0]
        cmd = packet[7]
        payload = packet[8:8+dlen]
        
        if cmd == 0x0D and len(payload) >= 12:
            yaw_i, pitch_i, roll_i = struct.unpack_from("<hhh", payload, 0)[:3]
            self.yaw_abs = (yaw_i / 10.0) % 360.0
            self.pitch_norm = pitch_i / 10.0 - 180.0
            while self.pitch_norm > 180: self.pitch_norm -= 360
            while self.pitch_norm < -180: self.pitch_norm += 360
            self.roll = roll_i / 10.0
            self.last_update = time.time()
        elif cmd == 0x0A and len(payload) >= 6:
            motion_mode = payload[4]
            mount_dir = payload[5]
            self.motion_mode = {0:"Follow", 1:"Lock", 2:"FPV"}.get(motion_mode, f"Unknown({motion_mode})")
            self.mount_dir = {1:"Normal", 2:"UpsideDown"}.get(mount_dir, f"Unknown({mount_dir})")
    
    def _rx_loop(self):
        while self._rx_alive:
            if time.time() - self._last_enable > 2.0:
                self._enable_stream(self._stream_hz)
            try:
                data, _ = self.sock.recvfrom(2048)
                self._parse_packet(data)
            except socket.timeout:
                self._probe_attitude()
                continue
            except OSError:
                break
    
    @property
    def is_connected(self) -> bool:
        return (self.sock is not None and 
                self.yaw_abs is not None and 
                time.time() - self.last_update < 2.0)
    
    def get_corrected_angles(self, aircraft_heading: float) -> Tuple[Optional[float], Optional[float]]:
        if not self.is_connected:
            return None, None
        
        raw_pitch = self.pitch_norm
        raw_yaw = self.yaw_abs
        
        if self.mount_dir == "UpsideDown":
            corrected_pitch = -raw_pitch
            corrected_yaw_offset = (raw_yaw + 180.0) % 360.0
        elif self.mount_dir == "Normal":
            corrected_pitch = raw_pitch
            corrected_yaw_offset = raw_yaw
        else:
            corrected_pitch = raw_pitch
            corrected_yaw_offset = raw_yaw
        
        final_yaw = (corrected_yaw_offset + aircraft_heading + 180.0) % 360.0
        return corrected_pitch, final_yaw


class SBSPublisher:
    """ADS-B SBS format publisher for QGroundControl"""
    
    def __init__(self, host: str = Config.SBS_BIND, port: int = Config.SBS_PORT, 
                 hexid: str = "ABCDEF", callsign: str = "TARGET"):
        self.host, self.port = host, port
        self.hexid = hexid
        self.callsign = callsign[:9]
        self._server = None
        self._connection = None
        self._stop = False
    
    def start(self) -> bool:
        if self._server:
            return True
        try:
            self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._server.bind((self.host, self.port))
            self._server.listen(1)
            self._stop = False
            threading.Thread(target=self._accept_loop, daemon=True).start()
            return True
        except Exception:
            return False
    
    def stop(self):
        self._stop = True
        if self._connection:
            try: self._connection.close()
            except: pass
        if self._server:
            try: self._server.close()
            except: pass
        self._connection = None
        self._server = None
    
    def publish(self, lat: float, lon: float, alt_amsl: float) -> bool:
        if not self._connection:
            return False
        try:
            timestamp = time.strftime("%Y/%m/%d,%H:%M:%S.000")
            alt_ft = int(alt_amsl * 3.28084)
            line1 = (f"MSG,1,1,1,{self.hexid},1,{timestamp},{timestamp},"
                     f"{self.callsign},,,,,,,,,,0\r\n")
            line3 = (f"MSG,3,1,1,{self.hexid},1,{timestamp},{timestamp},"
                     f"{self.callsign},{alt_ft},0,0,{lat:.6f},{lon:.6f},"
                     f"0,1200,0,0,0,0\r\n")
            self._connection.sendall((line1 + line3).encode("ascii"))
            return True
        except OSError:
            try: 
                if self._connection: self._connection.close()
            except: 
                pass
            self._connection = None
            return False
    
    def _accept_loop(self):
        while not self._stop:
            try:
                conn, _ = self._server.accept()
                if self._connection:
                    try: self._connection.close()
                    except: pass
                self._connection = conn
            except OSError:
                break


class MAVLinkHandler:
    """MAVLink communication handler with auto-reconnect and dual RX/TX links.
       - RX: listen telemetry (e.g. QGC forwarding 'udpin:127.0.0.1:14540')
       - TX: send commands (e.g. QGC UDP input 'udpout:127.0.0.1:14550')
    """
    
    def __init__(self, rx_conn: str = Config.MAVLINK_ADDRESS, tx_conn: Optional[str] = None):
        self.rx_conn_str = rx_conn
        self.tx_conn_str = tx_conn or ""
        self.rx_link = None
        self.tx_link = None
        self.connected = False
        self.target_sys = 1
        self.target_comp = 1
        self.reconnect_attempts = 0
        self.max_attempts = 5
        self._connect()
    
    def _send_heartbeat(self):
        """Send a few heartbeats so QGC/routers register us as a peer."""
        try:
            link = self.tx_link or self.rx_link
            if not link: return
            link.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, mavutil.mavlink.MAV_STATE_ACTIVE
            )
        except Exception:
            pass

    def _connect(self) -> bool:
        try:
            # RX link
            self.rx_link = mavutil.mavlink_connection(self.rx_conn_str, source_system=246, source_component=190)
            # TX link (if specified separately)
            if self.tx_conn_str:
                self.tx_link = mavutil.mavlink_connection(self.tx_conn_str, source_system=246, source_component=190)
            else:
                self.tx_link = self.rx_link

            # Send a few heartbeats so QGC sees us
            for _ in range(3):
                self._send_heartbeat()
                time.sleep(0.1)

            # Wait for vehicle heartbeat and capture sysid/compid
            hb = self.rx_link.wait_heartbeat(timeout=10)
            if hb:
                try:
                    self.target_sys = hb.get_srcSystem()
                    self.target_comp = hb.get_srcComponent()
                except Exception:
                    self.target_sys, self.target_comp = 1, 1

            self.connected = True
            self.reconnect_attempts = 0
            return True
        except Exception:
            self.connected = False
            self.reconnect_attempts += 1
            if self.reconnect_attempts < self.max_attempts:
                threading.Timer(2.0, self._connect).start()
            return False

    def set_connection_strings(self, rx_conn: str, tx_conn: Optional[str]):
        """Re-point to new RX/TX strings and reconnect."""
        self.rx_conn_str = rx_conn
        self.tx_conn_str = tx_conn or ""
        # Close and reconnect
        try:
            if self.rx_link and hasattr(self.rx_link, 'port') and self.rx_link.port:
                try: self.rx_link.port.close()
                except: pass
        except Exception:
            pass
        try:
            if self.tx_link and self.tx_link is not self.rx_link and hasattr(self.tx_link, 'port') and self.tx_link.port:
                try: self.tx_link.port.close()
                except: pass
        except Exception:
            pass
        self.connected = False
        self.reconnect_attempts = 0
        self._connect()
    
    def get_position(self) -> Optional[Dict[str, float]]:
        if not self.connected or not self.rx_link:
            return None
        try:
            msg = self.rx_link.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            if msg:
                return {
                    'lat': msg.lat / 1e7,
                    'lon': msg.lon / 1e7,
                    'alt_amsl': msg.alt / 1000.0,
                    'alt_agl': msg.relative_alt / 1000.0
                }
        except Exception:
            self.connected = False
            self._connect()
        return None
    
    def get_attitude(self) -> Optional[float]:
        if not self.connected or not self.rx_link:
            return None
        try:
            msg = self.rx_link.recv_match(type='ATTITUDE', blocking=False)
            if msg:
                hdg = math.degrees(msg.yaw)
                return hdg if hdg >= 0 else hdg + 360
        except Exception:
            self.connected = False
            self._connect()
        return None
    
    def set_loiter_mode(self, lat: float, lon: float, alt: float, radius: float) -> bool:
        if not self.connected or not self.tx_link:
            return False
        try:
            # PX4 Auto-Loiter
            self.tx_link.mav.command_long_send(
                self.target_sys, self.target_comp, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                4, 3, 0, 0, 0, 0
            )
            # Loiter radius param
            self.tx_link.mav.param_set_send(
                self.target_sys, self.target_comp, "NAV_LOITER_RAD".encode('utf-8'), radius,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            # Reposition
            self.tx_link.mav.command_long_send(
                self.target_sys, self.target_comp, mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0,
                -1, 0, 0, 0, lat, lon, alt
            )
            return True
        except Exception:
            return False
    
    def reposition(self, lat: float, lon: float, alt: float) -> bool:
        if not self.connected or not self.tx_link:
            return False
        try:
            self.tx_link.mav.command_long_send(
                self.target_sys, self.target_comp, mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0,
                -1, 0, 0, 0, lat, lon, alt
            )
            return True
        except Exception:
            return False
    
    def set_mission_mode(self) -> bool:
        if not self.connected or not self.tx_link:
            return False
        try:
            self.tx_link.mav.command_long_send(
                self.target_sys, self.target_comp, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                4, 4, 0, 0, 0, 0
            )
            return True
        except Exception:
            return False


class TargetCalculator:
    """Target position calculation with gimbal angles"""
    
    @staticmethod
    def calculate_target(aircraft_lat: float, aircraft_lon: float, aircraft_alt_agl: float,
                        pitch_deg: float, yaw_deg: float) -> Optional[Dict[str, Any]]:
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
            distance = Config.MAX_DISTANCE_KM * 1000
            a = math.radians(effective_yaw)
            dN = math.cos(a) * distance
            dE = math.sin(a) * distance
            lat, lon = ned_to_geodetic(aircraft_lat, aircraft_lon, dN, dE)
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
        max_distance = Config.MAX_DISTANCE_KM * 1000
        if distance > max_distance:
            scale = max_distance / distance
            dN *= scale; dE *= scale; distance = max_distance
            note += " - Capped"
        else:
            note += " - Normal"
        lat, lon = ned_to_geodetic(aircraft_lat, aircraft_lon, dN, dE)
        return {
            'lat': lat, 'lon': lon, 'distance': distance,
            'pitch': pitch_deg, 'yaw': yaw_deg,
            'note': note
        }


class DynamicTracker:
    """Dynamic loiter tracking system"""
    
    def __init__(self, mavlink_handler: MAVLinkHandler):
        self.mavlink = mavlink_handler
        self.active = False
        self.last_update = 0
        self.update_interval = Config.TRACKING_UPDATE_S
        self.min_movement = Config.MIN_MOVEMENT_THRESHOLD
        self.loiter_radius = Config.DEFAULT_LOITER_RADIUS
        self.start_time = None
        self.update_count = 0
        self.last_center_lat = None
        self.last_center_lon = None
        self._stop = False
        self._worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker_thread.start()
    
    def start_tracking(self, initial_lat: float, initial_lon: float, alt: float,
                      radius: float, update_interval: float, min_movement: float) -> bool:
        if not self.mavlink.connected:
            return False
        self.loiter_radius = radius
        self.update_interval = update_interval
        self.min_movement = min_movement
        if not self.mavlink.set_loiter_mode(initial_lat, initial_lon, alt, radius):
            return False
        self.active = True
        self.start_time = time.time()
        self.update_count = 0
        self.last_center_lat = initial_lat
        self.last_center_lon = initial_lon
        self.last_update = time.time()
        return True
    
    def stop_tracking(self):
        self.active = False
    
    def update_target(self, target_lat: float, target_lon: float, alt: float):
        self.target_lat = target_lat
        self.target_lon = target_lon
        self.target_alt = alt
    
    def _worker_loop(self):
        while not self._stop:
            try:
                if (self.active and 
                    time.time() - self.last_update >= self.update_interval and
                    hasattr(self, 'target_lat')):
                    
                    if self.last_center_lat and self.last_center_lon:
                        distance_moved = calculate_distance(
                            self.last_center_lat, self.last_center_lon,
                            self.target_lat, self.target_lon
                        )
                        if distance_moved >= self.min_movement:
                            if self.mavlink.reposition(self.target_lat, self.target_lon, self.target_alt):
                                self.last_center_lat = self.target_lat
                                self.last_center_lon = self.target_lon
                                self.update_count += 1
                    self.last_update = time.time()
                time.sleep(0.1)
            except Exception:
                time.sleep(1.0)
    
    def get_stats(self) -> Dict[str, Any]:
        duration = (time.time() - self.start_time) if self.start_time else 0
        return {
            'active': self.active,
            'duration': duration,
            'updates': self.update_count,
            'rate_per_min': (self.update_count / max(duration, 1)) * 60,
            'next_update': max(0, self.update_interval - (time.time() - self.last_update))
        }
    
    def cleanup(self):
        self._stop = True
        self.active = False

# =============================================================================
# SETTINGS DIALOG
# =============================================================================

class SettingsDialog:
    """Modal settings dialog to edit IPs/ports and MAVLink strings."""

    def __init__(self, parent, on_apply):
        self.parent = parent
        self.on_apply = on_apply
        self.win = tk.Toplevel(parent)
        self.win.title("Settings")
        self.win.resizable(False, False)
        self.win.transient(parent)
        self.win.grab_set()

        frm = ttk.Frame(self.win, padding=10)
        frm.pack(fill="both", expand=True)

        # Profile presets
        rowp = ttk.Frame(frm); rowp.pack(fill="x", pady=(0,8))
        ttk.Label(rowp, text="Profile:", width=16).pack(side="left")
        self.profile_var = tk.StringVar(value="Custom")
        cb = ttk.Combobox(rowp, textvariable=self.profile_var, values=["SITL/WSL", "Operation", "Custom"], state="readonly", width=20)
        cb.pack(side="left")
        cb.bind("<<ComboboxSelected>>", self._on_profile)

        # Fields
        self.siyi_ip = self._entry(frm, "Gimbal IP:", Config.SIYI_IP)
        self.siyi_port = self._entry(frm, "Gimbal Port:", str(Config.SIYI_PORT))
        # MAVLink RX/TX
        self.mavlink_addr = self._entry(frm, "MAVLink RX (listen):", Config.MAVLINK_ADDRESS)
        self.mavlink_tx_addr = self._entry(frm, "MAVLink TX (send):", Config.MAVLINK_TX_ADDRESS or "")
        self.sbs_bind = self._entry(frm, "SBS Bind Host:", Config.SBS_BIND)
        self.sbs_port = self._entry(frm, "SBS Port:", str(Config.SBS_PORT))

        # Buttons
        btns = ttk.Frame(frm); btns.pack(fill="x", pady=(12,0))
        ttk.Button(btns, text="Cancel", command=self.win.destroy).pack(side="right")
        ttk.Button(btns, text="Save & Apply", command=self._apply).pack(side="right", padx=6)

        hint = ttk.Label(frm, foreground="#555",
            text="Hint (QGC Forwarding):\n"
                 "- QGC → MAVLink → Forwarding: 127.0.0.1:14540\n"
                 "- RX: udpin:127.0.0.1:14540\n"
                 "- TX: udpout:127.0.0.1:14550 (QGC UDP input)")
        hint.pack(anchor="w", pady=(6,0))

    def _entry(self, parent, label, value):
        row = ttk.Frame(parent); row.pack(fill="x", pady=3)
        ttk.Label(row, text=label, width=18).pack(side="left")
        var = tk.StringVar(value=value)
        ent = ttk.Entry(row, textvariable=var, width=32); ent.pack(side="left")
        return var

    def _on_profile(self, _):
        p = self.profile_var.get()
        if p == "SITL/WSL":
            self.mavlink_addr.set("udpin:127.0.0.1:14540")
            self.mavlink_tx_addr.set("udpout:127.0.0.1:14550")
            self.sbs_bind.set("0.0.0.0")
            self.sbs_port.set("30003")
        elif p == "Operation":
            # Example listen/send addresses adjusted for operation network
            self.mavlink_addr.set("udpin:0.0.0.0:14540")
            self.mavlink_tx_addr.set("")  # Send via same link
            self.sbs_bind.set("0.0.0.0")
            self.sbs_port.set("30003")
        # Custom -> don't touch

    def _apply(self):
        try:
            new_cfg = {
                "SIYI_IP": self.siyi_ip.get().strip(),
                "SIYI_PORT": int(self.siyi_port.get().strip()),
                "MAVLINK_ADDRESS": self.mavlink_addr.get().strip(),
                "MAVLINK_TX_ADDRESS": self.mavlink_tx_addr.get().strip(),
                "SBS_BIND": self.sbs_bind.get().strip(),
                "SBS_PORT": int(self.sbs_port.get().strip()),
            }
        except Exception:
            messagebox.showerror("Error", "Please enter valid port/values.")
            return

        for k, v in new_cfg.items():
            setattr(Config, k, v)
        SettingsStore.save_from()

        try:
            self.on_apply(new_cfg)
        except Exception as e:
            messagebox.showerror("Error", f"Error applying settings: {e}")
            return

        self.win.destroy()

# =============================================================================
# MANUAL CONTROL WINDOW
# =============================================================================

class ManualControlWindow:
    """Manual gimbal control in separate window"""
    
    def __init__(self, parent, gimbal):
        self.gimbal = gimbal
        
        self.window = tk.Toplevel(parent)
        self.window.title("Manual Gimbal Control")
        self.window.geometry("300x400")
        self.window.transient(parent)
        
        parent_x = parent.winfo_x()
        parent_width = parent.winfo_width()
        self.window.geometry(f"300x400+{parent_x + parent_width + 10}+100")
        
        self._create_controls()
    
    def _create_controls(self):
        ttk.Label(self.window, text="Movement Speed").pack(pady=10)
        self.speed_var = tk.IntVar(value=50)
        speed_scale = ttk.Scale(self.window, from_=10, to=100, variable=self.speed_var, 
                               orient="horizontal", length=250)
        speed_scale.pack(pady=5)
        
        speed_label = ttk.Label(self.window, text="50")
        speed_label.pack()
        speed_scale.config(command=lambda v: speed_label.config(text=f"{int(float(v))}"))
        
        yaw_frame = ttk.LabelFrame(self.window, text="Yaw Control", padding=10)
        yaw_frame.pack(pady=15, padx=10, fill="x")
        yaw_btn_frame = ttk.Frame(yaw_frame); yaw_btn_frame.pack()
        self._create_control_button(yaw_btn_frame, "◄ Left", 
                                   lambda: (-self.speed_var.get(), 0)).pack(side="left", padx=10)
        self._create_control_button(yaw_btn_frame, "Right ►", 
                                   lambda: (self.speed_var.get(), 0)).pack(side="left", padx=10)
        
        pitch_frame = ttk.LabelFrame(self.window, text="Pitch Control", padding=10)
        pitch_frame.pack(pady=15, padx=10, fill="x")
        pitch_btn_frame = ttk.Frame(pitch_frame); pitch_btn_frame.pack()
        self._create_control_button(pitch_btn_frame, "▲ Up", 
                                   lambda: (0, self.speed_var.get())).pack(side="top", pady=5)
        self._create_control_button(pitch_btn_frame, "▼ Down", 
                                   lambda: (0, -self.speed_var.get())).pack(side="top", pady=5)
        
        ttk.Separator(self.window, orient="horizontal").pack(fill="x", pady=15)
        ttk.Button(self.window, text="Center Gimbal", 
                  command=self.gimbal.center).pack(pady=10)
        
        self.status_label = ttk.Label(self.window, text="Manual control ready")
        self.status_label.pack(pady=10)
    
    def _create_control_button(self, parent, text, speed_func):
        btn = ttk.Button(parent, text=text, width=12)
        moving = {"active": False}
        
        def start_movement():
            moving["active"] = True
            self.status_label.config(text=f"Moving: {text}")
            def move():
                if moving["active"]:
                    yaw_speed, pitch_speed = speed_func()
                    self.gimbal.jog(yaw_speed, pitch_speed)
                    self.window.after(50, move)
            move()
        
        def stop_movement():
            moving["active"] = False
            self.gimbal.jog(0, 0)
            self.status_label.config(text="Manual control ready")
        
        btn.bind("<Button-1>", lambda e: start_movement())
        btn.bind("<ButtonRelease-1>", lambda e: stop_movement())
        return btn

# =============================================================================
# MAIN APPLICATION GUI
# =============================================================================

class GimbalGPSApp:
    """Clean main application with fixed UI + Settings"""
    
    def __init__(self):
        # Load persisted settings before creating systems
        SettingsStore.load_into()

        # Initialize core systems with (possibly) loaded config
        self.gimbal = SiyiGimbal(Config.SIYI_IP, Config.SIYI_PORT)
        self.sbs = SBSPublisher(Config.SBS_BIND, Config.SBS_PORT)
        self.mavlink = MAVLinkHandler(Config.MAVLINK_ADDRESS, Config.MAVLINK_TX_ADDRESS or None)
        self.tracker = DynamicTracker(self.mavlink)
        
        # Application state
        self.aircraft_state = {
            'lat': 47.3977508, 'lon': 8.5455938,
            'alt_amsl': 500.0, 'alt_agl': 100.0, 'heading': 0.0
        }
        self.target_state = {
            'lat': None, 'lon': None, 'distance': 0.0,
            'calculation_result': None
        }
        self.ui_settings = {
            'calc_rate_hz': 10.0,
            'angle_threshold': 0.5,
            'loiter_radius': Config.DEFAULT_LOITER_RADIUS,
            'update_interval': Config.TRACKING_UPDATE_S,
            'min_movement': Config.MIN_MOVEMENT_THRESHOLD
        }
        self.last_calc_time = 0
        self.last_angles = (None, None)
        self._closing = False
        self.manual_window = None
        
        self._setup_gui()
        self._start_workers()
        self.root.after(Config.GUI_UPDATE_MS, self._main_loop)
    
    def _setup_gui(self):
        self.root = tk.Tk()
        self.root.title("Gimbal GPS - Dynamic Tracking")
        self.root.geometry("720x700")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        # Menubar with Settings
        menubar = tk.Menu(self.root)
        appmenu = tk.Menu(menubar, tearoff=0)
        appmenu.add_command(label="Settings…", command=self._open_settings)
        appmenu.add_separator()
        appmenu.add_command(label="Exit", command=self._on_close)
        menubar.add_cascade(label="Application", menu=appmenu)
        self.root.config(menu=menubar)
        
        self._create_mavlink_panel()
        self._create_gimbal_panel()
        self._create_target_panel()
        self._create_tracking_panel()
        self._create_controls_panel()
    
    def _create_mavlink_panel(self):
        mavlink_frame = ttk.LabelFrame(self.root, text="MAVLink Status")
        mavlink_frame.pack(padx=10, pady=5, fill="x")
        
        status_text = "Connected ✓" if self.mavlink.connected else "Disconnected (auto-reconnecting...)"
        ttk.Label(mavlink_frame, text=status_text).pack()
        
        self.lbl_telemetry = ttk.Label(mavlink_frame, text=f"RX: {Config.MAVLINK_ADDRESS} | TX: {Config.MAVLINK_TX_ADDRESS or '(same link)'} | Waiting for telemetry...")
        self.lbl_telemetry.pack()
    
    def _create_gimbal_panel(self):
        gimbal_frame = ttk.LabelFrame(self.root, text="SIYI ZR10 Gimbal")
        gimbal_frame.pack(padx=10, pady=5, fill="x")
        
        main_container = ttk.Frame(gimbal_frame); main_container.pack(fill="x", padx=5, pady=5)
        
        left_frame = ttk.Frame(main_container); left_frame.pack(side="left", fill="both", expand=True)
        
        self.var_gimbal = tk.BooleanVar()
        ttk.Checkbutton(left_frame, text=f"Connect SIYI ({Config.SIYI_IP}:{Config.SIYI_PORT})",
                       variable=self.var_gimbal, command=self._toggle_gimbal).pack(anchor="w")
        
        self.lbl_gimbal_status = ttk.Label(left_frame, text="Disconnected")
        self.lbl_gimbal_status.pack(anchor="w", pady=2)
        
        self.lbl_gimbal_info = ttk.Label(left_frame, text="Mount: — | Mode: —")
        self.lbl_gimbal_info.pack(anchor="w")
        
        center_frame = ttk.Frame(main_container); center_frame.pack(side="left", padx=20)
        ttk.Button(center_frame, text="Center", command=self._center_gimbal).pack(pady=2)
        ttk.Label(center_frame, text="Speed:", font=("Arial", 8)).pack(pady=(10,0))
        self.speed_var = tk.IntVar(value=50)
        speed_scale = ttk.Scale(center_frame, from_=10, to=100, variable=self.speed_var,
                               orient="horizontal", length=100)
        speed_scale.pack()
        speed_label = ttk.Label(center_frame, text="50"); speed_label.pack()
        speed_scale.config(command=lambda v: speed_label.config(text=f"{int(float(v))}"))
        
        right_frame = ttk.Frame(main_container); right_frame.pack(side="right")
        yaw_frame = ttk.Frame(right_frame); yaw_frame.pack(pady=5)
        yaw_btn_frame = ttk.Frame(yaw_frame); yaw_btn_frame.pack()
        self._create_control_button(yaw_btn_frame, "< L", 
                                   lambda: (-self.speed_var.get(), 0)).pack(side="left", padx=2)
        self._create_control_button(yaw_btn_frame, "R >", 
                                   lambda: (self.speed_var.get(), 0)).pack(side="left", padx=2)
        
        pitch_frame = ttk.Frame(right_frame); pitch_frame.pack()
        self._create_control_button(pitch_frame, "^ Up", 
                                   lambda: (0, self.speed_var.get())).pack(pady=1)
        self._create_control_button(pitch_frame, "v Down", 
                                   lambda: (0, -self.speed_var.get())).pack(pady=1)
    
    def _create_target_panel(self):
        target_frame = ttk.LabelFrame(self.root, text="Target Calculation")
        target_frame.pack(padx=10, pady=5, fill="x")
        
        self.txt_target = tk.Text(target_frame, height=6, wrap=tk.WORD, 
                                 font=("Consolas", 9), relief="flat", borderwidth=0)
        self.txt_target.pack(fill="x", padx=5, pady=5)
    
    def _create_tracking_panel(self):
        tracking_frame = ttk.LabelFrame(self.root, text="Dynamic Loiter Tracking")
        tracking_frame.pack(padx=10, pady=5, fill="x")
        
        params_frame = ttk.Frame(tracking_frame); params_frame.pack(fill="x", padx=5, pady=5)
        
        radius_frame = ttk.Frame(params_frame); radius_frame.pack(fill="x", pady=3)
        ttk.Label(radius_frame, text="Loiter Radius:", width=15).pack(side="left")
        self.radius_var = tk.DoubleVar(value=Config.DEFAULT_LOITER_RADIUS)
        radius_scale = ttk.Scale(radius_frame, from_=300.0, to=4000.0, variable=self.radius_var,
                                orient="horizontal", length=200)
        radius_scale.pack(side="left", padx=5)
        self.lbl_radius = ttk.Label(radius_frame, text=f"{Config.DEFAULT_LOITER_RADIUS:.0f}m")
        self.lbl_radius.pack(side="left", padx=5)
        radius_scale.config(command=lambda v: self.lbl_radius.config(text=f"{float(v):.0f}m"))
        
        update_frame = ttk.Frame(params_frame); update_frame.pack(fill="x", pady=3)
        ttk.Label(update_frame, text="Update Interval:", width=15).pack(side="left")
        self.update_var = tk.DoubleVar(value=Config.TRACKING_UPDATE_S)
        update_scale = ttk.Scale(update_frame, from_=0.5, to=5.0, variable=self.update_var,
                                orient="horizontal", length=200)
        update_scale.pack(side="left", padx=5)
        self.lbl_update = ttk.Label(update_frame, text=f"{Config.TRACKING_UPDATE_S:.1f}s")
        self.lbl_update.pack(side="left", padx=5)
        update_scale.config(command=lambda v: self.lbl_update.config(text=f"{float(v):.1f}s"))
        
        movement_frame = ttk.Frame(params_frame); movement_frame.pack(fill="x", pady=3)
        ttk.Label(movement_frame, text="Min Movement:", width=15).pack(side="left")
        self.movement_var = tk.DoubleVar(value=Config.MIN_MOVEMENT_THRESHOLD)
        movement_scale = ttk.Scale(movement_frame, from_=10.0, to=200.0, variable=self.movement_var,
                                  orient="horizontal", length=200)
        movement_scale.pack(side="left", padx=5)
        self.lbl_movement = ttk.Label(movement_frame, text=f"{Config.MIN_MOVEMENT_THRESHOLD:.0f}m")
        self.lbl_movement.pack(side="left", padx=5)
        movement_scale.config(command=lambda v: self.lbl_movement.config(text=f"{float(v):.0f}m"))
        
        btn_frame = ttk.Frame(tracking_frame); btn_frame.pack(pady=10)
        self.btn_start_tracking = ttk.Button(btn_frame, text="Start Dynamic Tracking", 
                                           command=self._start_tracking)
        self.btn_start_tracking.pack(side="left", padx=5)
        self.btn_stop_tracking = ttk.Button(btn_frame, text="Stop Tracking", 
                                          command=self._stop_tracking, state="disabled")
        self.btn_stop_tracking.pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Single GoTo", command=self._single_goto).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Return to Mission", command=self._return_to_mission).pack(side="left", padx=5)
    
    def _create_controls_panel(self):
        controls_frame = ttk.LabelFrame(self.root, text="System Controls")
        controls_frame.pack(padx=10, pady=5, fill="x")
        
        controls_container = ttk.Frame(controls_frame); controls_container.pack(fill="x", padx=5, pady=5)
        
        perf_frame = ttk.Frame(controls_container); perf_frame.pack(side="left")
        ttk.Label(perf_frame, text="Calc Rate:").pack(side="left")
        self.calc_rate_var = tk.DoubleVar(value=10.0)
        calc_scale = ttk.Scale(perf_frame, from_=1.0, to=20.0, variable=self.calc_rate_var,
                              orient="horizontal", length=150, command=self._update_calc_rate)
        calc_scale.pack(side="left", padx=5)
        ttk.Label(perf_frame, text="Hz").pack(side="left")
        
        sbs_frame = ttk.Frame(controls_container); sbs_frame.pack(side="right")
        self.var_sbs = tk.BooleanVar()
        ttk.Checkbutton(sbs_frame, text=f"SBS Publisher ({Config.SBS_BIND}:{Config.SBS_PORT})",
                       variable=self.var_sbs, command=self._toggle_sbs).pack()
    
    # =============================================================================
    # SETTINGS HOOKS
    # =============================================================================

    def _open_settings(self):
        SettingsDialog(self.root, self._apply_settings)

    def _apply_settings(self, new_cfg: Dict[str, Any]):
        """Re-create or reconnect subsystems to new addresses/ports."""
        try:
            # Recreate SIYI with new IP/port
            was_on = bool(self.var_gimbal.get())
            if was_on:
                try: self.gimbal.stop()
                except: pass
            self.gimbal = SiyiGimbal(Config.SIYI_IP, Config.SIYI_PORT)
            if was_on:
                self.var_gimbal.set(True)
                self.gimbal.start()

            # Update MAVLink (RX/TX)
            self.mavlink.set_connection_strings(Config.MAVLINK_ADDRESS, Config.MAVLINK_TX_ADDRESS or None)

            # Restart SBS if active
            if self.var_sbs.get():
                try: self.sbs.stop()
                except: pass
                self.sbs = SBSPublisher(Config.SBS_BIND, Config.SBS_PORT)
                self.sbs.start()
            else:
                self.sbs = SBSPublisher(Config.SBS_BIND, Config.SBS_PORT)

            # Update labels
            self.lbl_telemetry.config(text=f"RX: {Config.MAVLINK_ADDRESS} | TX: {Config.MAVLINK_TX_ADDRESS or '(same link)'} | Waiting for telemetry...")
        except Exception as e:
            messagebox.showerror("Error", f"Error updating subsystems: {e}")
    
    # =============================================================================
    # EVENT HANDLERS
    # =============================================================================
    
    def _toggle_gimbal(self):
        if self.var_gimbal.get():
            if not self.gimbal.start():
                self.var_gimbal.set(False)
                messagebox.showerror("Error", f"Could not connect to gimbal ({Config.SIYI_IP}:{Config.SIYI_PORT})")
        else:
            self.gimbal.stop()
    
    def _request_gimbal_attitude(self):
        if self.var_gimbal.get() and not self._closing:
            self.gimbal.request_attitude()
            self.root.after(Config.ATTITUDE_REQUEST_MS, self._request_gimbal_attitude)
    
    def _center_gimbal(self):
        if self.gimbal.is_connected:
            self.gimbal.center()
        else:
            tk.messagebox.showerror("Error", "Connect gimbal first")
    
    def _create_control_button(self, parent, text, speed_func):
        btn = ttk.Button(parent, text=text, width=6)
        moving = {"active": False}
        def start_movement():
            moving["active"] = True
            def move():
                if moving["active"] and self.gimbal.is_connected:
                    yaw_speed, pitch_speed = speed_func()
                    self.gimbal.jog(yaw_speed, pitch_speed)
                    self.root.after(50, move)
            move()
        def stop_movement():
            moving["active"] = False
            if self.gimbal.is_connected:
                self.gimbal.jog(0, 0)
        btn.bind("<Button-1>", lambda e: start_movement())
        btn.bind("<ButtonRelease-1>", lambda e: stop_movement())
        return btn
    
    def _start_tracking(self):
        if self.target_state['lat'] is None:
            tk.messagebox.showerror("Error", "No target calculated - point gimbal downward first")
            return
        if not self.mavlink.connected:
            tk.messagebox.showerror("Error", "No MAVLink connection")
            return
        try:
            radius = self.radius_var.get()
            interval = self.update_var.get()
            movement = self.movement_var.get()
        except (ValueError, AttributeError):
            tk.messagebox.showerror("Error", "Invalid parameter values")
            return
        if self.tracker.start_tracking(
            self.target_state['lat'], self.target_state['lon'],
            self.aircraft_state['alt_amsl'], radius, interval, movement
        ):
            self.btn_start_tracking.configure(state="disabled")
            self.btn_stop_tracking.configure(state="normal")
        else:
            tk.messagebox.showerror("Error", "Failed to start tracking")
    
    def _stop_tracking(self):
        self.tracker.stop_tracking()
        self.btn_start_tracking.configure(state="normal")
        self.btn_stop_tracking.configure(state="disabled")
    
    def _single_goto(self):
        if self.target_state['lat'] is None:
            tk.messagebox.showerror("Error", "No target calculated")
            return
        if not self.mavlink.connected:
            tk.messagebox.showerror("Error", "No MAVLink connection")
            return
        try:
            radius = max(50.0, min(5000.0, float(self.radius_var.get())))
        except ValueError:
            radius = Config.DEFAULT_LOITER_RADIUS
        self.mavlink.set_loiter_mode(
            self.target_state['lat'], self.target_state['lon'],
            self.aircraft_state['alt_amsl'], radius
        )
    
    def _return_to_mission(self):
        if self.mavlink.set_mission_mode():
            self.tracker.stop_tracking()
            self.btn_start_tracking.configure(state="normal")
            self.btn_stop_tracking.configure(state="disabled")
        else:
            tk.messagebox.showerror("Error", "Mission mode command failed")
    
    def _toggle_sbs(self):
        if self.var_sbs.get():
            if not self.sbs.start():
                tk.messagebox.showerror("Error", "Failed to start SBS publisher")
                self.var_sbs.set(False)
        else:
            self.sbs.stop()
    
    def _update_calc_rate(self, value):
        Config.CALC_THROTTLE_S = 1.0 / float(value)
    
    # =============================================================================
    # CORE LOGIC
    # =============================================================================
    
    def _calculate_target(self):
        current_time = time.time()
        if current_time - self.last_calc_time < Config.CALC_THROTTLE_S:
            return self.target_state.get('calculation_result')
        
        pitch, yaw = self.gimbal.get_corrected_angles(self.aircraft_state['heading'])
        if pitch is None or yaw is None:
            self.target_state['calculation_result'] = None
            return None
        
        if self.last_angles[0] is not None:
            pitch_change = abs(pitch - self.last_angles[0])
            yaw_change = abs(yaw - self.last_angles[1])
            if yaw_change > 180: yaw_change = 360 - yaw_change
            if (pitch_change < Config.ANGLE_CHANGE_THRESHOLD and 
                yaw_change < Config.ANGLE_CHANGE_THRESHOLD):
                return self.target_state.get('calculation_result')
        
        self.last_calc_time = current_time
        self.last_angles = (pitch, yaw)
        
        result = TargetCalculator.calculate_target(
            self.aircraft_state['lat'], self.aircraft_state['lon'],
            self.aircraft_state['alt_agl'], pitch, yaw
        )
        if result:
            self.target_state['lat'] = result['lat']
            self.target_state['lon'] = result['lon']
            self.target_state['distance'] = result['distance']
        else:
            self.target_state['lat'] = None
            self.target_state['lon'] = None
            self.target_state['distance'] = 0.0
        self.target_state['calculation_result'] = result
        return result
    
    def _update_displays(self):
        if self.mavlink.connected:
            self.lbl_telemetry.config(
                text=f"RX: {Config.MAVLINK_ADDRESS} | TX: {Config.MAVLINK_TX_ADDRESS or '(same link)'} | "
                     f"{self.aircraft_state['lat']:.6f}, {self.aircraft_state['lon']:.6f} | "
                     f"Alt: {self.aircraft_state['alt_agl']:.1f}m | Hdg: {self.aircraft_state['heading']:.1f}°"
            )
        
        if self.gimbal.is_connected:
            pitch, yaw = self.gimbal.get_corrected_angles(self.aircraft_state['heading'])
            raw_yaw = self.gimbal.yaw_abs or 0
            age = time.time() - self.gimbal.last_update
            status = f"P:{pitch:.1f}° Y:{yaw:.1f}° (Raw:{raw_yaw:.1f}°+AC:{self.aircraft_state['heading']:.1f}°) ({age:.1f}s)"
            color = "green" if age < 1.0 else "orange"
            self.lbl_gimbal_status.config(text=status, foreground=color)
            mount_info = f"Mount: {self.gimbal.mount_dir or '—'} | Mode: {self.gimbal.motion_mode or '—'}"
            self.lbl_gimbal_info.config(text=mount_info)
        else:
            self.lbl_gimbal_status.config(text=f"Disconnected ({Config.SIYI_IP}:{Config.SIYI_PORT})", foreground="red")
            self.lbl_gimbal_info.config(text="Mount: — | Mode: —")
        
        result = self._calculate_target()
        self._update_target_display(result)
    
    def _update_target_display(self, result):
        self.txt_target.delete(1.0, tk.END)
        if result:
            info = f"Target: {result['lat']:.7f}, {result['lon']:.7f}\n"
            info += f"Distance: {result['distance']:.0f}m | {result['note']}\n"
            info += f"Gimbal: P:{result['pitch']:.1f}° Y:{result['yaw']:.1f}°\n"
            if self.tracker.active:
                stats = self.tracker.get_stats()
                info += f"TRACKING: {stats['duration']:.0f}s | {stats['updates']} updates\n"
                info += f"Next update: {stats['next_update']:.1f}s\n"
            else:
                info += "Ready for tracking\n"
        else:
            info = "No target - Connect gimbal and point downward\n"
            info += f"AC: {self.aircraft_state['lat']:.6f}, {self.aircraft_state['lon']:.6f}\n"
            info += f"Alt: {self.aircraft_state['alt_agl']:.1f}m | Hdg: {self.aircraft_state['heading']:.1f}°\n"
            if self.tracker.active:
                info += "TRACKING ACTIVE - waiting for target...\n"
        self.txt_target.insert(1.0, info)
    
    # =============================================================================
    # BACKGROUND WORKERS & MAIN LOOP
    # =============================================================================
    
    def _start_workers(self):
        threading.Thread(target=self._sbs_worker, daemon=True).start()
        # also start periodic gimbal attitude poller
        self.root.after(Config.ATTITUDE_REQUEST_MS, self._request_gimbal_attitude)
    
    def _sbs_worker(self):
        while not self._closing:
            try:
                if (self.var_sbs.get() and self.target_state['lat'] is not None):
                    self.sbs.publish(
                        self.target_state['lat'],
                        self.target_state['lon'],
                        self.aircraft_state['alt_amsl']
                    )
                time.sleep(Config.SBS_UPDATE_S)
            except Exception:
                pass
    
    def _main_loop(self):
        if self._closing:
            return
        try:
            if self.mavlink.connected:
                position = self.mavlink.get_position()
                if position:
                    self.aircraft_state.update(position)
                heading = self.mavlink.get_attitude()
                if heading is not None:
                    self.aircraft_state['heading'] = heading
            if (self.tracker.active and self.target_state['lat'] is not None):
                self.tracker.update_target(
                    self.target_state['lat'],
                    self.target_state['lon'],
                    self.aircraft_state['alt_amsl']
                )
            self._update_displays()
        except Exception:
            pass
        self.root.after(Config.GUI_UPDATE_MS, self._main_loop)
    
    def _on_close(self):
        self._closing = True
        if self.manual_window and self.manual_window.window.winfo_exists():
            self.manual_window.window.destroy()
        self.tracker.cleanup()
        try:
            self.gimbal.stop()
        except Exception:
            pass
        try:
            self.sbs.stop()
        except Exception:
            pass
        if getattr(self.gimbal, "is_connected", False):
            try:
                self.gimbal.jog(0, 0)
            except Exception:
                pass
        self.root.destroy()
    
    def run(self):
        self.root.mainloop()

# =============================================================================
# MAIN
# =============================================================================

if __name__ == "__main__":
    try:
        app = GimbalGPSApp()
        app.run()
    except KeyboardInterrupt:
        print("Application interrupted")
    except Exception as e:
        print(f"Application error: {e}")
        import traceback
        traceback.print_exc()
