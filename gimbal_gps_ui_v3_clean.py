"""
gimbal_gps_ui_v3_clean.py
Clean version using modular architecture
- Based on original gimbal_gps_ui.py structure
- Uses gimbal_fixed_target_system.py for core functionality
- Uses gimbal_target_ui_components.py for UI
- Integrated with google_earth_integration.py
"""

import tkinter as tk
from tkinter import ttk, messagebox
import math, time, threading, json, os
from pymavlink import mavutil
import socket, struct
from typing import Optional, Tuple, Dict, Any

# Import modular components
from gimbal_fixed_target_system import FixedTargetSystem, TargetCalculator, calculate_target
from gimbal_target_ui_components import IntegratedTargetUI
from google_earth_integration import GimbalGoogleEarthInterface

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
# UTILITY FUNCTIONS (from original)
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
# CORE SYSTEM CLASSES (from original)
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
    """MAVLink communication handler with auto-reconnect and dual RX/TX links."""
    
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

# =============================================================================
# SETTINGS DIALOG (from original)
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
# MANUAL CONTROL WINDOW (from original)
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
# GOOGLE EARTH UI PANEL
# =============================================================================

class GoogleEarthPanel:
    """UI Panel for Google Earth integration"""
    
    def __init__(self, parent_frame, target_system: FixedTargetSystem):
        self.target_system = target_system
        self.ge_interface = GimbalGoogleEarthInterface(target_system)
        
        # Create the Google Earth panel
        self.frame = ttk.LabelFrame(parent_frame, text="Google Earth Integration")
        self.frame.pack(padx=10, pady=5, fill="x")
        
        self._create_widgets()
        self.update_display()
        
        # Start periodic updates
        self._schedule_update()
    
    def _create_widgets(self):
        """Create the Google Earth integration widgets"""
        
        # File selection row
        file_row = ttk.Frame(self.frame)
        file_row.pack(fill="x", padx=5, pady=5)
        
        ttk.Label(file_row, text="KML File:").pack(side="left")
        self.kml_file_var = tk.StringVar(value="gimbal_targets.kml")
        self.kml_entry = ttk.Entry(file_row, textvariable=self.kml_file_var, width=40)
        self.kml_entry.pack(side="left", padx=5, fill="x", expand=True)
        
        ttk.Button(file_row, text="Browse", command=self._browse_kml_file).pack(side="right", padx=2)
        ttk.Button(file_row, text="Create Sample", command=self._create_sample_kml).pack(side="right", padx=2)
        
        # Control buttons row
        control_row = ttk.Frame(self.frame)
        control_row.pack(fill="x", padx=5, pady=5)
        
        ttk.Button(control_row, text="Load Targets", command=self._load_targets).pack(side="left", padx=2)
        ttk.Button(control_row, text="Start Auto-Refresh", command=self._start_auto_refresh).pack(side="left", padx=2)
        ttk.Button(control_row, text="Stop Auto-Refresh", command=self._stop_auto_refresh).pack(side="left", padx=2)
        
        # Target navigation row
        nav_row = ttk.Frame(self.frame)
        nav_row.pack(fill="x", padx=5, pady=5)
        
        ttk.Button(nav_row, text="◄ Previous", command=self._previous_target).pack(side="left", padx=2)
        ttk.Button(nav_row, text="Next ►", command=self._next_target).pack(side="left", padx=2)
        
        # Target info display
        info_row = ttk.Frame(self.frame)
        info_row.pack(fill="x", padx=5, pady=5)
        
        self.target_info_var = tk.StringVar(value="No targets loaded")
        ttk.Label(info_row, textvariable=self.target_info_var, relief="sunken", anchor="w").pack(fill="x")
        
        # Current target details
        details_row = ttk.Frame(self.frame)
        details_row.pack(fill="x", padx=5, pady=5)
        
        self.target_details_var = tk.StringVar(value="")
        ttk.Label(details_row, textvariable=self.target_details_var, relief="sunken", anchor="w").pack(fill="x")
        
        # Status display
        status_row = ttk.Frame(self.frame)
        status_row.pack(fill="x", padx=5, pady=5)
        
        ttk.Label(status_row, text="Status:").pack(side="left")
        self.status_var = tk.StringVar(value="Not connected")
        self.status_label = ttk.Label(status_row, textvariable=self.status_var, foreground="red")
        self.status_label.pack(side="left", padx=5)
    
    def _browse_kml_file(self):
        """Browse for KML file"""
        from tkinter import filedialog
        filename = filedialog.askopenfilename(
            title="Select KML File",
            filetypes=[("KML files", "*.kml"), ("All files", "*.*")]
        )
        if filename:
            self.kml_file_var.set(filename)
    
    def _create_sample_kml(self):
        """Create a sample KML file"""
        from tkinter import filedialog
        from google_earth_integration import GoogleEarthIntegration
        
        filename = filedialog.asksaveasfilename(
            title="Create Sample KML File",
            defaultextension=".kml",
            filetypes=[("KML files", "*.kml"), ("All files", "*.*")]
        )
        if filename:
            ge = GoogleEarthIntegration()
            if ge.create_sample_kml(filename):
                self.kml_file_var.set(filename)
                messagebox.showinfo("Success", f"Sample KML file created: {filename}")
            else:
                messagebox.showerror("Error", "Failed to create sample KML file")
    
    def _load_targets(self):
        """Load targets from KML file"""
        kml_file = self.kml_file_var.get()
        if not kml_file:
            messagebox.showerror("Error", "Please specify a KML file")
            return
        
        if self.ge_interface.setup(kml_file, auto_refresh=False):
            self.status_var.set("Connected")
            self.status_label.config(foreground="green")
            messagebox.showinfo("Success", "Targets loaded successfully")
        else:
            self.status_var.set("Failed to load")
            self.status_label.config(foreground="red")
            messagebox.showerror("Error", f"Failed to load KML file: {kml_file}")
        
        self.update_display()
    
    def _start_auto_refresh(self):
        """Start auto-refresh of KML file"""
        if not self.ge_interface.active:
            kml_file = self.kml_file_var.get()
            if not kml_file:
                messagebox.showerror("Error", "Please load targets first")
                return
            
            if self.ge_interface.setup(kml_file, auto_refresh=True):
                self.status_var.set("Auto-refresh ON")
                self.status_label.config(foreground="blue")
            else:
                messagebox.showerror("Error", "Failed to start auto-refresh")
        else:
            self.ge_interface.ge_integration.start_auto_refresh()
            self.status_var.set("Auto-refresh ON")
            self.status_label.config(foreground="blue")
    
    def _stop_auto_refresh(self):
        """Stop auto-refresh of KML file"""
        self.ge_interface.ge_integration.stop_auto_refresh()
        if self.ge_interface.active:
            self.status_var.set("Connected")
            self.status_label.config(foreground="green")
        else:
            self.status_var.set("Not connected")
            self.status_label.config(foreground="red")
    
    def _next_target(self):
        """Move to next target"""
        if not self.ge_interface.active:
            messagebox.showerror("Error", "No targets loaded")
            return
        
        self.ge_interface.next_target()
        self.update_display()
    
    def _previous_target(self):
        """Move to previous target"""
        if not self.ge_interface.active:
            messagebox.showerror("Error", "No targets loaded")
            return
        
        self.ge_interface.previous_target()
        self.update_display()
    
    def update_display(self):
        """Update the display with current target info"""
        if not self.ge_interface.active:
            self.target_info_var.set("No targets loaded")
            self.target_details_var.set("")
            return
        
        info = self.ge_interface.get_current_target_info()
        
        if info['has_targets']:
            self.target_info_var.set(
                f"Target {info['current_index'] + 1} of {info['total_targets']}"
            )
            
            if info['current_target']:
                target = info['current_target']
                self.target_details_var.set(
                    f"{target.name}: {target.lat:.6f}, {target.lon:.6f}, {target.alt:.1f}m"
                )
            else:
                self.target_details_var.set("No current target")
        else:
            self.target_info_var.set("No targets in KML file")
            self.target_details_var.set("")
    
    def _schedule_update(self):
        """Schedule periodic updates"""
        self.update_display()
        # Schedule next update
        if hasattr(self, 'frame') and self.frame.winfo_exists():
            self.frame.after(1000, self._schedule_update)
    
    def shutdown(self):
        """Shutdown the Google Earth interface"""
        self.ge_interface.shutdown()

# =============================================================================
# MAIN APPLICATION GUI
# =============================================================================

class GimbalGPSAppV3:
    """Clean main application with modular architecture"""
    
    def __init__(self):
        # Load persisted settings before creating systems
        SettingsStore.load_into()

        # Initialize core systems with (possibly) loaded config
        self.gimbal = SiyiGimbal(Config.SIYI_IP, Config.SIYI_PORT)
        self.sbs = SBSPublisher(Config.SBS_BIND, Config.SBS_PORT)
        self.mavlink = MAVLinkHandler(Config.MAVLINK_ADDRESS, Config.MAVLINK_TX_ADDRESS or None)
        
        # Initialize fixed target system (NEW - modular)
        self.target_system = FixedTargetSystem(self.gimbal, self.mavlink)
        
        # Application state
        self.aircraft_state = {
            'lat': 47.3977508, 'lon': 8.5455938,
            'alt_amsl': 500.0, 'alt_agl': 100.0, 'heading': 0.0
        }
        self.ui_settings = {
            'calc_rate_hz': 10.0,
            'angle_threshold': 0.5
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
        self.root.title("Gimbal GPS - Fixed Target System v3")
        self.root.geometry("720x800")
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
        
        # NEW - Modular target UI
        self.target_ui = IntegratedTargetUI(self.root, self.target_system, self.mavlink)
        
        # NEW - Google Earth integration
        self.google_earth_panel = GoogleEarthPanel(self.root, self.target_system)
        
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
        """Calculate gimbal target (original logic)"""
        current_time = time.time()
        if current_time - self.last_calc_time < Config.CALC_THROTTLE_S:
            return self.target_system.target_manager.gimbal_target_state.get('calculation_result')
        
        pitch, yaw = self.gimbal.get_corrected_angles(self.aircraft_state['heading'])
        if pitch is None or yaw is None:
            return None
        
        if self.last_angles[0] is not None:
            pitch_change = abs(pitch - self.last_angles[0])
            yaw_change = abs(yaw - self.last_angles[1])
            if yaw_change > 180: yaw_change = 360 - yaw_change
            if (pitch_change < Config.ANGLE_CHANGE_THRESHOLD and 
                yaw_change < Config.ANGLE_CHANGE_THRESHOLD):
                return self.target_system.target_manager.gimbal_target_state.get('calculation_result')
        
        self.last_calc_time = current_time
        self.last_angles = (pitch, yaw)
        
        # Use original calculation function for compatibility
        result = calculate_target(
            self.aircraft_state['lat'], self.aircraft_state['lon'],
            self.aircraft_state['alt_agl'], pitch, yaw
        )
        
        if result:
            # Update target system
            self.target_system.target_manager.update_gimbal_target(
                result['lat'], result['lon'], result['distance'], result
            )
        
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
        
        # Calculate gimbal target
        self._calculate_target()
    
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
                if self.var_sbs.get():
                    target_lat, target_lon, target_alt = self.target_system.target_manager.get_current_target()
                    if target_lat is not None:
                        self.sbs.publish(
                            target_lat, target_lon, self.aircraft_state['alt_amsl']
                        )
                time.sleep(Config.SBS_UPDATE_S)
            except Exception:
                pass
    
    def _main_loop(self):
        if self._closing:
            return
        try:
            # Update aircraft state
            if self.mavlink.connected:
                position = self.mavlink.get_position()
                if position:
                    self.aircraft_state.update(position)
                heading = self.mavlink.get_attitude()
                if heading is not None:
                    self.aircraft_state['heading'] = heading
            
            # Update target system with aircraft state
            self.target_system.update_aircraft_state(self.aircraft_state)
            
            # Update target UI
            if hasattr(self, 'target_ui'):
                self.target_ui.update_aircraft_state(self.aircraft_state)
            
            self._update_displays()
        except Exception:
            pass
        self.root.after(Config.GUI_UPDATE_MS, self._main_loop)
    
    def _on_close(self):
        self._closing = True
        if self.manual_window and self.manual_window.window.winfo_exists():
            self.manual_window.window.destroy()
        
        # Cleanup modular components
        if hasattr(self, 'target_ui'):
            self.target_ui.cleanup()
        if hasattr(self, 'google_earth_panel'):
            self.google_earth_panel.shutdown()
        if hasattr(self, 'target_system'):
            self.target_system.cleanup()
            
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
        app = GimbalGPSAppV3()
        app.run()
    except KeyboardInterrupt:
        print("Application interrupted")
    except Exception as e:
        print(f"Application error: {e}")
        import traceback
        traceback.print_exc()