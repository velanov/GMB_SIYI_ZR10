from ..shared import *

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
