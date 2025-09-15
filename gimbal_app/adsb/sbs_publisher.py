from ..shared import *
from datetime import datetime

class SBSPublisher:
    """ADS-B SBS format publisher for QGroundControl"""
    
    def __init__(self, host: str = Config.SBS_BIND, port: int = Config.SBS_PORT, 
                 hexid: str = "ABCDEF", callsign: str = "TARGET"):
        self.host, self.port = host, port
        self.hexid = hexid.upper()  # Ensure uppercase hex ID
        self.callsign = callsign[:8].ljust(8)  # Pad callsign to 8 chars
        self._server = None
        self._connections = []  # Support multiple connections
        self._stop = False
        self._last_publish_time = 0
        self._publish_interval = 1.0  # Publish every second
        
        # Aircraft tracking state
        self._aircraft_id = 1
        self._session_id = 1
        self._flight_id = 1
        
        print(f"[SBS] Initialized with ICAO: {self.hexid}, Callsign: {self.callsign}")
    
    def start(self) -> bool:
        if self._server and not self._stop:
            return True
        
        # Ensure clean state before starting
        if self._server:
            self.stop()
        
        # Retry logic for address-in-use errors
        max_retries = 3
        for attempt in range(max_retries):
            try:
                self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)  # Additional reuse option
                self._server.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                self._server.bind((self.host, self.port))
                self._server.listen(5)  # Allow multiple connections
                self._stop = False
                threading.Thread(target=self._accept_loop, daemon=True).start()
                print(f"[SBS] Started SBS publisher on {self.host}:{self.port}")
                return True
            except OSError as e:
                if e.errno == 98:  # Address already in use
                    print(f"[SBS] Address in use, retry {attempt + 1}/{max_retries}")
                    # Clean up failed socket
                    if self._server:
                        try:
                            self._server.close()
                        except:
                            pass
                        self._server = None
                    
                    if attempt < max_retries - 1:
                        time.sleep(1.0)  # Wait before retry
                        continue
                raise e
            except Exception as e:
                print(f"[SBS] Failed to start SBS publisher: {e}")
                # Clean up on failure
                if self._server:
                    try:
                        self._server.close()
                    except:
                        pass
                    self._server = None
                return False
        
        print(f"[SBS] Failed to start after {max_retries} attempts")
        return False
    
    def stop(self):
        """Stop the SBS publisher"""
        print("[SBS] Stopping SBS publisher")
        self._stop = True
        
        # Close all client connections
        for conn in self._connections[:]:
            try:
                conn.close()
            except:
                pass
        self._connections.clear()
        
        # Close server socket with proper shutdown
        if self._server:
            try:
                # Shutdown the socket before closing
                self._server.shutdown(socket.SHUT_RDWR)
            except:
                pass
            try:
                self._server.close()
            except:
                pass
            self._server = None
        
        # Longer delay to ensure socket is fully released
        time.sleep(0.5)
        
        print("[SBS] SBS publisher stopped")
    
    def get_status(self) -> dict:
        """Get current SBS publisher status"""
        return {
            'running': self._server is not None,
            'host': self.host,
            'port': self.port,
            'clients_connected': len(self._connections),
            'icao_code': self.hexid,
            'callsign': self.callsign.strip(),
            'last_publish': self._last_publish_time
        }
    
    def publish(self, lat: float, lon: float, alt_amsl: float, 
                ground_speed: float = 0.0, track: float = 0.0) -> bool:
        """Publish target position in SBS-1 format"""
        current_time = time.time()
        
        # Throttle publishing to avoid spam
        if current_time - self._last_publish_time < self._publish_interval:
            return True
        
        if not self._connections:
            return False
            
        try:
            # Generate timestamps
            now = datetime.now()
            date_str = now.strftime("%Y/%m/%d")
            time_str = now.strftime("%H:%M:%S.%f")[:-3]  # Milliseconds
            timestamp = f"{date_str},{time_str}"
            
            # Convert units
            alt_ft = int(alt_amsl * 3.28084)  # Meters to feet
            speed_kt = int(ground_speed * 1.94384)  # m/s to knots
            track_deg = int(track) % 360
            
            # Create SBS messages - QGC needs multiple message types
            messages = []
            
            # MSG,1: ES Identification and Category
            msg1 = (f"MSG,1,{self._session_id},{self._aircraft_id},{self.hexid},{self._flight_id},"
                   f"{timestamp},{timestamp},{self.callsign},,,,,,,,,,0")
            
            # MSG,3: ES Airborne Position Message
            msg3 = (f"MSG,3,{self._session_id},{self._aircraft_id},{self.hexid},{self._flight_id},"
                   f"{timestamp},{timestamp},{self.callsign},{alt_ft},0,0,{lat:.6f},{lon:.6f},"
                   f"0,0,0,0,0,0")
            
            # MSG,4: ES Airborne Velocity Message (important for QGC)
            msg4 = (f"MSG,4,{self._session_id},{self._aircraft_id},{self.hexid},{self._flight_id},"
                   f"{timestamp},{timestamp},{self.callsign},{speed_kt},{track_deg},0,,,0,0,0,0")
            
            # MSG,6: ES Surface Position Message (backup)
            msg6 = (f"MSG,6,{self._session_id},{self._aircraft_id},{self.hexid},{self._flight_id},"
                   f"{timestamp},{timestamp},{self.callsign},0,0,0,{lat:.6f},{lon:.6f},"
                   f"{speed_kt},{track_deg},0,0,0,0")
            
            # Combine all messages
            full_message = "\r\n".join([msg1, msg3, msg4, msg6]) + "\r\n"
            
            # Send to all connected clients
            active_connections = []
            for conn in self._connections[:]:
                try:
                    conn.sendall(full_message.encode("ascii"))
                    active_connections.append(conn)
                except (OSError, ConnectionError, BrokenPipeError):
                    try:
                        conn.close()
                    except:
                        pass
                    print("[SBS] Client disconnected")
            
            self._connections = active_connections
            self._last_publish_time = current_time
            
            if active_connections:
                print(f"[SBS] Published target: {lat:.6f},{lon:.6f} @ {alt_ft}ft to {len(active_connections)} clients")
            
            return len(active_connections) > 0
            
        except Exception as e:
            print(f"[SBS] Publish error: {e}")
            return False
    
    def _accept_loop(self):
        """Accept connections from QGC and other SBS clients"""
        print(f"[SBS] Listening for connections on {self.host}:{self.port}")
        
        while not self._stop:
            try:
                conn, addr = self._server.accept()
                print(f"[SBS] New client connected from {addr}")
                
                # Configure socket for better performance
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                
                self._connections.append(conn)
                
                # Clean up dead connections
                active_connections = []
                for c in self._connections:
                    try:
                        # Test if connection is still alive
                        c.getpeername()
                        active_connections.append(c)
                    except:
                        try:
                            c.close()
                        except:
                            pass
                
                self._connections = active_connections
                print(f"[SBS] Total active connections: {len(self._connections)}")
                
            except OSError:
                if not self._stop:
                    print("[SBS] Accept loop error")
                break
