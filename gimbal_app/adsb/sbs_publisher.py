from ..shared import *

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
