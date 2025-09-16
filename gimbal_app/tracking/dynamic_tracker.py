from ..shared import *
from ..calc.target_calculator import calculate_distance
from ..mavlink.handler import MAVLinkHandler

class DynamicTracker:
    """Dynamic loiter tracking system - works with both gimbal and fixed coordinates"""
    
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
    
    def update_radius(self, radius: float):
        """Update the loiter radius during tracking"""
        self.loiter_radius = radius
        if self.active and self.mavlink.connected:
            # Update the MAVLink loiter radius if currently tracking
            if hasattr(self, 'target_lat') and hasattr(self, 'target_lon') and hasattr(self, 'target_alt'):
                self.mavlink.set_loiter_mode(self.target_lat, self.target_lon, self.target_alt, radius)
    
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
