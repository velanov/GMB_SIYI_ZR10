"""
Waypoint manager for multi-target tracking and navigation.
"""

from typing import List, Optional, Callable
from enum import Enum
from dataclasses import dataclass
from .kml_parser import Waypoint
import threading
import time


class TrackingMode(Enum):
    """Different tracking modes for waypoint navigation."""
    SEQUENTIAL = "sequential"  # Go through waypoints in order
    MANUAL_SELECT = "manual_select"  # Operator manually selects targets
    LOITER_ALL = "loiter_all"  # Loiter at each waypoint
    CLOSEST_FIRST = "closest_first"  # Navigate to closest waypoint first


@dataclass
class WaypointStatus:
    """Status information for a waypoint."""
    waypoint: Waypoint
    visited: bool = False
    currently_tracking: bool = False
    visit_count: int = 0
    last_visited: Optional[float] = None  # timestamp


class WaypointManager:
    """Manages multiple waypoints for tracking missions."""
    
    def __init__(self):
        self._waypoints: List[WaypointStatus] = []
        self._current_index: int = -1
        self._tracking_mode: TrackingMode = TrackingMode.SEQUENTIAL
        self._is_active: bool = False
        self._lock = threading.Lock()
        
        # Callbacks
        self._on_waypoint_changed: Optional[Callable[[Waypoint], None]] = None
        self._on_mission_completed: Optional[Callable[[], None]] = None
    
    def load_waypoints_from_file(self, file_path: str) -> int:
        """
        Load waypoints from KML/KMZ file.
        
        Args:
            file_path: Path to the KML/KMZ file
            
        Returns:
            Number of waypoints loaded
        """
        from .kml_parser import KMLParser
        
        parser = KMLParser()
        waypoints = parser.parse_file(file_path)
        
        with self._lock:
            self._waypoints = [WaypointStatus(wp) for wp in waypoints]
            self._current_index = -1
        
        return len(waypoints)
    
    def add_waypoint(self, waypoint: Waypoint) -> None:
        """Add a single waypoint to the manager."""
        with self._lock:
            self._waypoints.append(WaypointStatus(waypoint))
    
    def add_waypoints(self, waypoints: List[Waypoint]) -> None:
        """Add multiple waypoints to the manager."""
        with self._lock:
            for wp in waypoints:
                self._waypoints.append(WaypointStatus(wp))
    
    def clear_waypoints(self) -> None:
        """Clear all waypoints."""
        with self._lock:
            self._waypoints.clear()
            self._current_index = -1
            self._is_active = False
    
    def set_tracking_mode(self, mode: TrackingMode) -> None:
        """Set the tracking mode."""
        with self._lock:
            self._tracking_mode = mode
    
    def get_waypoints(self) -> List[WaypointStatus]:
        """Get all waypoints with their status."""
        with self._lock:
            return self._waypoints.copy()
    
    def get_current_waypoint(self) -> Optional[Waypoint]:
        """Get the currently active waypoint."""
        with self._lock:
            return self._get_current_waypoint_unlocked()
    
    def _get_current_waypoint_unlocked(self) -> Optional[Waypoint]:
        """Get the currently active waypoint without acquiring lock."""
        if 0 <= self._current_index < len(self._waypoints):
            return self._waypoints[self._current_index].waypoint
        return None
    
    def get_current_coordinates(self) -> Optional[tuple]:
        """Get current waypoint coordinates as (lat, lon, alt)."""
        with self._lock:
            current = self._get_current_waypoint_unlocked()
            if current:
                return (current.latitude, current.longitude, current.altitude)
            return None
    
    def get_current_index(self) -> int:
        """Get current waypoint index."""
        with self._lock:
            return self._current_index
    
    def start_mission(self) -> bool:
        """
        Start the waypoint mission.
        
        Returns:
            True if mission started successfully
        """
        with self._lock:
            if not self._waypoints:
                return False
            
            self._is_active = True
            if self._tracking_mode == TrackingMode.SEQUENTIAL:
                self._current_index = 0
            elif self._tracking_mode == TrackingMode.CLOSEST_FIRST:
                # For now, start with first waypoint
                # TODO: Implement closest waypoint calculation based on current position
                self._current_index = 0
            
            if self._current_index >= 0:
                self._waypoints[self._current_index].currently_tracking = True
                if self._on_waypoint_changed:
                    self._on_waypoint_changed(self._waypoints[self._current_index].waypoint)
        
        return True
    
    def stop_mission(self) -> None:
        """Stop the current mission."""
        with self._lock:
            self._is_active = False
            if 0 <= self._current_index < len(self._waypoints):
                self._waypoints[self._current_index].currently_tracking = False
    
    def next_waypoint(self) -> Optional[Waypoint]:
        """
        Move to the next waypoint based on tracking mode.
        
        Returns:
            Next waypoint if available, None if mission completed
        """
        with self._lock:
            if not self._is_active or not self._waypoints:
                return None
            
            # Mark current waypoint as visited
            if 0 <= self._current_index < len(self._waypoints):
                current_status = self._waypoints[self._current_index]
                current_status.currently_tracking = False
                current_status.visited = True
                current_status.visit_count += 1
                current_status.last_visited = time.time()
            
            # Determine next waypoint based on mode
            next_index = self._get_next_waypoint_index()
            
            if next_index is None:
                # Mission completed
                self._is_active = False
                if self._on_mission_completed:
                    self._on_mission_completed()
                return None
            
            self._current_index = next_index
            next_status = self._waypoints[next_index]
            next_status.currently_tracking = True
            
            if self._on_waypoint_changed:
                self._on_waypoint_changed(next_status.waypoint)
            
            return next_status.waypoint
    
    def select_waypoint(self, index: int) -> Optional[Waypoint]:
        """
        Manually select a waypoint by index.
        
        Args:
            index: Index of the waypoint to select
            
        Returns:
            Selected waypoint if valid index
        """
        with self._lock:
            if not (0 <= index < len(self._waypoints)):
                return None
            
            # Mark current as not tracking
            if 0 <= self._current_index < len(self._waypoints):
                self._waypoints[self._current_index].currently_tracking = False
            
            # Select new waypoint
            self._current_index = index
            self._waypoints[index].currently_tracking = True
            self._is_active = True
            
            if self._on_waypoint_changed:
                self._on_waypoint_changed(self._waypoints[index].waypoint)
            
            return self._waypoints[index].waypoint
    
    def _get_next_waypoint_index(self) -> Optional[int]:
        """Get the next waypoint index based on tracking mode."""
        if self._tracking_mode == TrackingMode.SEQUENTIAL:
            next_idx = self._current_index + 1
            if next_idx < len(self._waypoints):
                return next_idx
        
        elif self._tracking_mode == TrackingMode.LOITER_ALL:
            # In loiter mode, stay at current waypoint
            return self._current_index
        
        elif self._tracking_mode == TrackingMode.MANUAL_SELECT:
            # Manual mode requires explicit selection
            return None
        
        # Mission completed
        return None
    
    def set_waypoint_changed_callback(self, callback: Callable[[Waypoint], None]) -> None:
        """Set callback for when waypoint changes."""
        self._on_waypoint_changed = callback
    
    def set_mission_completed_callback(self, callback: Callable[[], None]) -> None:
        """Set callback for when mission completes."""
        self._on_mission_completed = callback
    
    def get_mission_status(self) -> dict:
        """Get comprehensive mission status."""
        with self._lock:
            total_waypoints = len(self._waypoints)
            visited_waypoints = sum(1 for wp in self._waypoints if wp.visited)
            
            return {
                'is_active': self._is_active,
                'tracking_mode': self._tracking_mode.value,
                'total_waypoints': total_waypoints,
                'visited_waypoints': visited_waypoints,
                'current_index': self._current_index,
                'current_waypoint': self._get_current_waypoint_unlocked(),
                'progress_percentage': (visited_waypoints / total_waypoints * 100) if total_waypoints > 0 else 0
            }
    
    def is_mission_active(self) -> bool:
        """Check if mission is currently active."""
        with self._lock:
            return self._is_active
    
    def get_waypoint_count(self) -> int:
        """Get total number of waypoints."""
        with self._lock:
            return len(self._waypoints)