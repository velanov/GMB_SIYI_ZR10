"""
SRTM Elevation Service

Provides elevation data from SRTM (Shuttle Radar Topography Mission) tiles.
Supports offline operation using local .hgt files.
"""

import os
import struct
import math
from typing import Optional

class OfflineSRTMService:
    """Offline SRTM elevation service using local .hgt files"""
    
    def __init__(self, data_dir: str = "dem_data"):
        """
        Initialize SRTM service with local data directory.
        
        Args:
            data_dir: Directory containing .hgt files (default: "dem_data")
        """
        # Get absolute path relative to project root
        if not os.path.isabs(data_dir):
            # Assume data_dir is relative to project root
            project_root = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
            self.data_dir = os.path.join(project_root, data_dir)
        else:
            self.data_dir = data_dir
        
        self.tile_cache = {}  # Cache for loaded tiles
        self.tile_size = 1201  # SRTM3 tile size (1 arc-second resolution)
        
        # Scan for available tiles
        self.available_tiles = self._scan_available_tiles()
        print(f"[SRTM] Initialized with {len(self.available_tiles)} available tiles")
        if self.available_tiles:
            print(f"[SRTM] Available tiles: {', '.join(sorted(self.available_tiles))}")
    
    def _scan_available_tiles(self) -> set:
        """Scan data directory for available .hgt files"""
        tiles = set()
        if not os.path.exists(self.data_dir):
            print(f"[SRTM] Data directory not found: {self.data_dir}")
            return tiles
        
        try:
            for filename in os.listdir(self.data_dir):
                if filename.endswith('.hgt') and len(filename) == 11:  # e.g., N47E008.hgt
                    tile_name = filename[:-4]  # Remove .hgt extension
                    tiles.add(tile_name)
        except Exception as e:
            print(f"[SRTM] Error scanning data directory: {e}")
        
        return tiles
    
    def _get_tile_name(self, lat: float, lon: float) -> str:
        """Get SRTM tile name for given coordinates"""
        # SRTM tiles are named like N47E008.hgt for lat=47.x, lon=8.x
        lat_int = int(math.floor(lat))
        lon_int = int(math.floor(lon))
        
        lat_str = f"N{lat_int:02d}" if lat_int >= 0 else f"S{abs(lat_int):02d}"
        lon_str = f"E{lon_int:03d}" if lon_int >= 0 else f"W{abs(lon_int):03d}"
        
        return f"{lat_str}{lon_str}"
    
    def _load_tile(self, tile_name: str) -> Optional[bytes]:
        """Load SRTM tile data from .hgt file"""
        if tile_name in self.tile_cache:
            return self.tile_cache[tile_name]
        
        if tile_name not in self.available_tiles:
            print(f"[SRTM] Tile not available: {tile_name}")
            return None
        
        tile_path = os.path.join(self.data_dir, f"{tile_name}.hgt")
        
        try:
            with open(tile_path, 'rb') as f:
                tile_data = f.read()
            
            # Verify tile size (SRTM3 should be 1201x1201 = 2,884,802 bytes)
            expected_size = self.tile_size * self.tile_size * 2  # 2 bytes per elevation point
            if len(tile_data) != expected_size:
                print(f"[SRTM] Warning: {tile_name} has unexpected size {len(tile_data)}, expected {expected_size}")
            
            self.tile_cache[tile_name] = tile_data
            print(f"[SRTM] Loaded tile: {tile_name}")
            return tile_data
            
        except Exception as e:
            print(f"[SRTM] Error loading tile {tile_name}: {e}")
            return None
    
    def get_elevation(self, lat: float, lon: float) -> float:
        """
        Get elevation at given coordinates.
        
        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            
        Returns:
            Elevation in meters above sea level (0.0 if no data available)
        """
        try:
            tile_name = self._get_tile_name(lat, lon)
            tile_data = self._load_tile(tile_name)
            
            if tile_data is None:
                # print(f"[SRTM] No elevation data for {lat:.6f}, {lon:.6f} (tile: {tile_name})")
                return 0.0
            
            # Calculate position within tile
            lat_int = int(math.floor(lat))
            lon_int = int(math.floor(lon))
            
            # Fractional part within the 1x1 degree tile
            lat_frac = lat - lat_int
            lon_frac = lon - lon_int
            
            # Convert to pixel coordinates (0-1200 for SRTM3)
            # Note: SRTM data is stored with (0,0) at top-left (north-west corner)
            row = int((1.0 - lat_frac) * (self.tile_size - 1))  # Flip Y axis
            col = int(lon_frac * (self.tile_size - 1))
            
            # Bounds check
            row = max(0, min(self.tile_size - 1, row))
            col = max(0, min(self.tile_size - 1, col))
            
            # Calculate byte offset (2 bytes per sample, big-endian)
            offset = (row * self.tile_size + col) * 2
            
            if offset + 1 >= len(tile_data):
                print(f"[SRTM] Offset out of bounds: {offset} >= {len(tile_data)}")
                return 0.0
            
            # Read elevation value (big-endian signed 16-bit integer)
            elevation_bytes = tile_data[offset:offset + 2]
            elevation = struct.unpack('>h', elevation_bytes)[0]  # >h = big-endian signed short
            
            # Handle void data (SRTM uses -32768 for no data)
            if elevation == -32768:
                return 0.0
            
            return float(elevation)
            
        except Exception as e:
            print(f"[SRTM] Error getting elevation for {lat:.6f}, {lon:.6f}: {e}")
            return 0.0
    
    def is_tile_available(self, lat: float, lon: float) -> bool:
        """Check if SRTM tile is available for given coordinates"""
        tile_name = self._get_tile_name(lat, lon)
        return tile_name in self.available_tiles
    
    def get_coverage_info(self) -> str:
        """Get information about available tile coverage"""
        if not self.available_tiles:
            return "No SRTM tiles available"
        
        info = f"SRTM Coverage: {len(self.available_tiles)} tiles\n"
        info += f"Data directory: {self.data_dir}\n"
        info += f"Available tiles: {', '.join(sorted(self.available_tiles))}"
        
        return info