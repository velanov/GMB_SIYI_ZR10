#!/usr/bin/env python3
"""
Test script for SRTM elevation service
"""

import sys
import os

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from gimbal_app.elevation import OfflineSRTMService
from gimbal_app.calc.target_calculator import TargetCalculator

def test_elevation_service():
    """Test the SRTM elevation service"""
    print("="*50)
    print("SRTM Elevation Service Test")
    print("="*50)
    
    # Initialize SRTM service
    srtm = OfflineSRTMService()
    print(f"\nSRTM Service Info:")
    print(srtm.get_coverage_info())
    
    # Test coordinates in Switzerland area
    test_points = [
        (47.4085, 8.5490, "Zurich Airport area"),
        (47.3769, 8.5417, "Zurich city center"),
        (46.9481, 7.4474, "Bern area"),
        (46.2044, 6.1432, "Geneva area"),
    ]
    
    print(f"\nTesting elevation at key locations:")
    print("-" * 50)
    
    for lat, lon, description in test_points:
        elevation = srtm.get_elevation(lat, lon)
        tile_available = srtm.is_tile_available(lat, lon)
        tile_name = srtm._get_tile_name(lat, lon)
        
        print(f"{description}:")
        print(f"  Location: {lat:.4f}°N, {lon:.4f}°E")
        print(f"  Tile: {tile_name} ({'Available' if tile_available else 'Missing'})")
        print(f"  Elevation: {elevation:.1f}m")
        print()
    
    # Test target calculator integration
    print("Testing Target Calculator Integration:")
    print("-" * 50)
    
    calculator = TargetCalculator()
    
    # Test a typical gimbal pointing scenario
    aircraft_lat = 47.4085
    aircraft_lon = 8.5490
    aircraft_alt_agl = 100.0  # 100m above ground
    pitch_deg = -30.0  # Looking down 30 degrees (negative pitch = down)
    yaw_deg = 0.0     # Pointing north
    
    result = calculator.calculate_target(
        aircraft_lat, aircraft_lon, aircraft_alt_agl,
        pitch_deg, yaw_deg
    )
    
    if result:
        print(f"Aircraft position: {aircraft_lat:.6f}°N, {aircraft_lon:.6f}°E @ {aircraft_alt_agl}m AGL")
        print(f"Gimbal angles: Pitch={pitch_deg}°, Yaw={yaw_deg}°")
        print(f"Target coordinates: {result['lat']:.6f}°N, {result['lon']:.6f}°E")
        print(f"Target elevation: {result['alt']:.1f}m")
        print(f"Target distance: {result['distance']:.1f}m")
        print(f"Calculation note: {result.get('note', 'N/A')}")
        
        if 'iterations' in result:
            print(f"3D iterations: {result['iterations']}")
            print(f"Convergence error: {result['error_m']:.3f}m")
            print(f"Processing time: {result['processing_time_ms']:.1f}ms")
    else:
        print("Target calculation failed!")
    
    print("\n" + "="*50)
    print("Test completed!")

if __name__ == "__main__":
    test_elevation_service()