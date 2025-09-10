from ..shared import *

class TargetCalculator:
    """Target position calculation with gimbal angles and reverse calculation"""
    
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

    @staticmethod
    def calculate_gimbal_angles(aircraft_lat: float, aircraft_lon: float, aircraft_alt_agl: float,
                               aircraft_heading: float, target_lat: float, target_lon: float, 
                               target_alt: float = 0.0) -> Optional[Dict[str, Any]]:
        """Calculate gimbal angles needed to point at target coordinates"""
        if None in [aircraft_lat, aircraft_lon, target_lat, target_lon]:
            return None
        
        # Calculate distance and bearing to target
        distance_2d = calculate_distance(aircraft_lat, aircraft_lon, target_lat, target_lon)
        if distance_2d == 0:
            return None
        
        # Calculate bearing from aircraft to target (true north = 0°)
        lat1, lon1, lat2, lon2 = map(math.radians, [aircraft_lat, aircraft_lon, target_lat, target_lon])
        dlon = lon2 - lon1
        
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        bearing_rad = math.atan2(y, x)
        bearing_deg = math.degrees(bearing_rad) % 360.0
        
        # Calculate pitch angle (vertical angle to target)
        # Positive pitch = looking down, negative pitch = looking up
        altitude_diff = aircraft_alt_agl - target_alt  # Height above target
        pitch_rad = math.atan2(altitude_diff, distance_2d)
        pitch_deg = math.degrees(pitch_rad)
        
        # Calculate yaw angle relative to aircraft heading
        # Convert true bearing to relative bearing
        relative_bearing = (bearing_deg - aircraft_heading) % 360.0
        if relative_bearing > 180:
            relative_bearing -= 360.0
        
        # Convert to gimbal yaw (accounting for gimbal coordinate system)
        gimbal_yaw = relative_bearing
        
        return {
            'pitch': pitch_deg,
            'yaw': gimbal_yaw,
            'distance_2d': distance_2d,
            'bearing': bearing_deg,
            'relative_bearing': relative_bearing,
            'altitude_diff': altitude_diff
        }
