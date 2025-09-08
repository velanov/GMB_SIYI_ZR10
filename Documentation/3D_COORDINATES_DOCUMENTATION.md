# 3D Coordinate Calculation Documentation

## Overview

This document explains the mathematical formulas and algorithms used to calculate 3D target coordinates from gimbal pointing angles and aircraft position.

## Problem Statement

Given:
- Aircraft position: `(lat_aircraft, lon_aircraft, alt_aircraft)`
- Gimbal angles: `(pitch, yaw)` in degrees
- Aircraft heading: `heading` in degrees

Calculate:
- Target 3D position: `(lat_target, lon_target, alt_target)`
- 2D and 3D distances to target

## Coordinate Systems

### 1. Geographic Coordinate System
- **Latitude**: North-South position (-90° to +90°)
- **Longitude**: East-West position (-180° to +180°)  
- **Altitude AMSL**: Above Mean Sea Level (meters)
- **Altitude AGL**: Above Ground Level (meters)

### 2. NED Coordinate System (North-East-Down)
- **North (N)**: Positive north direction (meters)
- **East (E)**: Positive east direction (meters) 
- **Down (D)**: Positive downward direction (meters)

Used for local calculations relative to aircraft position.

### 3. Gimbal Coordinate System
- **Pitch**: Vertical angle (-90° up, 0° horizon, +90° down)
- **Yaw**: Horizontal angle (0° north, 90° east, 180° south, 270° west)
- **Roll**: Not used in this implementation

## Mathematical Formulas

### 1. Gimbal Angle Correction

Raw gimbal yaw is corrected for aircraft heading:
```
corrected_yaw = raw_gimbal_yaw + aircraft_heading
```

### 2. 3D Ray Vector Calculation

From gimbal angles, calculate the 3D pointing vector:

```
pitch_rad = pitch_degrees × π/180
yaw_rad = corrected_yaw × π/180

# 3D unit vector components
vN = cos(pitch_rad) × cos(yaw_rad)    # North component
vE = cos(pitch_rad) × sin(yaw_rad)    # East component  
vD = sin(pitch_rad)                   # Down component
```

### 3. Ground Intersection Calculation

For ground pointing (pitch > 0°):

```
# Time parameter for ray intersection with ground
t = aircraft_altitude_AGL / vD

# Horizontal displacement in NED coordinates
dN = vN × t    # North displacement (meters)
dE = vE × t    # East displacement (meters)

# 2D horizontal distance
distance_2D = √(dN² + dE²)
```

### 4. Sky Pointing Calculation

For upward pointing (pitch < 0°):

```
# Use fixed maximum distance (5km)
distance = 5000 meters

# Horizontal displacement
dN = cos(yaw_rad) × distance
dE = sin(yaw_rad) × distance

# Vertical displacement
vertical_offset = distance × tan(|pitch_rad|)
target_altitude_AGL = aircraft_altitude_AGL + vertical_offset
```

### 5. NED to Geographic Conversion

Convert NED offsets to geographic coordinates:

```
# Earth radius (WGS84)
R = 6,378,137 meters

# Convert to angular displacements
dlat_rad = dN / R
dlon_rad = dE / (R × cos(lat_aircraft_rad))

# Final target coordinates
lat_target = lat_aircraft + (dlat_rad × 180/π)
lon_target = lon_aircraft + (dlon_rad × 180/π)
```

### 6. Altitude Calculations

#### Ground Targets
```
target_altitude_AGL = 0 meters                     # Ground level
target_altitude_AMSL = aircraft_AMSL - aircraft_AGL  # Ground level in AMSL
```

#### Sky Targets  
```
target_altitude_AGL = aircraft_AGL + vertical_offset
target_altitude_AMSL = target_altitude_AGL + ground_level_AMSL
```

### 7. 3D Distance Calculation

```
# Horizontal distance (already calculated)
distance_2D = √(dN² + dE²)

# Vertical distance
vertical_distance = |aircraft_altitude_AGL - target_altitude_AGL|

# True 3D distance
distance_3D = √(distance_2D² + vertical_distance²)
```

## Implementation Details

### Target Scenarios

1. **Ground Targets (pitch > 0°)**
   - Calculate ray intersection with ground plane
   - Target altitude = 0m AGL
   - Distance limited to 5km maximum

2. **Sky Targets (pitch < 0°)**
   - Use fixed 5km horizontal distance
   - Calculate altitude at that distance
   - Target above aircraft level

3. **Horizon Targets (pitch = 0°)**
   - Use fixed 5km distance
   - Target at aircraft altitude level

### Edge Cases Handled

1. **Negative Aircraft Altitude**
   - Use 10m AGL fallback for calculations
   - Maintain actual altitude for distance calculations
   - Add warning notes to results

2. **Distance Limiting**
   - Maximum 5km distance to prevent unrealistic targets
   - Scale down calculations if exceeded
   - Add "capped" note to results

3. **Invalid Geometry**
   - Skip calculation if pointing nearly straight up (vD ≤ 1e-6)
   - Return null result for invalid scenarios

### Validation Layers

1. **Input Validation**
   - Coordinate range checking (-90° ≤ lat ≤ 90°)
   - Altitude reasonability checks
   - Angle normalization

2. **Output Validation**
   - ADS-B minimum altitude (35m AMSL)
   - Maximum altitude limits (15,000m AMSL) 
   - Coordinate bounds checking

## ADS-B 3D Trajectory

### Problem with Original Implementation
The original ADS-B implementation sent only the final target position, causing "jumps" on the map because QGC couldn't see the smooth transition from aircraft to target.

### Enhanced 3D Trajectory Solution

Instead of a single target point, we now send multiple intermediate points along the 3D trajectory:

```
# Generate N intermediate points from aircraft to target
for i in range(0, N+1):
    t = i / N  # Interpolation factor (0 to 1)
    
    # Linear interpolation in 3D space
    lat_i = lat_aircraft + t × (lat_target - lat_aircraft)
    lon_i = lon_aircraft + t × (lon_target - lon_aircraft)
    alt_i = alt_aircraft + t × (alt_target - alt_aircraft)
    
    # Send as separate ADS-B target with unique ID
    publish_adsb_point(lat_i, lon_i, alt_i, hex_id=f"TAR{i:02d}")
```

This creates a smooth trajectory visualization in QGC showing the complete 3D path from aircraft to target.

## Performance Optimizations

### Calculation Throttling
- Minimum 0.1s between calculations (`CALC_THROTTLE_S`)
- Skip calculations for minimal angle changes (< 0.1°)
- Force recalculation every 2 seconds regardless

### Angle Change Detection
```
pitch_change = |current_pitch - last_pitch|
yaw_change = |current_yaw - last_yaw|

# Handle yaw wraparound (360° → 0°)
if yaw_change > 180°:
    yaw_change = 360° - yaw_change

# Skip if both changes are minimal AND recent
skip = (pitch_change < 0.1° AND yaw_change < 0.1° AND time_since_last < 2s)
```

## Error Handling

### Robust Altitude Handling
- Negative altitudes use 10m fallback
- Extreme values are clamped to reasonable ranges
- Clear logging of all adjustments

### Connection Resilience  
- ADS-B connection recovery on errors
- MAVLink reconnection on failure
- Graceful degradation when systems unavailable

## Coordinate Transformation Summary

```
Input: Gimbal(pitch, yaw) + Aircraft(lat, lon, alt, heading)
  ↓
Step 1: Correct yaw for aircraft heading
  ↓  
Step 2: Convert to 3D ray vector (vN, vE, vD)
  ↓
Step 3: Calculate intersection/projection distance
  ↓
Step 4: Compute NED displacement (dN, dE)
  ↓
Step 5: Convert NED to geographic coordinates
  ↓
Step 6: Calculate target altitude (AGL/AMSL)
  ↓
Step 7: Compute 2D and 3D distances
  ↓
Output: Target(lat, lon, alt_AGL, alt_AMSL, distance_2D, distance_3D)
```

This mathematical foundation ensures accurate, robust 3D coordinate calculations for all gimbal pointing scenarios.