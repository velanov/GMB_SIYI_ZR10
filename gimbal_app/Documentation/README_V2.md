# Gimbal GPS UI V2 - Fixed Coordinate Target Mode

## Overview

This is an enhanced version of the original `gimbal_gps_ui.py` that adds a **Fixed Coordinate Target Mode** alongside the existing gimbal tracking functionality. The original gimbal mode is preserved unchanged as an example/reference.

## Key Features

### 🆕 New: Fixed Coordinate Mode
- **Manual target input**: Set precise GPS coordinates (latitude, longitude, altitude)
- **Static targeting**: Target stays fixed regardless of gimbal movement  
- **Same tracking logic**: Uses identical DynamicTracker for consistent behavior
- **Mission integration**: Full support for loiter and mission mode switching

### 🔄 Dual Mode Operation
- **Gimbal Mode**: Calculate target from where gimbal is pointing (original behavior)
- **Fixed Mode**: Use manually entered GPS coordinates as target
- **Easy switching**: Radio buttons to switch between modes
- **Independent operation**: Each mode works completely independently

### 🎯 Identical Tracking System
- **Same DynamicTracker**: Both modes use the same tracking implementation
- **Same parameters**: Loiter radius, update interval, movement threshold
- **Same commands**: Identical MAVLink commands sent to aircraft
- **Same mission switching**: Return to mission works identically for both modes

## Usage

### 1. Start the Application
```bash
python3 gimbal_gps_ui_v2.py
```

### 2. Gimbal Mode (Original)
1. Select "Gimbal Pointing Mode" 
2. Connect SIYI gimbal
3. Point gimbal at target location
4. Target coordinates calculated automatically
5. Start tracking to loiter around calculated target

### 3. Fixed Coordinate Mode (New)
1. Select "Fixed Coordinate Mode"
2. Enter target coordinates:
   - Latitude (e.g., 47.3980000)
   - Longitude (e.g., 8.5470000) 
   - Altitude (e.g., 0 for ground level)
3. Click "Set Fixed Target"
4. Start tracking to loiter around fixed target

### 4. Additional Features
- **Use Aircraft Position**: Quickly set current aircraft location as fixed target
- **Dynamic updates**: Gimbal mode updates as gimbal moves; fixed mode stays constant
- **Mission return**: Both modes support returning to pre-planned mission

## Technical Implementation

### Mode Architecture
- **Mode-agnostic tracking**: DynamicTracker doesn't know/care about coordinate source
- **Clean separation**: Gimbal calculations separate from fixed coordinate input
- **Unified interface**: Both modes provide coordinates to same tracking system

### Tracking Logic Preservation
The original tracking logic is completely preserved:
- Same loiter commands (`MAV_CMD_DO_SET_MODE`, `NAV_LOITER_RAD`, `MAV_CMD_DO_REPOSITION`)
- Same mission commands (`MAV_CMD_DO_SET_MODE` to Mission)
- Same parameter handling (radius, update interval, movement threshold)
- Same error handling and reconnection logic

### Coordinate Sources
```
Gimbal Mode: Gimbal Angles → TargetCalculator → Coordinates → DynamicTracker
Fixed Mode:  User Input → Validation → Coordinates → DynamicTracker
```

## Files

- `gimbal_gps_ui.py` - Original version (kept unchanged as example)
- `gimbal_gps_ui_v2.py` - Enhanced version with fixed coordinate mode
- `test_tracking_modes.py` - Tests both tracking modes
- `test_mission_switching.py` - Tests mission mode switching
- `README_V2.md` - This documentation

## Testing

Run the included tests to verify functionality:

```bash
# Test both tracking modes
python3 test_tracking_modes.py

# Test mission mode switching  
python3 test_mission_switching.py
```

## Compatibility

- **Same settings**: Uses same settings file format and network configuration
- **Same dependencies**: No additional Python packages required
- **Same MAVLink**: Compatible with same PX4/ArduPilot systems
- **Same QGC integration**: Works with QGroundControl forwarding setup

## Roadmap Integration

This implementation satisfies the increment requested while preserving the roadmap path:
- ✅ **Fixed coordinate target mode** (this implementation)
- 🔄 **3D coordinates** (already documented in roadmap)
- 🔄 **Gimbal lock commands** (Phase 2 of roadmap)
- 🔄 **Compass calibration** (Phase 3 of roadmap)
- 🔄 **Google Earth export** (Phase 4 of roadmap)

The V2 implementation is fully compatible with all future roadmap phases.