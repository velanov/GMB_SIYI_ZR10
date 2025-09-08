# Enhanced Gimbal GPS UI

This enhanced version implements Phase 1 of the gimbal aircraft features roadmap, with full 3D coordinate support.

## What's New in Phase 1

### ✅ 3D Coordinate Support
- **Enhanced TargetCalculator**: Now calculates full 3D coordinates (lat, lon, altitude)
- **Altitude calculations**: Supports both AGL (Above Ground Level) and AMSL (Above Mean Sea Level)
- **3D distance**: Calculates both 2D horizontal distance and true 3D distance
- **Improved UI**: Target display now shows complete 3D information

### Key Features Implemented

1. **Extended calculate_target() method**:
   - Added `aircraft_alt_amsl` parameter for AMSL calculations
   - Returns altitude data: `alt` (AGL), `alt_amsl` (AMSL)
   - Calculates `3d_distance` including vertical component
   - Improved sky/horizon altitude estimation

2. **Enhanced target state management**:
   - Target state now stores 3D coordinates: `lat`, `lon`, `alt`, `alt_amsl`
   - Tracks both 2D and 3D distances: `distance`, `3d_distance`

3. **Improved UI display**:
   - Shows target altitude in both AGL and AMSL
   - Displays both 2D and 3D distances
   - Enhanced status information

## Testing

Run the test script to see the 3D coordinate calculations in action:

```bash
python3 test_3d_coordinates.py
```

## Usage

Run the enhanced gimbal GPS UI:

```bash
python3 gimbal_gps_ui_enhanced.py
```

The interface will now display 3D target information including:
- Target coordinates with altitude
- Both 2D horizontal and 3D direct distances
- Altitude in both AGL and AMSL formats

## Files

- `gimbal_gps_ui_enhanced.py` - Main enhanced application
- `ROADMAP.md` - Complete roadmap for all planned features
- `test_3d_coordinates.py` - Phase 1 testing script
- `gimbal_gps_settings.json` - Settings file (copied from original)
- `requirements.txt` - Dependencies (copied from original)

## Next Steps

See `ROADMAP.md` for the complete implementation plan. The next phases will include:
- Phase 2: Gimbal lock & aircraft adjustment commands
- Phase 3: Gimbal-compass calibration
- Phase 4: Google Earth integration
- Phase 5: PX4 mode analysis (Offboard vs GoTo)