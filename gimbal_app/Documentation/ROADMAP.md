# Roadmap for New Gimbal–Aircraft Features

## 1. Coordinate Mode (gimbal pointing → coordinate)

**Where in code:** Extend TargetCalculator (currently computes lat/lon from pitch/yaw).

**Steps:**
- Add support for returning altitude in calculate_target (currently only lat/lon)
- Store results in self.target_state inside GimbalGPSApp._calculate_target
- Update target panel (_update_target_display) to show 3D coordinates

✅ **Result:** Whenever gimbal is pointed, you get a full 3D coordinate (lat, lon, alt).

## 2. Coordinate → Gimbal lock & Aircraft adjust

**Where in code:** Add new method to MAVLinkHandler for "look at" command.

**Steps:**
- Create MAVLinkHandler.point_gimbal(lat, lon, alt) using MAV_CMD_DO_SET_ROI_LOCATION
- Add UI button "Lock Gimbal on Coordinate"
- In handler, after ROI command, continuously fetch gimbal attitude and send to PX4
- Integrate with DynamicTracker to loiter around that point

✅ **Result:** If you click on a coordinate or select "lock", the gimbal points, airplane loiters.

## 3. Gimbal–Compass Calibration

**Where in code:** In SiyiGimbal.get_corrected_angles.

**Steps:**
- Add calibration offset parameter (stored in JSON via SettingsStore)
- Apply offset before returning corrected yaw
- Provide "Calibrate Compass" button in UI to sync gimbal yaw with aircraft heading (get from MAVLink attitude)

✅ **Result:** Gimbal yaw aligns with PX4 heading consistently.

## 4. Full 3D Coordinates

**Where in code:** TargetCalculator.calculate_target.

**Steps:**
- Extend return dict to include altitude of intersection (currently only ground intersection)
- Modify UI (_update_target_display) to show lat, lon, alt
- Update SBS publisher to broadcast 3D positions (lat, lon, alt)

✅ **Result:** Targets aren't just 2D — altitude is included.

## 5. Google Earth Integration

**Where in code:** New module googleearth_export.py.

**Steps:**
- Use simplekml to generate KML placemarks with aircraft & target positions
- Create periodic exporter thread (similar to _sbs_worker)
- Optionally add live network link (KML refresh URL → Flask/HTTP server)
- Add UI toggle "Export to Google Earth"

✅ **Result:** See aircraft + gimbal target in real-time on Google Earth.

## 6. PX4 Mode Analysis (Offboard vs GoTo)

**Where in code:** MAVLinkHandler.

**Steps:**
- Implement Offboard mode support (setpoints stream at 20Hz with SET_POSITION_TARGET_GLOBAL_INT)
- Keep current set_loiter_mode (GoTo/Reposition)
- Add config option in UI "Use Offboard / Use GoTo"
- Benchmark both in SITL:
  - GoTo/Loiter → simpler, less bandwidth, autopilot handles loiter
  - Offboard → more precise, but mission computer must stream continuously

✅ **Result:** Operator can choose mode depending on mission.

## Implementation Phases

### Phase 1 – Core math & data flow
- Extend TargetCalculator to handle altitude
- Update GimbalGPSApp to store/display full 3D target

### Phase 2 – Commanding the gimbal/aircraft
- Add MAV_CMD_DO_SET_ROI_LOCATION command in MAVLinkHandler
- Add "Lock on target" UI
- Integrate with DynamicTracker for loiter

### Phase 3 – Calibration
- Add calibration offset setting in JSON
- UI button "Calibrate compass"

### Phase 4 – Visualization
- Create KML exporter for Google Earth
- Test with SITL flights

### Phase 5 – Mode support
- Implement Offboard setpoint streaming
- Add UI option to switch between GoTo vs Offboard