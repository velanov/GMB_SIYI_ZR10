# Google Earth Pro Integration Setup Instructions

## Overview
This guide explains how to set up Google Earth Pro to work with your gimbal targeting system. You'll create waypoints (flags) in Google Earth Pro, export them as KML files, and your Python application will automatically read these coordinates for gimbal targeting.

## Step 1: Install Google Earth Pro

1. **Download Google Earth Pro** (free):
   - Visit: https://www.google.com/earth/versions/#earth-pro
   - Download and install Google Earth Pro for your operating system
   - Launch Google Earth Pro

2. **Sign in (optional but recommended)**:
   - Sign in with your Google account for sync capabilities

## Step 2: Create Target Waypoints in Google Earth Pro

### Method 1: Manual Placemark Creation

1. **Navigate to your area of interest**:
   - Use the search box to find your location
   - Zoom to the appropriate level for your mission area

2. **Add Placemarks (Flags)**:
   - Click the **Placemark icon** in the toolbar (📍) or press `Ctrl+Shift+P`
   - A dialog will open with placemark properties
   - **Name**: Enter a descriptive name (e.g., "Target 1", "Building Alpha")
   - **Description**: Add any additional information
   - **Position the placemark**:
     - Drag the yellow pushpin to the exact location
     - OR manually enter coordinates in the dialog
   - Click **OK** to save

3. **Create multiple targets**:
   - Repeat the process for each target location
   - Each placemark will appear in the "Places" panel on the left

### Method 2: Import GPS Coordinates

1. **Prepare coordinate data**:
   - Create a simple text file with coordinates (CSV format)
   - Format: `Name, Latitude, Longitude, Altitude (optional)`

2. **Import coordinates**:
   - Go to `File > Import`
   - Select your coordinate file
   - Choose "Delimited" format and set delimiter to comma
   - Map columns appropriately
   - Click "OK"

## Step 3: Organize Targets

1. **Create a folder for your targets**:
   - In the "Places" panel, right-click "My Places"
   - Select "Add" > "Folder"
   - Name it "Gimbal Targets" or similar

2. **Move placemarks to folder**:
   - Drag all your target placemarks into this folder
   - This keeps them organized and makes exporting easier

## Step 4: Export KML File

1. **Select your target folder**:
   - Right-click on your "Gimbal Targets" folder in Places panel
   - Select "Save Place As..."

2. **Save as KML**:
   - Choose location: `C:\Users\Amara\Gimbal\GMB_SIYI_ZR10\`
   - Filename: `gimbal_targets.kml` (or any name you prefer)
   - File type: Select "Kml (*.kml)"
   - Click "Save"

## Step 5: Configure Python Application

1. **Update your Python application**:
   ```python
   # Add this to your gimbal_gps_ui_v2.py imports
   from gimbal_google_earth_integration_example import GimbalGoogleEarthPanel
   
   # In your GimbalGPSAppV2.__init__() method, add:
   self._create_google_earth_panel()
   
   def _create_google_earth_panel(self):
       """Create Google Earth integration panel"""
       self.ge_panel = GimbalGoogleEarthPanel(self.root, self)
   ```

2. **Run your application**:
   - Launch your gimbal application
   - You'll see a new "Google Earth Integration" panel
   - Click "Load Targets" and browse to your KML file

## Step 6: Using the Integration

### Loading Targets
1. Click **"Browse"** to select your KML file
2. Click **"Load Targets"** to read the waypoints
3. Status should show "Connected" in green

### Auto-Refresh (Real-time Updates)
1. Click **"Start Auto-Refresh"** to monitor KML file changes
2. Now you can add/modify waypoints in Google Earth Pro
3. Save the KML file again (overwrite existing)
4. Your Python app will automatically detect changes and reload targets

### Target Navigation
1. Use **"Next ►"** and **"◄ Previous"** buttons to cycle through targets
2. Current target coordinates are automatically set in your gimbal system
3. The target will switch to "fixed" mode and start tracking

## Step 7: Workflow for Mission Planning

### Before Flight
1. Open Google Earth Pro
2. Plan your mission route and identify targets
3. Create placemarks for each target
4. Export as KML file to your gimbal application directory

### During Flight
1. Launch your gimbal application
2. Load the KML file with targets
3. Enable auto-refresh for real-time updates
4. Use Next/Previous buttons to switch between targets
5. Each target switch will automatically:
   - Update gimbal coordinates
   - Switch to fixed target mode
   - Begin loitering around the target

### Real-time Target Updates
1. Keep Google Earth Pro open during flight
2. Add new targets or modify existing ones
3. Save the KML file (Ctrl+S or File > Save)
4. Your application automatically detects changes
5. New targets become immediately available

## Advanced Tips

### Creating Precise Coordinates
1. **Use ruler tool** for distance measurements
2. **Right-click any location** > "What's here?" for exact coordinates
3. **Use historical imagery** to plan based on different time periods

### Altitude Settings
1. Placemarks can include altitude information
2. Set altitude in placemark properties dialog
3. Choose ground-relative or absolute altitude

### Multiple Target Sets
1. Create different folders for different missions
2. Export separate KML files for different scenarios
3. Switch between KML files during operations

## Troubleshooting

### KML File Not Loading
- Check file path is correct
- Ensure KML file is valid (try opening in Google Earth)
- Verify file permissions (readable by Python application)

### Targets Not Appearing
- Check that placemarks contain valid coordinates
- Ensure placemarks are not in subfolders within subfolders
- Verify coordinate format (latitude, longitude, altitude)

### Auto-Refresh Not Working
- Ensure KML file path hasn't changed
- Check that Google Earth Pro is saving to the same location
- Restart auto-refresh if file location changed

## Sample KML Structure

Your exported KML file will look like this:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>Gimbal Targets</name>
    <Placemark>
      <name>Target 1</name>
      <description>Primary building</description>
      <Point>
        <coordinates>-122.4194,37.7749,100</coordinates>
      </Point>
    </Placemark>
    <Placemark>
      <name>Target 2</name>
      <description>Secondary location</description>
      <Point>
        <coordinates>-122.4094,37.7849,150</coordinates>
      </Point>
    </Placemark>
  </Document>
</kml>
```

## Quick Start Checklist

- [ ] Install Google Earth Pro
- [ ] Create target placemarks
- [ ] Organize in folder
- [ ] Export as KML file
- [ ] Configure Python application
- [ ] Load targets in gimbal app
- [ ] Test target switching
- [ ] Enable auto-refresh for real-time updates

## Support

If you encounter issues:
1. Check the console output in your Python application for error messages
2. Verify KML file format by opening in Google Earth Pro
3. Test with the sample KML file first (use "Create Sample" button)
4. Ensure all file paths are correct and accessible