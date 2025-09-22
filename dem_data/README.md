# SRTM Elevation Data

This directory contains SRTM (Shuttle Radar Topography Mission) elevation tiles in .hgt format.

## Required Files for Switzerland Area

For the gimbal system to work with accurate terrain elevation in the Switzerland area, you need these SRTM tiles:

- **N46E007.hgt** - Covers area 46-47°N, 7-8°E
- **N46E008.hgt** - Covers area 46-47°N, 8-9°E  
- **N47E007.hgt** - Covers area 47-48°N, 7-8°E
- **N47E008.hgt** - Covers area 47-48°N, 8-9°E (Main Zurich area)

## Download SRTM Tiles

You can download SRTM tiles from:
1. **NASA Earthdata**: https://search.earthdata.nasa.gov/
2. **USGS EarthExplorer**: https://earthexplorer.usgs.gov/
3. **SRTM 1-arc-second (30m resolution)** is recommended

## File Format

- Files must be named exactly like: `N47E008.hgt`
- SRTM3 format: 1201x1201 pixels
- File size: ~2.88 MB per tile
- Data format: Big-endian signed 16-bit integers

## Usage

The gimbal application will automatically:
1. Scan this directory for available tiles
2. Load elevation data for target calculations
3. Use real terrain elevation instead of assuming ground level (0m)

Without these tiles, the system defaults to sea level (0m) for all targets.