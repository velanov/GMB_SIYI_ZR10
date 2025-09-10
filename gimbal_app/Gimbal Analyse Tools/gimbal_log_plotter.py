#!/usr/bin/env python3
"""
Gimbal Log Plotter - Parse gimbal_log.txt and create visual analysis plots
"""

import re
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from datetime import datetime
import pandas as pd
import numpy as np

def parse_gimbal_log(log_file="gimbal_log.txt"):
    """Parse the gimbal log file and extract data for plotting"""
    
    data = {
        'timestamp': [],
        'required_yaw': [],
        'required_pitch': [],
        'current_yaw': [],
        'current_pitch': [],
        'yaw_speed': [],
        'pitch_speed': [],
        'distance': [],
        'aircraft_lat': [],
        'aircraft_lon': [],
        'aircraft_alt': [],
        'aircraft_heading': []
    }
    
    current_required_yaw = None
    current_required_pitch = None
    current_distance = None
    
    print("Parsing gimbal log file...")
    
    try:
        with open(log_file, 'r') as f:
            for line_num, line in enumerate(f, 1):
                try:
                    # Extract timestamp
                    timestamp_match = re.match(r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})', line)
                    if not timestamp_match:
                        continue
                    
                    timestamp_str = timestamp_match.group(1)
                    timestamp = datetime.strptime(timestamp_str, '%Y-%m-%d %H:%M:%S')
                    
                    # Parse ANGLES_CALC entries
                    if 'ANGLES_CALC' in line:
                        angles_match = re.search(r'Required: Y=(-?\d+\.?\d*)° P=(-?\d+\.?\d*)° \| Distance: (\d+\.?\d*)m', line)
                        if angles_match:
                            current_required_yaw = float(angles_match.group(1))
                            current_required_pitch = float(angles_match.group(2))
                            current_distance = float(angles_match.group(3))
                    
                    # Parse GIMBAL_CMD entries  
                    elif 'GIMBAL_CMD' in line:
                        cmd_match = re.search(r'Target: Y=(-?\d+\.?\d*)° P=(-?\d+\.?\d*)° \| Current: Y=(-?\d+\.?\d*)° P=(-?\d+\.?\d*)° \| Speed: yaw=(-?\d+) pitch=(-?\d+)', line)
                        if cmd_match:
                            data['timestamp'].append(timestamp)
                            data['required_yaw'].append(float(cmd_match.group(1)))
                            data['required_pitch'].append(float(cmd_match.group(2))) 
                            data['current_yaw'].append(float(cmd_match.group(3)))
                            data['current_pitch'].append(float(cmd_match.group(4)))
                            data['yaw_speed'].append(int(cmd_match.group(5)))
                            data['pitch_speed'].append(int(cmd_match.group(6)))
                            data['distance'].append(current_distance or 0)
                            # Fill aircraft data with last known values (will be updated below)
                            data['aircraft_lat'].append(0)
                            data['aircraft_lon'].append(0)
                            data['aircraft_alt'].append(0)
                            data['aircraft_heading'].append(0)
                    
                    # Parse AIRCRAFT entries
                    elif 'AIRCRAFT' in line:
                        aircraft_match = re.search(r'Lat: (-?\d+\.?\d*)° \| Lon: (-?\d+\.?\d*)° \| Alt: (-?\d+\.?\d*)m \| Heading: (-?\d+\.?\d*)°', line)
                        if aircraft_match and data['timestamp']:  # Update last entry
                            data['aircraft_lat'][-1] = float(aircraft_match.group(1))
                            data['aircraft_lon'][-1] = float(aircraft_match.group(2))
                            data['aircraft_alt'][-1] = float(aircraft_match.group(3))
                            data['aircraft_heading'][-1] = float(aircraft_match.group(4))
                
                except Exception as e:
                    print(f"Error parsing line {line_num}: {e}")
                    continue
    
    except FileNotFoundError:
        print(f"Error: {log_file} not found!")
        return None
    
    print(f"Parsed {len(data['timestamp'])} gimbal command entries")
    return data

def create_plots(data):
    """Create comprehensive gimbal performance plots"""
    
    if not data or len(data['timestamp']) == 0:
        print("No data to plot!")
        return
    
    # Convert to pandas DataFrame for easier handling
    df = pd.DataFrame(data)
    
    # Create figure with subplots
    fig, axes = plt.subplots(4, 1, figsize=(15, 12))
    fig.suptitle('Gimbal Performance Analysis', fontsize=16, fontweight='bold')
    
    # Plot 1: Yaw Tracking Performance
    axes[0].plot(df['timestamp'], df['required_yaw'], 'b-', label='Required Yaw', linewidth=2)
    axes[0].plot(df['timestamp'], df['current_yaw'], 'r--', label='Actual Yaw', linewidth=2)
    axes[0].set_ylabel('Yaw Angle (°)')
    axes[0].set_title('Yaw Tracking Performance')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    
    # Plot 2: Pitch Tracking Performance  
    axes[1].plot(df['timestamp'], df['required_pitch'], 'g-', label='Required Pitch', linewidth=2)
    axes[1].plot(df['timestamp'], df['current_pitch'], 'm--', label='Actual Pitch', linewidth=2)
    axes[1].set_ylabel('Pitch Angle (°)')
    axes[1].set_title('Pitch Tracking Performance')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    
    # Plot 3: Control Commands
    axes[2].plot(df['timestamp'], df['yaw_speed'], 'c-', label='Yaw Speed', linewidth=1.5)
    axes[2].plot(df['timestamp'], df['pitch_speed'], 'orange', label='Pitch Speed', linewidth=1.5)
    axes[2].set_ylabel('Speed Command')
    axes[2].set_title('Gimbal Control Commands')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)
    
    # Plot 4: Tracking Errors with proper yaw wraparound
    yaw_error_raw = np.array(df['required_yaw']) - np.array(df['current_yaw'])
    # Normalize yaw errors to [-180, +180] range
    yaw_error = np.copy(yaw_error_raw)
    yaw_error[yaw_error > 180] -= 360
    yaw_error[yaw_error < -180] += 360
    
    pitch_error = np.array(df['required_pitch']) - np.array(df['current_pitch'])
    
    axes[3].plot(df['timestamp'], yaw_error, 'b-', label='Yaw Error', linewidth=1.5)
    axes[3].plot(df['timestamp'], pitch_error, 'g-', label='Pitch Error', linewidth=1.5)
    axes[3].axhline(y=0, color='k', linestyle='-', alpha=0.3)
    axes[3].set_ylabel('Error (°)')
    axes[3].set_xlabel('Time')
    axes[3].set_title('Tracking Errors (Required - Actual)')
    axes[3].legend()
    axes[3].grid(True, alpha=0.3)
    
    # Format x-axis for all subplots
    for ax in axes:
        ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
        ax.xaxis.set_major_locator(mdates.MinuteLocator(interval=1))
        plt.setp(ax.xaxis.get_majorticklabels(), rotation=45)
    
    plt.tight_layout()
    plt.savefig('gimbal_performance.png', dpi=300, bbox_inches='tight')
    print("Plot saved as 'gimbal_performance.png'")
    plt.show()

def create_summary_stats(data):
    """Create summary statistics table"""
    
    if not data or len(data['timestamp']) == 0:
        print("No data for statistics!")
        return
    
    df = pd.DataFrame(data)
    
    # Calculate errors with proper yaw wraparound
    yaw_error_raw = np.array(df['required_yaw']) - np.array(df['current_yaw'])
    # Normalize yaw errors to [-180, +180] range
    yaw_error = np.copy(yaw_error_raw)
    yaw_error[yaw_error > 180] -= 360
    yaw_error[yaw_error < -180] += 360
    
    pitch_error = np.array(df['required_pitch']) - np.array(df['current_pitch'])
    
    print("\n" + "="*60)
    print("GIMBAL PERFORMANCE STATISTICS")
    print("="*60)
    
    print(f"Total Commands Analyzed: {len(df)}")
    print(f"Time Span: {df['timestamp'].iloc[0]} to {df['timestamp'].iloc[-1]}")
    
    print("\nYAW PERFORMANCE:")
    print(f"  Mean Error:      {np.mean(yaw_error):+7.2f}°")
    print(f"  RMS Error:       {np.sqrt(np.mean(yaw_error**2)):7.2f}°")
    print(f"  Max Error:       {np.max(np.abs(yaw_error)):7.2f}°")
    print(f"  Std Deviation:   {np.std(yaw_error):7.2f}°")
    
    print("\nPITCH PERFORMANCE:")
    print(f"  Mean Error:      {np.mean(pitch_error):+7.2f}°")
    print(f"  RMS Error:       {np.sqrt(np.mean(pitch_error**2)):7.2f}°")
    print(f"  Max Error:       {np.max(np.abs(pitch_error)):7.2f}°") 
    print(f"  Std Deviation:   {np.std(pitch_error):7.2f}°")
    
    print("\nCONTROL ACTIVITY:")
    print(f"  Avg Yaw Speed:   {np.mean(np.abs(df['yaw_speed'])):7.2f}")
    print(f"  Avg Pitch Speed: {np.mean(np.abs(df['pitch_speed'])):7.2f}")
    print(f"  Max Yaw Speed:   {np.max(np.abs(df['yaw_speed'])):7.0f}")
    print(f"  Max Pitch Speed: {np.max(np.abs(df['pitch_speed'])):7.0f}")
    
    # Accuracy analysis
    yaw_accurate = np.sum(np.abs(yaw_error) < 2.0) / len(yaw_error) * 100
    pitch_accurate = np.sum(np.abs(pitch_error) < 2.0) / len(pitch_error) * 100
    
    print(f"\nACCURACY (within 2°):")
    print(f"  Yaw:             {yaw_accurate:7.1f}%")
    print(f"  Pitch:           {pitch_accurate:7.1f}%")
    
    print("="*60)

def export_csv(data, filename="gimbal_data.csv"):
    """Export data to CSV for further analysis"""
    
    if not data or len(data['timestamp']) == 0:
        print("No data to export!")
        return
    
    df = pd.DataFrame(data)
    
    # Add calculated columns with proper yaw wraparound
    yaw_error_raw = df['required_yaw'] - df['current_yaw']
    # Normalize yaw errors to [-180, +180] range
    yaw_error_normalized = yaw_error_raw.copy()
    yaw_error_normalized[yaw_error_normalized > 180] -= 360
    yaw_error_normalized[yaw_error_normalized < -180] += 360
    
    df['yaw_error'] = yaw_error_normalized
    df['yaw_error_raw'] = yaw_error_raw  # Keep original for reference
    df['pitch_error'] = df['required_pitch'] - df['current_pitch']
    df['total_error'] = np.sqrt(df['yaw_error']**2 + df['pitch_error']**2)
    
    df.to_csv(filename, index=False)
    print(f"Data exported to {filename}")

def main():
    """Main function"""
    
    print("Gimbal Log Analysis Tool")
    print("="*50)
    
    # Parse the log file
    data = parse_gimbal_log("gimbal_log.txt")
    
    if data is None:
        return
    
    # Create plots
    create_plots(data)
    
    # Show statistics
    create_summary_stats(data)
    
    # Export CSV
    export_csv(data)
    
    print("\nAnalysis complete!")
    print("Files generated:")
    print("  - gimbal_performance.png (visual plots)")
    print("  - gimbal_data.csv (raw data)")

if __name__ == "__main__":
    main()