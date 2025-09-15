#!/usr/bin/env python3
"""
Gimbal Log Analyzer - Parse gimbal_log.txt and create readable summary
No external dependencies required - pure Python
"""

import re
from datetime import datetime
import json

def parse_gimbal_log(log_file="gimbal_log.txt"):
    """Parse the gimbal log file and extract data"""
    
    commands = []
    recoveries = []
    warnings = []
    
    print(f"Analyzing {log_file}...")
    
    try:
        with open(log_file, 'r') as f:
            lines = f.readlines()
            
        print(f"Processing {len(lines)} log lines...")
        
        for line_num, line in enumerate(lines, 1):
            try:
                # Extract timestamp
                timestamp_match = re.match(r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})', line)
                if not timestamp_match:
                    continue
                
                timestamp_str = timestamp_match.group(1)
                
                # Parse GIMBAL_CMD entries  
                if 'GIMBAL_CMD' in line:
                    cmd_match = re.search(r'Target: Y=(-?\d+\.?\d*)° P=(-?\d+\.?\d*)° \| Current: Y=(-?\d+\.?\d*)° P=(-?\d+\.?\d*)° \| Speed: yaw=(-?\d+) pitch=(-?\d+)', line)
                    if cmd_match:
                        cmd_data = {
                            'timestamp': timestamp_str,
                            'target_yaw': float(cmd_match.group(1)),
                            'target_pitch': float(cmd_match.group(2)),
                            'current_yaw': float(cmd_match.group(3)),
                            'current_pitch': float(cmd_match.group(4)),
                            'yaw_speed': int(cmd_match.group(5)),
                            'pitch_speed': int(cmd_match.group(6))
                        }
                        
                        # Calculate errors with proper yaw wraparound
                        yaw_error_raw = cmd_data['target_yaw'] - cmd_data['current_yaw']
                        # Normalize yaw error to [-180, +180] for shortest path
                        while yaw_error_raw > 180: yaw_error_raw -= 360
                        while yaw_error_raw < -180: yaw_error_raw += 360
                        
                        cmd_data['yaw_error'] = yaw_error_raw
                        cmd_data['pitch_error'] = cmd_data['target_pitch'] - cmd_data['current_pitch']
                        cmd_data['total_error'] = (cmd_data['yaw_error']**2 + cmd_data['pitch_error']**2)**0.5
                        
                        commands.append(cmd_data)
                
                # Parse RECOVERY entries
                elif 'RECOVERY' in line:
                    recovery_match = re.search(r'RECOVERY \| Reason: (.+?) \| Current: (-?\d+\.?\d*)° \| Diff: (-?\d+\.?\d*)°', line)
                    if recovery_match:
                        recoveries.append({
                            'timestamp': timestamp_str,
                            'reason': recovery_match.group(1),
                            'current_pitch': float(recovery_match.group(2)),
                            'pitch_diff': float(recovery_match.group(3))
                        })
                
                # Parse WARNING entries
                elif 'WARNING' in line and 'Large yaw difference' in line:
                    warning_match = re.search(r'Large yaw difference \((-?\d+\.?\d*)°\) with strong command \((-?\d+)\)', line)
                    if warning_match:
                        warnings.append({
                            'timestamp': timestamp_str,
                            'yaw_diff': float(warning_match.group(1)),
                            'command': int(warning_match.group(2))
                        })
            
            except Exception as e:
                print(f"Error parsing line {line_num}: {e}")
                continue
    
    except FileNotFoundError:
        print(f"Error: {log_file} not found!")
        return None, None, None
    
    print(f"Parsed {len(commands)} commands, {len(recoveries)} recoveries, {len(warnings)} warnings")
    return commands, recoveries, warnings

def analyze_performance(commands):
    """Analyze gimbal performance from commands"""
    
    if not commands:
        return
    
    # Calculate statistics
    yaw_errors = [cmd['yaw_error'] for cmd in commands]
    pitch_errors = [cmd['pitch_error'] for cmd in commands]
    total_errors = [cmd['total_error'] for cmd in commands]
    yaw_speeds = [abs(cmd['yaw_speed']) for cmd in commands]
    pitch_speeds = [abs(cmd['pitch_speed']) for cmd in commands]
    
    def stats(data, name):
        if not data:
            return f"{name}: No data"
        
        mean_val = sum(data) / len(data)
        sorted_data = sorted(data)
        median_val = sorted_data[len(data)//2]
        min_val = min(data)
        max_val = max(data)
        
        return {
            'mean': mean_val,
            'median': median_val,
            'min': min_val,
            'max': max_val,
            'count': len(data)
        }
    
    # Performance analysis
    print("\n" + "="*70)
    print("GIMBAL PERFORMANCE ANALYSIS")
    print("="*70)
    
    print(f"Analysis Period: {commands[0]['timestamp']} to {commands[-1]['timestamp']}")
    print(f"Total Commands: {len(commands)}")
    
    yaw_stats = stats([abs(e) for e in yaw_errors], "Yaw Error")
    pitch_stats = stats([abs(e) for e in pitch_errors], "Pitch Error") 
    
    print(f"\nYAW TRACKING:")
    print(f"  Mean Abs Error:    {yaw_stats['mean']:6.2f}°")
    print(f"  Median Abs Error:  {yaw_stats['median']:6.2f}°")
    print(f"  Max Abs Error:     {yaw_stats['max']:6.2f}°")
    print(f"  Min Abs Error:     {yaw_stats['min']:6.2f}°")
    
    print(f"\nPITCH TRACKING:")
    print(f"  Mean Abs Error:    {pitch_stats['mean']:6.2f}°")
    print(f"  Median Abs Error:  {pitch_stats['median']:6.2f}°")
    print(f"  Max Abs Error:     {pitch_stats['max']:6.2f}°")
    print(f"  Min Abs Error:     {pitch_stats['min']:6.2f}°")
    
    speed_yaw = stats(yaw_speeds, "Yaw Speed")
    speed_pitch = stats(pitch_speeds, "Pitch Speed")
    
    print(f"\nCONTROL ACTIVITY:")
    print(f"  Mean Yaw Speed:    {speed_yaw['mean']:6.2f}")
    print(f"  Mean Pitch Speed:  {speed_pitch['mean']:6.2f}")
    print(f"  Max Yaw Speed:     {speed_yaw['max']:6.0f}")
    print(f"  Max Pitch Speed:   {speed_pitch['max']:6.0f}")
    
    # Accuracy within thresholds
    yaw_good = sum(1 for e in yaw_errors if abs(e) < 2.0)
    pitch_good = sum(1 for e in pitch_errors if abs(e) < 2.0) 
    total_good = sum(1 for e in total_errors if e < 3.0)
    
    print(f"\nACCURACY:")
    print(f"  Yaw within ±2°:    {yaw_good:3d}/{len(commands)} ({100*yaw_good/len(commands):5.1f}%)")
    print(f"  Pitch within ±2°:  {pitch_good:3d}/{len(commands)} ({100*pitch_good/len(commands):5.1f}%)")
    print(f"  Total error <3°:   {total_good:3d}/{len(commands)} ({100*total_good/len(commands):5.1f}%)")

def show_problem_areas(commands, recoveries, warnings):
    """Identify and show problem periods"""
    
    print("\n" + "="*70)
    print("PROBLEM ANALYSIS")
    print("="*70)
    
    # Large errors
    large_yaw_errors = [cmd for cmd in commands if abs(cmd['yaw_error']) > 10.0]
    large_pitch_errors = [cmd for cmd in commands if abs(cmd['pitch_error']) > 10.0]
    
    print(f"\nLARGE ERRORS:")
    print(f"  Yaw errors >10°:   {len(large_yaw_errors)}")
    print(f"  Pitch errors >10°: {len(large_pitch_errors)}")
    
    if large_yaw_errors:
        print(f"\n  Worst Yaw Errors:")
        sorted_yaw = sorted(large_yaw_errors, key=lambda x: abs(x['yaw_error']), reverse=True)[:5]
        for cmd in sorted_yaw:
            print(f"    {cmd['timestamp']} | Error: {cmd['yaw_error']:+6.1f}° | Speed: {cmd['yaw_speed']:+3d}")
    
    if large_pitch_errors:
        print(f"\n  Worst Pitch Errors:")
        sorted_pitch = sorted(large_pitch_errors, key=lambda x: abs(x['pitch_error']), reverse=True)[:5]
        for cmd in sorted_pitch:
            print(f"    {cmd['timestamp']} | Error: {cmd['pitch_error']:+6.1f}° | Speed: {cmd['pitch_speed']:+3d}")
    
    # Recovery analysis
    print(f"\nRECOVERY ATTEMPTS:")
    print(f"  Total recoveries:  {len(recoveries)}")
    
    if recoveries:
        print(f"  Recovery reasons:")
        reasons = {}
        for recovery in recoveries:
            reason = recovery['reason']
            reasons[reason] = reasons.get(reason, 0) + 1
        
        for reason, count in reasons.items():
            print(f"    {reason}: {count}")
    
    # Warning analysis  
    print(f"\nWARNINGS:")
    print(f"  Large yaw warnings: {len(warnings)}")
    
    if warnings:
        print(f"  Worst yaw differences:")
        sorted_warnings = sorted(warnings, key=lambda x: abs(x['yaw_diff']), reverse=True)[:5]
        for warning in sorted_warnings:
            print(f"    {warning['timestamp']} | Diff: {warning['yaw_diff']:+6.1f}° | Cmd: {warning['command']:+3d}")

def export_readable_data(commands, recoveries, warnings, filename="gimbal_analysis.txt"):
    """Export human-readable analysis"""
    
    with open(filename, 'w') as f:
        f.write("GIMBAL LOG ANALYSIS REPORT\n")
        f.write("="*50 + "\n\n")
        
        if commands:
            f.write(f"Commands analyzed: {len(commands)}\n")
            f.write(f"Time period: {commands[0]['timestamp']} to {commands[-1]['timestamp']}\n\n")
            
            f.write("SAMPLE COMMANDS (every 10th entry):\n")
            f.write("Time          | Target Y/P    | Current Y/P   | Error Y/P     | Speed Y/P\n")
            f.write("-" * 80 + "\n")
            
            for i, cmd in enumerate(commands):
                if i % 10 == 0:  # Every 10th command
                    f.write(f"{cmd['timestamp'][-8:]} | "
                           f"{cmd['target_yaw']:6.1f}/{cmd['target_pitch']:+5.1f} | "
                           f"{cmd['current_yaw']:6.1f}/{cmd['current_pitch']:+5.1f} | "
                           f"{cmd['yaw_error']:+6.1f}/{cmd['pitch_error']:+5.1f} | "
                           f"{cmd['yaw_speed']:+3d}/{cmd['pitch_speed']:+3d}\n")
        
        f.write(f"\nRecoveries: {len(recoveries)}\n")
        f.write(f"Warnings: {len(warnings)}\n")
    
    print(f"Detailed analysis saved to {filename}")

def main():
    """Main analysis function"""
    
    print("Gimbal Log Analyzer")
    print("=" * 50)
    
    # Parse log
    commands, recoveries, warnings = parse_gimbal_log("gimbal_log.txt")
    
    if commands is None:
        return
    
    # Analyze performance
    analyze_performance(commands)
    
    # Show problems
    show_problem_areas(commands, recoveries, warnings)
    
    # Export readable data
    export_readable_data(commands, recoveries, warnings)
    
    print("\n" + "="*70)
    print("Analysis complete! Check 'gimbal_analysis.txt' for detailed data.")

if __name__ == "__main__":
    main()