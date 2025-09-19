#!/usr/bin/env python3
"""
Manual Log Analysis Script for Gimbal Sessions
Run this script to analyze any gimbal session logs manually
"""
import os
import sys
import argparse
import subprocess
from pathlib import Path

def find_session_directories(base_dir="/home/zagros/Documents/Gimbal_Flight_Logs"):
    """Find all available session directories"""
    if not os.path.exists(base_dir):
        return []
    
    sessions = []
    for item in os.listdir(base_dir):
        session_path = os.path.join(base_dir, item)
        if os.path.isdir(session_path) and item.startswith("Flight_"):
            sessions.append(session_path)
    
    return sorted(sessions, reverse=True)  # Most recent first

def analyze_session(session_dir, script_dir):
    """Run analysis on a specific session"""
    print(f"\n[ANALYSIS] Analyzing session: {os.path.basename(session_dir)}")
    print(f"[ANALYSIS] Session directory: {session_dir}")
    
    # Check if session has data
    csv_files = [
        os.path.join(session_dir, "gimbal_performance.csv"),
        os.path.join(session_dir, "coordinate_calculations.csv"),
        os.path.join(session_dir, "target_selections.csv")
    ]
    
    has_data = any(os.path.exists(f) and os.path.getsize(f) > 100 for f in csv_files)
    
    if not has_data:
        print("[ANALYSIS] ⚠️  No significant data found in session")
        return False
    
    # Find analysis script
    analysis_script = os.path.join(script_dir, "gimbal_app", "session_logging", "analyze_session.py")
    
    if not os.path.exists(analysis_script):
        print(f"[ANALYSIS] ❌ Analysis script not found at: {analysis_script}")
        return False
    
    # Use virtual environment Python if available
    venv_python = os.path.join(script_dir, "venv", "bin", "python3")
    if not os.path.exists(venv_python):
        venv_python = "python3"
    
    print(f"[ANALYSIS] Using Python: {venv_python}")
    
    try:
        # Run analysis
        result = subprocess.run([
            venv_python, analysis_script, session_dir
        ], capture_output=False, text=True, timeout=60)
        
        if result.returncode == 0:
            print(f"[ANALYSIS] ✅ Analysis completed successfully")
            
            # Show results location
            results_dir = os.path.join(session_dir, "analysis_plots")
            if os.path.exists(results_dir):
                print(f"\n[RESULTS] Analysis files available in:")
                print(f"[RESULTS] {results_dir}")
                
                # List generated files
                files = os.listdir(results_dir)
                if files:
                    print(f"[RESULTS] Generated files:")
                    for file in sorted(files):
                        print(f"[RESULTS]   - {file}")
            
            return True
        else:
            print(f"[ANALYSIS] ❌ Analysis failed with return code: {result.returncode}")
            return False
            
    except subprocess.TimeoutExpired:
        print(f"[ANALYSIS] ❌ Analysis timed out")
        return False
    except Exception as e:
        print(f"[ANALYSIS] ❌ Failed to run analysis: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description="Manually analyze gimbal session logs")
    parser.add_argument("--session", "-s", help="Specific session directory to analyze")
    parser.add_argument("--list", "-l", action="store_true", help="List available sessions")
    parser.add_argument("--all", "-a", action="store_true", help="Analyze all sessions")
    parser.add_argument("--latest", action="store_true", help="Analyze only the latest session")
    
    args = parser.parse_args()
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Find available sessions
    sessions = find_session_directories()
    
    if args.list:
        print(f"\n[SESSIONS] Found {len(sessions)} session(s):")
        for i, session in enumerate(sessions, 1):
            session_name = os.path.basename(session)
            print(f"[SESSIONS] {i:2d}. {session_name}")
            
            # Show basic info
            csv_files = [
                os.path.join(session, "gimbal_performance.csv"),
                os.path.join(session, "coordinate_calculations.csv"),
                os.path.join(session, "target_selections.csv")
            ]
            has_data = any(os.path.exists(f) and os.path.getsize(f) > 100 for f in csv_files)
            data_status = "✅ Has data" if has_data else "⚠️  No data"
            
            # Check if already analyzed
            results_dir = os.path.join(session, "analysis_plots")
            analyzed_status = "📊 Analyzed" if os.path.exists(results_dir) else "🔄 Not analyzed"
            
            print(f"[SESSIONS]     {data_status} | {analyzed_status}")
        
        return
    
    if not sessions:
        print("[SESSIONS] ❌ No session directories found")
        return
    
    sessions_to_analyze = []
    
    if args.session:
        # Specific session
        if os.path.isabs(args.session):
            sessions_to_analyze = [args.session]
        else:
            # Try to find by name
            found = False
            for session in sessions:
                if args.session in os.path.basename(session):
                    sessions_to_analyze = [session]
                    found = True
                    break
            if not found:
                print(f"[SESSIONS] ❌ Session '{args.session}' not found")
                return
    
    elif args.latest:
        # Latest session only
        sessions_to_analyze = [sessions[0]]
    
    elif args.all:
        # All sessions
        sessions_to_analyze = sessions
    
    else:
        # Interactive selection
        print(f"\n[SESSIONS] Found {len(sessions)} session(s). Select one to analyze:")
        for i, session in enumerate(sessions, 1):
            session_name = os.path.basename(session)
            csv_files = [
                os.path.join(session, "gimbal_performance.csv"),
                os.path.join(session, "coordinate_calculations.csv"),
                os.path.join(session, "target_selections.csv")
            ]
            has_data = any(os.path.exists(f) and os.path.getsize(f) > 100 for f in csv_files)
            data_indicator = "✅" if has_data else "⚠️ "
            
            print(f"[SESSIONS] {i:2d}. {data_indicator} {session_name}")
        
        try:
            choice = input(f"\n[SESSIONS] Enter session number (1-{len(sessions)}) or 'q' to quit: ")
            if choice.lower() == 'q':
                return
            
            session_idx = int(choice) - 1
            if 0 <= session_idx < len(sessions):
                sessions_to_analyze = [sessions[session_idx]]
            else:
                print("[SESSIONS] ❌ Invalid selection")
                return
        except (ValueError, KeyboardInterrupt):
            print("\n[SESSIONS] Cancelled")
            return
    
    # Analyze selected sessions
    success_count = 0
    for session_dir in sessions_to_analyze:
        try:
            if analyze_session(session_dir, script_dir):
                success_count += 1
        except KeyboardInterrupt:
            print("\n[ANALYSIS] Cancelled by user")
            break
    
    print(f"\n[SUMMARY] Analyzed {success_count}/{len(sessions_to_analyze)} session(s) successfully")

if __name__ == "__main__":
    main()