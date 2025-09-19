"""
Main Analysis Runner Script
Executes both gimbal and coordinate analysis for a completed session
"""

import os
import sys
import subprocess
import json
from datetime import datetime

def run_session_analysis(session_dir: str):
    """Run complete analysis for a flight session"""
    if not os.path.exists(session_dir):
        print(f"[ANALYSIS] Error: Session directory not found: {session_dir}")
        return False
    
    print(f"[ANALYSIS] Starting complete analysis for session: {os.path.basename(session_dir)}")
    print(f"[ANALYSIS] Session directory: {session_dir}")
    
    # Get the directory containing this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    analysis_results = {
        'session_dir': session_dir,
        'analysis_start_time': datetime.now().isoformat(),
        'gimbal_analysis': None,
        'coordinate_analysis': None,
        'errors': []
    }
    
    try:
        # Run gimbal analysis
        print("[ANALYSIS] Running gimbal performance analysis...")
        gimbal_script = os.path.join(script_dir, "gimbal_analyzer.py")
        
        try:
            result = subprocess.run([
                sys.executable, gimbal_script, session_dir
            ], capture_output=True, text=True, timeout=300)  # 5 minute timeout
            
            if result.returncode == 0:
                print("[ANALYSIS] Gimbal analysis completed successfully")
                analysis_results['gimbal_analysis'] = 'success'
            else:
                print(f"[ANALYSIS] Gimbal analysis failed: {result.stderr}")
                analysis_results['errors'].append(f"Gimbal analysis: {result.stderr}")
                analysis_results['gimbal_analysis'] = 'failed'
        
        except subprocess.TimeoutExpired:
            print("[ANALYSIS] Gimbal analysis timed out")
            analysis_results['errors'].append("Gimbal analysis: Timeout")
            analysis_results['gimbal_analysis'] = 'timeout'
        
        except Exception as e:
            print(f"[ANALYSIS] Gimbal analysis error: {e}")
            analysis_results['errors'].append(f"Gimbal analysis: {e}")
            analysis_results['gimbal_analysis'] = 'error'
        
        # Run coordinate analysis
        print("[ANALYSIS] Running coordinate calculation analysis...")
        coord_script = os.path.join(script_dir, "coordinate_analyzer.py")
        
        try:
            result = subprocess.run([
                sys.executable, coord_script, session_dir
            ], capture_output=True, text=True, timeout=300)  # 5 minute timeout
            
            if result.returncode == 0:
                print("[ANALYSIS] Coordinate analysis completed successfully")
                analysis_results['coordinate_analysis'] = 'success'
            else:
                print(f"[ANALYSIS] Coordinate analysis failed: {result.stderr}")
                analysis_results['errors'].append(f"Coordinate analysis: {result.stderr}")
                analysis_results['coordinate_analysis'] = 'failed'
        
        except subprocess.TimeoutExpired:
            print("[ANALYSIS] Coordinate analysis timed out")
            analysis_results['errors'].append("Coordinate analysis: Timeout")
            analysis_results['coordinate_analysis'] = 'timeout'
        
        except Exception as e:
            print(f"[ANALYSIS] Coordinate analysis error: {e}")
            analysis_results['errors'].append(f"Coordinate analysis: {e}")
            analysis_results['coordinate_analysis'] = 'error'
        
    except Exception as e:
        print(f"[ANALYSIS] General analysis error: {e}")
        analysis_results['errors'].append(f"General error: {e}")
    
    # Save analysis results summary
    analysis_results['analysis_end_time'] = datetime.now().isoformat()
    
    results_file = os.path.join(session_dir, "analysis_results.json")
    with open(results_file, 'w') as f:
        json.dump(analysis_results, f, indent=2)
    
    # Create summary
    success_count = sum([
        1 for result in [analysis_results['gimbal_analysis'], analysis_results['coordinate_analysis']]
        if result == 'success'
    ])
    
    print(f"[ANALYSIS] Analysis complete. {success_count}/2 analyses successful.")
    
    if analysis_results['errors']:
        print("[ANALYSIS] Errors encountered:")
        for error in analysis_results['errors']:
            print(f"  - {error}")
    
    # Print results summary
    analysis_dir = os.path.join(session_dir, "analysis_plots")
    if os.path.exists(analysis_dir):
        plots = [f for f in os.listdir(analysis_dir) if f.endswith('.png')]
        reports = [f for f in os.listdir(analysis_dir) if f.endswith('.txt')]
        
        print(f"[ANALYSIS] Results available in: {analysis_dir}")
        print(f"[ANALYSIS] Generated {len(plots)} plots and {len(reports)} reports")
        
        if plots:
            print("[ANALYSIS] Generated plots:")
            for plot in plots:
                print(f"  - {plot}")
        
        if reports:
            print("[ANALYSIS] Generated reports:")
            for report in reports:
                print(f"  - {report}")
    
    return success_count == 2

if __name__ == "__main__":
    if len(sys.argv) > 1:
        session_dir = sys.argv[1]
        success = run_session_analysis(session_dir)
        sys.exit(0 if success else 1)
    else:
        print("Usage: python analyze_session.py <session_directory>")
        sys.exit(1)