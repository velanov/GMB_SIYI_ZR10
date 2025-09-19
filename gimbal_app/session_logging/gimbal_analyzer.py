"""
Gimbal Performance Analysis Script
Analyzes gimbal angle accuracy and creates performance plots
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from typing import Dict, List, Tuple
import json

class GimbalAnalyzer:
    """Analyze gimbal performance data and create plots"""
    
    def __init__(self, session_dir: str):
        self.session_dir = session_dir
        self.gimbal_csv = os.path.join(session_dir, "gimbal_performance.csv")
        self.output_dir = os.path.join(session_dir, "analysis_plots")
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Load data
        try:
            self.df = pd.read_csv(self.gimbal_csv)
            self.df['timestamp'] = pd.to_datetime(self.df['timestamp'], unit='s')
            print(f"[GIMBAL ANALYZER] Loaded {len(self.df)} gimbal performance records")
        except Exception as e:
            print(f"[GIMBAL ANALYZER] Error loading data: {e}")
            self.df = pd.DataFrame()
    
    def analyze_accuracy(self) -> Dict[str, float]:
        """Analyze gimbal accuracy statistics"""
        if self.df.empty:
            return {}
        
        # Filter out manual control data since it doesn't have meaningful target accuracy
        tracking_df = self.df[self.df['operation_mode'] != 'manual']
        
        if tracking_df.empty:
            print("[GIMBAL ANALYZER] No tracking data found - only manual control logged")
            return {
                'pitch_accuracy_percent': 0,
                'yaw_accuracy_percent': 0,
                'overall_accuracy_percent': 0,
                'average_pitch_error_deg': 0,
                'average_yaw_error_deg': 0,
                'max_pitch_error_deg': 0,
                'max_yaw_error_deg': 0,
                'average_response_time_sec': 0,
                'total_commands': len(self.df),
                'tracking_commands': 0,
                'connection_uptime_percent': (self.df['gimbal_connected'].sum() / len(self.df)) * 100
            }
        
        # Calculate accuracy metrics based on error thresholds (for tracking data only)
        # Accuracy = percentage of commands within acceptable error threshold
        pitch_threshold = 2.0  # 2 degrees acceptable error
        yaw_threshold = 2.0    # 2 degrees acceptable error
        
        pitch_accurate = (tracking_df['pitch_error'] <= pitch_threshold).sum()
        yaw_accurate = (tracking_df['yaw_error'] <= yaw_threshold).sum()
        total_tracking = len(tracking_df)
        
        pitch_accuracy = (pitch_accurate / total_tracking) * 100 if total_tracking > 0 else 0
        yaw_accuracy = (yaw_accurate / total_tracking) * 100 if total_tracking > 0 else 0
        overall_accuracy = (pitch_accuracy + yaw_accuracy) / 2
        
        # Calculate response time (time to reach target within threshold)
        response_times = []
        threshold = 1.0  # 1 degree threshold
        
        for mode in self.df['operation_mode'].unique():
            mode_data = self.df[self.df['operation_mode'] == mode]
            if len(mode_data) > 1:
                # Find sequences where error drops below threshold
                below_threshold = (mode_data['pitch_error'] < threshold) & (mode_data['yaw_error'] < threshold)
                if below_threshold.any():
                    first_accurate = mode_data[below_threshold].iloc[0]
                    start_time = mode_data.iloc[0]['timestamp']
                    response_time = (first_accurate['timestamp'] - start_time).total_seconds()
                    response_times.append(response_time)
        
        avg_response_time = np.mean(response_times) if response_times else 0
        
        stats = {
            'pitch_accuracy_percent': max(0, pitch_accuracy),
            'yaw_accuracy_percent': max(0, yaw_accuracy),
            'overall_accuracy_percent': max(0, overall_accuracy),
            'average_pitch_error_deg': tracking_df['pitch_error'].mean(),
            'average_yaw_error_deg': tracking_df['yaw_error'].mean(),
            'max_pitch_error_deg': tracking_df['pitch_error'].max(),
            'max_yaw_error_deg': tracking_df['yaw_error'].max(),
            'average_response_time_sec': avg_response_time,
            'total_commands': len(self.df),
            'tracking_commands': len(tracking_df),
            'connection_uptime_percent': (self.df['gimbal_connected'].sum() / len(self.df)) * 100
        }
        
        return stats
    
    def create_accuracy_plots(self):
        """Create accuracy and performance plots"""
        if self.df.empty:
            print("[GIMBAL ANALYZER] No data to plot")
            return
        
        # Set up the plotting style
        plt.style.use('default')
        fig = plt.figure(figsize=(16, 12))
        
        # Plot 1: Command vs Actual Angles Over Time
        ax1 = plt.subplot(2, 3, 1)
        time_minutes = (self.df['timestamp'] - self.df['timestamp'].iloc[0]).dt.total_seconds() / 60
        
        plt.plot(time_minutes, self.df['commanded_pitch'], 'b-', alpha=0.7, label='Commanded Pitch', linewidth=1)
        plt.plot(time_minutes, self.df['actual_pitch'], 'r-', alpha=0.7, label='Actual Pitch', linewidth=1)
        plt.xlabel('Time (minutes)')
        plt.ylabel('Pitch Angle (degrees)')
        plt.title('Pitch: Commanded vs Actual')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Plot 2: Yaw Command vs Actual
        ax2 = plt.subplot(2, 3, 2)
        plt.plot(time_minutes, self.df['commanded_yaw'], 'b-', alpha=0.7, label='Commanded Yaw', linewidth=1)
        plt.plot(time_minutes, self.df['actual_yaw'], 'r-', alpha=0.7, label='Actual Yaw', linewidth=1)
        plt.xlabel('Time (minutes)')
        plt.ylabel('Yaw Angle (degrees)')
        plt.title('Yaw: Commanded vs Actual')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Plot 3: Error Over Time
        ax3 = plt.subplot(2, 3, 3)
        plt.plot(time_minutes, self.df['pitch_error'], 'r-', alpha=0.7, label='Pitch Error', linewidth=1)
        plt.plot(time_minutes, self.df['yaw_error'], 'b-', alpha=0.7, label='Yaw Error', linewidth=1)
        plt.xlabel('Time (minutes)')
        plt.ylabel('Error (degrees)')
        plt.title('Angle Errors Over Time')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Plot 4: Error Distribution Histogram
        ax4 = plt.subplot(2, 3, 4)
        plt.hist(self.df['pitch_error'], bins=30, alpha=0.7, label='Pitch Error', color='red')
        plt.hist(self.df['yaw_error'], bins=30, alpha=0.7, label='Yaw Error', color='blue')
        plt.xlabel('Error (degrees)')
        plt.ylabel('Frequency')
        plt.title('Error Distribution')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Plot 5: Accuracy by Operation Mode
        ax5 = plt.subplot(2, 3, 5)
        mode_stats = []
        modes = self.df['operation_mode'].unique()
        
        for mode in modes:
            mode_data = self.df[self.df['operation_mode'] == mode]
            if len(mode_data) > 0:
                avg_error = (mode_data['pitch_error'].mean() + mode_data['yaw_error'].mean()) / 2
                accuracy = max(0, 100 - avg_error)
                mode_stats.append((mode, accuracy, len(mode_data)))
        
        if mode_stats:
            modes, accuracies, counts = zip(*mode_stats)
            bars = plt.bar(modes, accuracies, color=['green', 'blue', 'orange', 'red'][:len(modes)])
            
            # Add count labels on bars
            for bar, count in zip(bars, counts):
                height = bar.get_height()
                plt.text(bar.get_x() + bar.get_width()/2., height + 1,
                        f'n={count}', ha='center', va='bottom', fontsize=8)
            
            plt.xlabel('Operation Mode')
            plt.ylabel('Accuracy (%)')
            plt.title('Accuracy by Operation Mode')
            plt.xticks(rotation=45)
            plt.grid(True, alpha=0.3)
        
        # Plot 6: Performance Summary
        ax6 = plt.subplot(2, 3, 6)
        stats = self.analyze_accuracy()
        
        metrics = ['Pitch\nAccuracy', 'Yaw\nAccuracy', 'Overall\nAccuracy', 'Connection\nUptime']
        values = [
            stats.get('pitch_accuracy_percent', 0),
            stats.get('yaw_accuracy_percent', 0), 
            stats.get('overall_accuracy_percent', 0),
            stats.get('connection_uptime_percent', 0)
        ]
        
        colors = ['green' if v >= 90 else 'orange' if v >= 70 else 'red' for v in values]
        bars = plt.bar(metrics, values, color=colors, alpha=0.7)
        
        # Add value labels on bars
        for bar, value in zip(bars, values):
            height = bar.get_height()
            plt.text(bar.get_x() + bar.get_width()/2., height + 1,
                    f'{value:.1f}%', ha='center', va='bottom', fontweight='bold')
        
        plt.ylabel('Performance (%)')
        plt.title('Performance Summary')
        plt.ylim(0, 105)
        plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # Save plot
        plot_file = os.path.join(self.output_dir, "gimbal_performance_analysis.png")
        plt.savefig(plot_file, dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"[GIMBAL ANALYZER] Performance plot saved: {plot_file}")
        
        return plot_file
    
    def create_summary_report(self):
        """Create text summary report"""
        stats = self.analyze_accuracy()
        
        report_file = os.path.join(self.output_dir, "gimbal_performance_report.txt")
        
        with open(report_file, 'w') as f:
            f.write("GIMBAL PERFORMANCE ANALYSIS REPORT\n")
            f.write("=" * 50 + "\n\n")
            
            f.write("ACCURACY METRICS (TRACKING MODE ONLY):\n")
            f.write(f"  Overall Accuracy: {stats.get('overall_accuracy_percent', 0):.1f}%\n")
            f.write(f"  Pitch Accuracy: {stats.get('pitch_accuracy_percent', 0):.1f}%\n")
            f.write(f"  Yaw Accuracy: {stats.get('yaw_accuracy_percent', 0):.1f}%\n")
            f.write(f"  (Accuracy = % of commands within 2° of target)\n\n")
            
            f.write("ERROR STATISTICS:\n")
            f.write(f"  Average Pitch Error: {stats.get('average_pitch_error_deg', 0):.2f}°\n")
            f.write(f"  Average Yaw Error: {stats.get('average_yaw_error_deg', 0):.2f}°\n")
            f.write(f"  Maximum Pitch Error: {stats.get('max_pitch_error_deg', 0):.2f}°\n")
            f.write(f"  Maximum Yaw Error: {stats.get('max_yaw_error_deg', 0):.2f}°\n\n")
            
            f.write("PERFORMANCE METRICS:\n")
            f.write(f"  Average Response Time: {stats.get('average_response_time_sec', 0):.2f} seconds\n")
            f.write(f"  Connection Uptime: {stats.get('connection_uptime_percent', 0):.1f}%\n")
            f.write(f"  Total Commands Logged: {stats.get('total_commands', 0)}\n")
            f.write(f"  Tracking Commands: {stats.get('tracking_commands', 0)}\n")
            f.write(f"  Manual Commands: {stats.get('total_commands', 0) - stats.get('tracking_commands', 0)}\n\n")
            
            # Performance assessment
            overall_acc = stats.get('overall_accuracy_percent', 0)
            if overall_acc >= 95:
                assessment = "EXCELLENT - Gimbal performing within specifications"
            elif overall_acc >= 90:
                assessment = "GOOD - Minor accuracy variations detected"
            elif overall_acc >= 80:
                assessment = "FAIR - Some accuracy issues present"
            else:
                assessment = "POOR - Significant accuracy problems detected"
            
            f.write(f"ASSESSMENT: {assessment}\n")
        
        print(f"[GIMBAL ANALYZER] Performance report saved: {report_file}")
        return report_file

def analyze_gimbal_session(session_dir: str):
    """Main function to analyze gimbal performance for a session"""
    print(f"[GIMBAL ANALYZER] Starting gimbal analysis for session: {os.path.basename(session_dir)}")
    
    analyzer = GimbalAnalyzer(session_dir)
    
    # Create plots and report
    plot_file = analyzer.create_accuracy_plots()
    report_file = analyzer.create_summary_report()
    
    # Save analysis metadata
    stats = analyzer.analyze_accuracy()
    analysis_metadata = {
        'analysis_timestamp': pd.Timestamp.now().isoformat(),
        'plot_file': plot_file,
        'report_file': report_file,
        'statistics': stats
    }
    
    metadata_file = os.path.join(session_dir, "gimbal_analysis_metadata.json")
    with open(metadata_file, 'w') as f:
        json.dump(analysis_metadata, f, indent=2)
    
    print(f"[GIMBAL ANALYZER] Analysis complete. Results saved in: {analyzer.output_dir}")
    return analyzer.output_dir

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        session_dir = sys.argv[1]
        analyze_gimbal_session(session_dir)
    else:
        print("Usage: python gimbal_analyzer.py <session_directory>")