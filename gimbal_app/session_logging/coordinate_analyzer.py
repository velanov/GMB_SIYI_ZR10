"""
Coordinate Analysis Script
Analyzes coordinate calculations and creates elevation/transformation plots
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from typing import Dict, List, Tuple
import json
from mpl_toolkits.mplot3d import Axes3D

class CoordinateAnalyzer:
    """Analyze coordinate calculation data and create plots"""
    
    def __init__(self, session_dir: str):
        self.session_dir = session_dir
        self.coord_csv = os.path.join(session_dir, "coordinate_calculations.csv")
        self.target_csv = os.path.join(session_dir, "target_selections.csv")
        self.output_dir = os.path.join(session_dir, "analysis_plots")
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Load coordinate calculation data
        try:
            self.df = pd.read_csv(self.coord_csv)
            self.df['timestamp'] = pd.to_datetime(self.df['timestamp'], unit='s')
            print(f"[COORD ANALYZER] Loaded {len(self.df)} coordinate calculation records")
        except Exception as e:
            print(f"[COORD ANALYZER] Error loading coordinate data: {e}")
            self.df = pd.DataFrame()
        
        # Load target selection data
        try:
            self.target_df = pd.read_csv(self.target_csv)
            self.target_df['timestamp'] = pd.to_datetime(self.target_df['timestamp'], unit='s')
            print(f"[COORD ANALYZER] Loaded {len(self.target_df)} target selection records")
        except Exception as e:
            print(f"[COORD ANALYZER] Error loading target data: {e}")
            self.target_df = pd.DataFrame()
    
    def analyze_coordinate_accuracy(self) -> Dict[str, float]:
        """Analyze coordinate calculation statistics"""
        if self.df.empty:
            return {}
        
        # Calculate transformation impact statistics
        avg_diff = self.df['coordinate_diff_meters'].mean()
        max_diff = self.df['coordinate_diff_meters'].max()
        min_diff = self.df['coordinate_diff_meters'].min()
        std_diff = self.df['coordinate_diff_meters'].std()
        
        # Flight statistics
        max_altitude = self.df['aircraft_alt'].max()
        min_altitude = self.df['aircraft_alt'].min()
        avg_altitude = self.df['aircraft_alt'].mean()
        
        # Calculate flight path distance
        if len(self.df) > 1:
            lat_diff = self.df['aircraft_lat'].diff()
            lon_diff = self.df['aircraft_lon'].diff()
            # Simple distance approximation
            distances = np.sqrt(
                (lat_diff * 111320) ** 2 + 
                (lon_diff * 111320 * np.cos(np.radians(self.df['aircraft_lat']))) ** 2
            )
            total_distance = distances.sum()
        else:
            total_distance = 0
        
        # Gimbal usage statistics
        avg_gimbal_pitch = self.df['gimbal_pitch'].mean()
        avg_gimbal_yaw = self.df['gimbal_yaw'].mean()
        pitch_range = self.df['gimbal_pitch'].max() - self.df['gimbal_pitch'].min()
        yaw_range = self.df['gimbal_yaw'].max() - self.df['gimbal_yaw'].min()
        
        stats = {
            'avg_transformation_diff_m': avg_diff,
            'max_transformation_diff_m': max_diff,
            'min_transformation_diff_m': min_diff,
            'std_transformation_diff_m': std_diff,
            'max_altitude_m': max_altitude,
            'min_altitude_m': min_altitude,
            'avg_altitude_m': avg_altitude,
            'total_flight_distance_m': total_distance,
            'avg_gimbal_pitch_deg': avg_gimbal_pitch,
            'avg_gimbal_yaw_deg': avg_gimbal_yaw,
            'gimbal_pitch_range_deg': pitch_range,
            'gimbal_yaw_range_deg': yaw_range,
            'total_calculations': len(self.df)
        }
        
        return stats
    
    def create_coordinate_plots(self):
        """Create coordinate analysis plots"""
        if self.df.empty:
            print("[COORD ANALYZER] No data to plot")
            return
        
        # Set up the plotting style
        plt.style.use('default')
        fig = plt.figure(figsize=(20, 16))
        
        # Plot 1: Flight Path with Coordinate Differences
        ax1 = plt.subplot(3, 3, 1)
        
        # Color points by transformation difference
        scatter = plt.scatter(self.df['aircraft_lon'], self.df['aircraft_lat'], 
                            c=self.df['coordinate_diff_meters'], cmap='viridis', 
                            alpha=0.7, s=20)
        plt.colorbar(scatter, label='Transformation Diff (m)')
        plt.xlabel('Longitude')
        plt.ylabel('Latitude')
        plt.title('Flight Path (colored by coordinate transformation impact)')
        plt.grid(True, alpha=0.3)
        
        # Plot 2: Altitude Profile
        ax2 = plt.subplot(3, 3, 2)
        time_minutes = (self.df['timestamp'] - self.df['timestamp'].iloc[0]).dt.total_seconds() / 60
        plt.plot(time_minutes, self.df['aircraft_alt'], 'b-', linewidth=1.5, label='Altitude')
        plt.fill_between(time_minutes, self.df['aircraft_alt'], alpha=0.3)
        plt.xlabel('Time (minutes)')
        plt.ylabel('Altitude (m)')
        plt.title('Altitude Profile')
        plt.grid(True, alpha=0.3)
        plt.legend()
        
        # Plot 3: Coordinate Transformation Differences Over Time
        ax3 = plt.subplot(3, 3, 3)
        plt.plot(time_minutes, self.df['coordinate_diff_meters'], 'r-', linewidth=1.5)
        plt.xlabel('Time (minutes)')
        plt.ylabel('Coordinate Difference (m)')
        plt.title('3D Transformation Impact Over Time')
        plt.grid(True, alpha=0.3)
        
        # Plot 4: Before vs After Euler Coordinates (Latitude)
        ax4 = plt.subplot(3, 3, 4)
        plt.scatter(self.df['coord_before_euler_lat'], self.df['coord_after_euler_lat'], 
                   alpha=0.6, s=10, c='blue')
        
        # Perfect correlation line
        min_lat = min(self.df['coord_before_euler_lat'].min(), self.df['coord_after_euler_lat'].min())
        max_lat = max(self.df['coord_before_euler_lat'].max(), self.df['coord_after_euler_lat'].max())
        plt.plot([min_lat, max_lat], [min_lat, max_lat], 'r--', alpha=0.8, label='Perfect Correlation')
        
        plt.xlabel('Latitude Before Euler Transformation')
        plt.ylabel('Latitude After Euler Transformation')
        plt.title('Latitude: Before vs After 3D Transformation')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Plot 5: Before vs After Euler Coordinates (Longitude)
        ax5 = plt.subplot(3, 3, 5)
        plt.scatter(self.df['coord_before_euler_lon'], self.df['coord_after_euler_lon'], 
                   alpha=0.6, s=10, c='green')
        
        # Perfect correlation line
        min_lon = min(self.df['coord_before_euler_lon'].min(), self.df['coord_after_euler_lon'].min())
        max_lon = max(self.df['coord_before_euler_lon'].max(), self.df['coord_after_euler_lon'].max())
        plt.plot([min_lon, max_lon], [min_lon, max_lon], 'r--', alpha=0.8, label='Perfect Correlation')
        
        plt.xlabel('Longitude Before Euler Transformation')
        plt.ylabel('Longitude After Euler Transformation')
        plt.title('Longitude: Before vs After 3D Transformation')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Plot 6: Gimbal Angles Over Time
        ax6 = plt.subplot(3, 3, 6)
        plt.plot(time_minutes, self.df['gimbal_pitch'], 'b-', alpha=0.8, label='Pitch', linewidth=1)
        plt.plot(time_minutes, self.df['gimbal_yaw'], 'r-', alpha=0.8, label='Yaw', linewidth=1)
        plt.xlabel('Time (minutes)')
        plt.ylabel('Angle (degrees)')
        plt.title('Gimbal Angles Over Time')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Plot 7: Transformation Difference vs Altitude
        ax7 = plt.subplot(3, 3, 7)
        plt.scatter(self.df['aircraft_alt'], self.df['coordinate_diff_meters'], 
                   alpha=0.6, s=15, c='purple')
        plt.xlabel('Aircraft Altitude (m)')
        plt.ylabel('Coordinate Difference (m)')
        plt.title('Transformation Impact vs Altitude')
        plt.grid(True, alpha=0.3)
        
        # Add trend line
        z = np.polyfit(self.df['aircraft_alt'], self.df['coordinate_diff_meters'], 1)
        p = np.poly1d(z)
        plt.plot(self.df['aircraft_alt'], p(self.df['aircraft_alt']), "r--", alpha=0.8, linewidth=2)
        
        # Plot 8: Transformation Difference Distribution
        ax8 = plt.subplot(3, 3, 8)
        plt.hist(self.df['coordinate_diff_meters'], bins=30, alpha=0.7, color='orange', edgecolor='black')
        plt.axvline(self.df['coordinate_diff_meters'].mean(), color='red', linestyle='--', 
                   linewidth=2, label=f'Mean: {self.df["coordinate_diff_meters"].mean():.2f}m')
        plt.xlabel('Coordinate Difference (m)')
        plt.ylabel('Frequency')
        plt.title('Distribution of Transformation Impact')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Plot 9: 3D Visualization of Target Points
        ax9 = plt.subplot(3, 3, 9, projection='3d')
        
        # Sample data for 3D plot (take every 10th point to avoid clutter)
        sample_indices = range(0, len(self.df), max(1, len(self.df) // 100))
        sample_df = self.df.iloc[sample_indices]
        
        # Plot before transformation points
        ax9.scatter(sample_df['coord_before_euler_lon'], sample_df['coord_before_euler_lat'], 
                   sample_df['terrain_elevation'], c='blue', alpha=0.6, s=20, label='Before 3D Transform')
        
        # Plot after transformation points
        ax9.scatter(sample_df['coord_after_euler_lon'], sample_df['coord_after_euler_lat'], 
                   sample_df['terrain_elevation'], c='red', alpha=0.6, s=20, label='After 3D Transform')
        
        ax9.set_xlabel('Longitude')
        ax9.set_ylabel('Latitude')
        ax9.set_zlabel('Elevation (m)')
        ax9.set_title('3D Target Points: Before vs After Transformation')
        ax9.legend()
        
        plt.tight_layout()
        
        # Save plot
        plot_file = os.path.join(self.output_dir, "coordinate_analysis.png")
        plt.savefig(plot_file, dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"[COORD ANALYZER] Coordinate analysis plot saved: {plot_file}")
        return plot_file
    
    def create_elevation_profile_plot(self):
        """Create detailed elevation profile plot"""
        if self.df.empty:
            return None
        
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))
        
        time_minutes = (self.df['timestamp'] - self.df['timestamp'].iloc[0]).dt.total_seconds() / 60
        
        # Plot 1: Aircraft altitude vs terrain elevation
        ax1.plot(time_minutes, self.df['aircraft_alt'], 'b-', linewidth=2, label='Aircraft Altitude', alpha=0.8)
        ax1.fill_between(time_minutes, self.df['terrain_elevation'], alpha=0.3, color='brown', label='Terrain Elevation')
        ax1.fill_between(time_minutes, self.df['terrain_elevation'], self.df['aircraft_alt'], 
                        alpha=0.2, color='lightblue', label='AGL Height')
        
        ax1.set_xlabel('Time (minutes)')
        ax1.set_ylabel('Elevation (m)')
        ax1.set_title('Elevation Profile: Aircraft vs Terrain')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Plot 2: Coordinate transformation impact vs elevation difference
        elevation_diff = self.df['aircraft_alt'] - self.df['terrain_elevation']
        ax2.scatter(elevation_diff, self.df['coordinate_diff_meters'], alpha=0.6, s=15, c='green')
        ax2.set_xlabel('Height Above Ground (m)')
        ax2.set_ylabel('Coordinate Transformation Impact (m)')
        ax2.set_title('Transformation Impact vs Height Above Ground')
        ax2.grid(True, alpha=0.3)
        
        # Add trend line
        if len(elevation_diff) > 1:
            z = np.polyfit(elevation_diff, self.df['coordinate_diff_meters'], 1)
            p = np.poly1d(z)
            ax2.plot(elevation_diff, p(elevation_diff), "r--", alpha=0.8, linewidth=2, 
                    label=f'Trend: y={z[0]:.4f}x+{z[1]:.2f}')
            ax2.legend()
        
        plt.tight_layout()
        
        elevation_plot_file = os.path.join(self.output_dir, "elevation_profile.png")
        plt.savefig(elevation_plot_file, dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"[COORD ANALYZER] Elevation profile plot saved: {elevation_plot_file}")
        return elevation_plot_file
    
    def create_summary_report(self):
        """Create coordinate analysis summary report"""
        stats = self.analyze_coordinate_accuracy()
        
        report_file = os.path.join(self.output_dir, "coordinate_analysis_report.txt")
        
        with open(report_file, 'w') as f:
            f.write("COORDINATE ANALYSIS REPORT\n")
            f.write("=" * 50 + "\n\n")
            
            f.write("3D TRANSFORMATION IMPACT:\n")
            f.write(f"  Average Impact: {stats.get('avg_transformation_diff_m', 0):.2f} meters\n")
            f.write(f"  Maximum Impact: {stats.get('max_transformation_diff_m', 0):.2f} meters\n")
            f.write(f"  Minimum Impact: {stats.get('min_transformation_diff_m', 0):.2f} meters\n")
            f.write(f"  Standard Deviation: {stats.get('std_transformation_diff_m', 0):.2f} meters\n\n")
            
            f.write("FLIGHT STATISTICS:\n")
            f.write(f"  Maximum Altitude: {stats.get('max_altitude_m', 0):.1f} meters\n")
            f.write(f"  Minimum Altitude: {stats.get('min_altitude_m', 0):.1f} meters\n")
            f.write(f"  Average Altitude: {stats.get('avg_altitude_m', 0):.1f} meters\n")
            f.write(f"  Total Flight Distance: {stats.get('total_flight_distance_m', 0):.1f} meters\n\n")
            
            f.write("GIMBAL USAGE:\n")
            f.write(f"  Average Pitch: {stats.get('avg_gimbal_pitch_deg', 0):.1f}°\n")
            f.write(f"  Average Yaw: {stats.get('avg_gimbal_yaw_deg', 0):.1f}°\n")
            f.write(f"  Pitch Range: {stats.get('gimbal_pitch_range_deg', 0):.1f}°\n")
            f.write(f"  Yaw Range: {stats.get('gimbal_yaw_range_deg', 0):.1f}°\n")
            f.write(f"  Total Calculations: {stats.get('total_calculations', 0)}\n\n")
            
            # Analysis assessment
            avg_impact = stats.get('avg_transformation_diff_m', 0)
            if avg_impact < 1.0:
                assessment = "MINIMAL - 3D transformations have low impact"
            elif avg_impact < 5.0:
                assessment = "MODERATE - 3D transformations provide some correction"
            elif avg_impact < 10.0:
                assessment = "SIGNIFICANT - 3D transformations provide important corrections"
            else:
                assessment = "HIGH - 3D transformations are critical for accuracy"
            
            f.write(f"TRANSFORMATION ASSESSMENT: {assessment}\n")
            f.write(f"The 3D Euler transformations change coordinate calculations by an average of {avg_impact:.2f} meters.\n")
        
        print(f"[COORD ANALYZER] Coordinate analysis report saved: {report_file}")
        return report_file
    
    def create_target_selection_report(self):
        """Create target selection summary report"""
        if self.target_df.empty:
            return None
            
        report_file = os.path.join(self.output_dir, "target_selection_report.txt")
        
        with open(report_file, 'w') as f:
            f.write("TARGET SELECTION ANALYSIS REPORT\n")
            f.write("=" * 50 + "\n\n")
            
            f.write("TARGET SELECTION SUMMARY:\n")
            f.write(f"  Total Targets Selected: {len(self.target_df)}\n")
            
            # Breakdown by selection mode
            mode_counts = self.target_df['selection_mode'].value_counts()
            for mode, count in mode_counts.items():
                f.write(f"  {mode.replace('_', ' ').title()}: {count}\n")
            
            f.write("\n3D TRANSFORMATION ANALYSIS:\n")
            valid_transformations = self.target_df[self.target_df['transformation_impact_meters'] > 0]
            if not valid_transformations.empty:
                avg_impact = valid_transformations['transformation_impact_meters'].mean()
                max_impact = valid_transformations['transformation_impact_meters'].max()
                min_impact = valid_transformations['transformation_impact_meters'].min()
                
                f.write(f"  Average 3D Impact: {avg_impact:.2f} meters\n")
                f.write(f"  Maximum 3D Impact: {max_impact:.2f} meters\n")
                f.write(f"  Minimum 3D Impact: {min_impact:.2f} meters\n")
                f.write(f"  Targets with 3D Data: {len(valid_transformations)}/{len(self.target_df)}\n")
            else:
                f.write("  No 3D transformation data available\n")
            
            f.write("\nALTITUDE ANALYSIS:\n")
            valid_altitudes = self.target_df[self.target_df['aircraft_alt_agl'] > 0]
            if not valid_altitudes.empty:
                avg_alt = valid_altitudes['aircraft_alt_agl'].mean()
                max_alt = valid_altitudes['aircraft_alt_agl'].max()
                min_alt = valid_altitudes['aircraft_alt_agl'].min()
                
                f.write(f"  Average UAV Altitude: {avg_alt:.1f} meters AGL\n")
                f.write(f"  Maximum UAV Altitude: {max_alt:.1f} meters AGL\n")
                f.write(f"  Minimum UAV Altitude: {min_alt:.1f} meters AGL\n")
            else:
                f.write("  No altitude data available\n")
            
            f.write("\nTARGET DISTANCE ANALYSIS:\n")
            valid_distances = self.target_df[self.target_df['target_distance_2d'] > 0]
            if not valid_distances.empty:
                avg_dist = valid_distances['target_distance_2d'].mean()
                max_dist = valid_distances['target_distance_2d'].max()
                min_dist = valid_distances['target_distance_2d'].min()
                
                f.write(f"  Average Target Distance: {avg_dist:.1f} meters\n")
                f.write(f"  Maximum Target Distance: {max_dist:.1f} meters\n")
                f.write(f"  Minimum Target Distance: {min_dist:.1f} meters\n")
            else:
                f.write("  No distance data available\n")
            
            # Detailed target list
            f.write("\nDETAILED TARGET LOG:\n")
            f.write("-" * 50 + "\n")
            for idx, row in self.target_df.iterrows():
                timestamp = row['timestamp'].strftime('%H:%M:%S')
                f.write(f"{timestamp} | {row['selection_mode'].upper()}\n")
                f.write(f"  Target: {row['target_lat']:.6f},{row['target_lon']:.6f} @ {row['target_alt']:.1f}m\n")
                f.write(f"  Aircraft: {row['aircraft_lat']:.6f},{row['aircraft_lon']:.6f} @ {row['aircraft_alt_agl']:.1f}m AGL\n")
                
                # Show ray intersection analysis if available
                if (row['coord_before_euler_lat'] != row['coord_after_euler_lat'] or 
                    row['coord_before_euler_lon'] != row['coord_after_euler_lon']):
                    f.write(f"  Raw Estimate: {row['coord_before_euler_lat']:.6f},{row['coord_before_euler_lon']:.6f}\n")
                    f.write(f"  Terrain Corrected: {row['coord_after_euler_lat']:.6f},{row['coord_after_euler_lon']:.6f}\n")
                    if row['transformation_impact_meters'] > 0:
                        f.write(f"  Terrain Impact: {row['transformation_impact_meters']:.2f}m\n")
                else:
                    # Fallback for manual coordinate entry or no terrain correction
                    if row['transformation_impact_meters'] > 0:
                        f.write(f"  3D Impact: {row['transformation_impact_meters']:.2f}m\n")
                
                f.write(f"  Distance: {row['target_distance_2d']:.1f}m\n")
                f.write("\n")
        
        print(f"[COORD ANALYZER] Target selection report saved: {report_file}")
        return report_file

def analyze_coordinate_session(session_dir: str):
    """Main function to analyze coordinate calculations for a session"""
    print(f"[COORD ANALYZER] Starting coordinate analysis for session: {os.path.basename(session_dir)}")
    
    analyzer = CoordinateAnalyzer(session_dir)
    
    # Create plots and reports
    coord_plot = analyzer.create_coordinate_plots()
    elevation_plot = analyzer.create_elevation_profile_plot()
    report_file = analyzer.create_summary_report()
    target_report = analyzer.create_target_selection_report()
    
    # Save analysis metadata
    stats = analyzer.analyze_coordinate_accuracy()
    analysis_metadata = {
        'analysis_timestamp': pd.Timestamp.now().isoformat(),
        'coordinate_plot': coord_plot,
        'elevation_plot': elevation_plot,
        'report_file': report_file,
        'target_report': target_report,
        'statistics': stats
    }
    
    metadata_file = os.path.join(session_dir, "coordinate_analysis_metadata.json")
    with open(metadata_file, 'w') as f:
        json.dump(analysis_metadata, f, indent=2)
    
    print(f"[COORD ANALYZER] Analysis complete. Results saved in: {analyzer.output_dir}")
    return analyzer.output_dir

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        session_dir = sys.argv[1]
        analyze_coordinate_session(session_dir)
    else:
        print("Usage: python coordinate_analyzer.py <session_directory>")