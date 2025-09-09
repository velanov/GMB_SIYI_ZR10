"""
Gimbal Target UI Components - Extracted UI functionality from gimbal_gps_ui_v2.py
Provides UI panels for fixed target management and target mode switching
"""

import tkinter as tk
from tkinter import ttk, messagebox
from typing import Optional, Callable, Dict, Any
from gimbal_fixed_target_system import FixedTargetSystem

class TargetModePanel:
    """UI panel for switching between gimbal and fixed target modes"""
    
    def __init__(self, parent_frame, target_system: FixedTargetSystem, on_mode_change: Optional[Callable] = None):
        self.target_system = target_system
        self.on_mode_change = on_mode_change
        
        # Create main frame
        self.frame = ttk.LabelFrame(parent_frame, text="Target Mode Selection")
        self.frame.pack(padx=10, pady=5, fill="x")
        
        # Create UI elements
        self._create_widgets()
        
    def _create_widgets(self):
        """Create the target mode selection widgets"""
        
        # Mode selection
        mode_row = ttk.Frame(self.frame)
        mode_row.pack(fill="x", padx=5, pady=5)
        
        ttk.Label(mode_row, text="Target Mode:").pack(side="left")
        
        self.target_mode_var = tk.StringVar(value="gimbal")
        
        ttk.Radiobutton(mode_row, text="Gimbal Calculated", 
                       variable=self.target_mode_var, value="gimbal",
                       command=self._on_target_mode_change).pack(side="left", padx=10)
        ttk.Radiobutton(mode_row, text="Fixed Coordinates", 
                       variable=self.target_mode_var, value="fixed",
                       command=self._on_target_mode_change).pack(side="left", padx=10)
        
        # Fixed coordinate controls (initially hidden)
        self.fixed_coord_frame = ttk.Frame(self.frame)
        
        # Coordinate input row 1
        coord_row1 = ttk.Frame(self.fixed_coord_frame)
        coord_row1.pack(fill="x", padx=5, pady=3)
        
        ttk.Label(coord_row1, text="Latitude:", width=10).pack(side="left")
        self.fixed_lat_var = tk.StringVar()
        ttk.Entry(coord_row1, textvariable=self.fixed_lat_var, width=15).pack(side="left", padx=5)
        
        ttk.Label(coord_row1, text="Longitude:", width=10).pack(side="left", padx=(20,0))
        self.fixed_lon_var = tk.StringVar()
        ttk.Entry(coord_row1, textvariable=self.fixed_lon_var, width=15).pack(side="left", padx=5)
        
        ttk.Label(coord_row1, text="Alt (m):", width=8).pack(side="left", padx=(20,0))
        self.fixed_alt_var = tk.StringVar(value="0.0")
        ttk.Entry(coord_row1, textvariable=self.fixed_alt_var, width=10).pack(side="left", padx=5)
        
        # Coordinate input row 2 (buttons)
        coord_row2 = ttk.Frame(self.fixed_coord_frame)
        coord_row2.pack(fill="x", padx=5, pady=3)
        
        ttk.Button(coord_row2, text="Set Fixed Target", 
                  command=self._set_fixed_target).pack(side="left", padx=(20,0))
        ttk.Button(coord_row2, text="Use Current Aircraft Position", 
                  command=self._use_aircraft_position).pack(side="left", padx=5)
        
        # Initially hide fixed coordinate controls
        self.fixed_coord_frame.pack_forget()
    
    def _on_target_mode_change(self):
        """Handle target mode change"""
        mode = self.target_mode_var.get()
        
        # Update target system
        if mode == "fixed":
            self.target_system.target_manager.set_target_mode("fixed")
            self.fixed_coord_frame.pack(fill="x", padx=5, pady=5)
        else:
            self.target_system.target_manager.set_target_mode("gimbal")
            self.fixed_coord_frame.pack_forget()
        
        # Callback for additional handling
        if self.on_mode_change:
            self.on_mode_change(mode)
    
    def _set_fixed_target(self):
        """Set the fixed coordinate target"""
        try:
            lat = float(self.fixed_lat_var.get())
            lon = float(self.fixed_lon_var.get()) 
            alt = float(self.fixed_alt_var.get())
            
            success, message = self.target_system.set_fixed_target(lat, lon, alt)
            
            if success:
                messagebox.showinfo("Success", message)
            else:
                messagebox.showerror("Error", message)
                
        except ValueError:
            messagebox.showerror("Error", "Please enter valid numeric coordinates")
    
    def _use_aircraft_position(self):
        """Use current aircraft position as fixed target"""
        if self.target_system.aircraft_state and self.target_system.aircraft_state.get('lat') is not None:
            # Update entry fields with aircraft position
            self.fixed_lat_var.set(f"{self.target_system.aircraft_state['lat']:.7f}")
            self.fixed_lon_var.set(f"{self.target_system.aircraft_state['lon']:.7f}")
            self.fixed_alt_var.set(f"{self.target_system.aircraft_state.get('alt_agl', 0.0):.1f}")
            
            # Set the target
            self._set_fixed_target()
        else:
            messagebox.showerror("Error", "No aircraft position available")
    
    def update_aircraft_state(self, aircraft_state: Dict[str, float]):
        """Update aircraft state for the panel"""
        # This is handled by the target system, but could be used for UI updates
        pass

class TargetDisplayPanel:
    """UI panel for displaying current target information"""
    
    def __init__(self, parent_frame, target_system: FixedTargetSystem):
        self.target_system = target_system
        
        # Create main frame
        self.frame = ttk.LabelFrame(parent_frame, text="Target Information")
        self.frame.pack(padx=10, pady=5, fill="x")
        
        # Create display area
        self.txt_target = tk.Text(self.frame, height=8, wrap=tk.WORD, 
                                 font=("Consolas", 9), relief="flat", borderwidth=0)
        self.txt_target.pack(fill="x", padx=5, pady=5)
        
        # Start periodic updates
        self._schedule_update()
    
    def _schedule_update(self):
        """Schedule periodic display updates"""
        self.update_display()
        # Schedule next update
        if hasattr(self, 'frame') and self.frame.winfo_exists():
            self.frame.after(1000, self._schedule_update)
    
    def update_display(self):
        """Update the target display with current information"""
        try:
            status = self.target_system.get_system_status()
            target_info = status['target_info']
            gimbal_lock_info = status['gimbal_lock_info']
            tracking_stats = status['tracking_stats']
            
            self.txt_target.delete(1.0, tk.END)
            
            # Target mode header
            info = f"=== TARGET MODE: {target_info['mode'].upper()} ===\n"
            
            if target_info['mode'] == "fixed":
                if target_info['has_target']:
                    # Fixed target information
                    info += f"Fixed Target: {target_info['target_lat']:.7f}, {target_info['target_lon']:.7f}\n"
                    info += f"Fixed Altitude: {target_info['target_alt']:.1f}m\n"
                    
                    # Distance to fixed target
                    if target_info.get('distance_to_target') is not None:
                        info += f"Distance: {target_info['distance_to_target']:.0f}m\n"
                    
                    # Gimbal lock status
                    if gimbal_lock_info['active']:
                        info += f"GIMBAL LOCKED ON TARGET ✓\n"
                        if gimbal_lock_info.get('required_angles'):
                            angles = gimbal_lock_info['required_angles']
                            info += f"Required: P:{angles['pitch']:.1f}° Y:{angles['yaw']:.1f}°\n"
                    else:
                        info += "Gimbal lock: OFF\n"
                else:
                    info += "No fixed target set - enter coordinates and click 'Set Fixed Target'\n"
                
            else:
                # Gimbal mode
                if target_info['has_target']:
                    gimbal_target = target_info.get('gimbal_target', {})
                    result = gimbal_target.get('calculation_result')
                    
                    if result:
                        info += f"Gimbal Target: {target_info['target_lat']:.7f}, {target_info['target_lon']:.7f}\n"
                        info += f"Distance: {gimbal_target.get('distance', 0):.0f}m | {result.get('note', '')}\n"
                        info += f"Gimbal: P:{result['pitch']:.1f}° Y:{result['yaw']:.1f}°\n"
                        
                        # Gimbal lock status
                        if gimbal_lock_info['active']:
                            info += f"GIMBAL LOCKED ON TARGET ✓\n"
                        else:
                            info += "Gimbal lock: OFF\n"
                    else:
                        info += "No gimbal target - point gimbal downward\n"
                else:
                    info += "No gimbal target calculated - point gimbal downward\n"
            
            # Tracking information
            if tracking_stats['active']:
                info += f"\nTRACKING: {tracking_stats['duration']:.0f}s | {tracking_stats['updates']} updates\n"
                info += f"Next update: {tracking_stats['next_update']:.1f}s\n"
            else:
                info += "\nTracking: OFF - Ready for tracking\n"
            
            # Aircraft position (if available)
            if self.target_system.aircraft_state:
                aircraft = self.target_system.aircraft_state
                info += f"\nAircraft: {aircraft.get('lat', 0):.6f}, {aircraft.get('lon', 0):.6f}\n"
                info += f"Alt: {aircraft.get('alt_agl', 0):.1f}m | Hdg: {aircraft.get('heading', 0):.1f}°"
            
            self.txt_target.insert(1.0, info)
            
        except Exception as e:
            # Fallback display on error
            self.txt_target.delete(1.0, tk.END)
            self.txt_target.insert(1.0, f"Display update error: {str(e)}")

class TargetControlPanel:
    """UI panel for target control operations"""
    
    def __init__(self, parent_frame, target_system: FixedTargetSystem, mavlink_handler):
        self.target_system = target_system
        self.mavlink = mavlink_handler
        
        # Create main frame
        self.frame = ttk.LabelFrame(parent_frame, text="Target Controls")
        self.frame.pack(padx=10, pady=5, fill="x")
        
        self._create_widgets()
    
    def _create_widgets(self):
        """Create target control widgets"""
        
        # Gimbal lock controls
        lock_frame = ttk.Frame(self.frame)
        lock_frame.pack(fill="x", padx=5, pady=5)
        
        self.btn_start_lock = ttk.Button(lock_frame, text="Start Gimbal Lock", 
                                        command=self._start_gimbal_lock)
        self.btn_start_lock.pack(side="left", padx=5)
        
        self.btn_stop_lock = ttk.Button(lock_frame, text="Stop Gimbal Lock", 
                                       command=self._stop_gimbal_lock, state="disabled")
        self.btn_stop_lock.pack(side="left", padx=5)
        
        # Tracking parameters
        params_frame = ttk.Frame(self.frame)
        params_frame.pack(fill="x", padx=5, pady=5)
        
        # Radius control
        radius_frame = ttk.Frame(params_frame)
        radius_frame.pack(fill="x", pady=3)
        ttk.Label(radius_frame, text="Loiter Radius:", width=15).pack(side="left")
        self.radius_var = tk.DoubleVar(value=500.0)
        radius_scale = ttk.Scale(radius_frame, from_=300.0, to=4000.0, variable=self.radius_var,
                                orient="horizontal", length=200)
        radius_scale.pack(side="left", padx=5)
        self.lbl_radius = ttk.Label(radius_frame, text="500m")
        self.lbl_radius.pack(side="left", padx=5)
        radius_scale.config(command=lambda v: self.lbl_radius.config(text=f"{float(v):.0f}m"))
        
        # Update interval control
        update_frame = ttk.Frame(params_frame)
        update_frame.pack(fill="x", pady=3)
        ttk.Label(update_frame, text="Update Interval:", width=15).pack(side="left")
        self.update_var = tk.DoubleVar(value=1.0)
        update_scale = ttk.Scale(update_frame, from_=0.5, to=5.0, variable=self.update_var,
                                orient="horizontal", length=200)
        update_scale.pack(side="left", padx=5)
        self.lbl_update = ttk.Label(update_frame, text="1.0s")
        self.lbl_update.pack(side="left", padx=5)
        update_scale.config(command=lambda v: self.lbl_update.config(text=f"{float(v):.1f}s"))
        
        # Movement threshold control
        movement_frame = ttk.Frame(params_frame)
        movement_frame.pack(fill="x", pady=3)
        ttk.Label(movement_frame, text="Min Movement:", width=15).pack(side="left")
        self.movement_var = tk.DoubleVar(value=10.0)
        movement_scale = ttk.Scale(movement_frame, from_=5.0, to=100.0, variable=self.movement_var,
                                  orient="horizontal", length=200)
        movement_scale.pack(side="left", padx=5)
        self.lbl_movement = ttk.Label(movement_frame, text="10m")
        self.lbl_movement.pack(side="left", padx=5)
        movement_scale.config(command=lambda v: self.lbl_movement.config(text=f"{float(v):.0f}m"))
        
        # Tracking control buttons
        btn_frame = ttk.Frame(self.frame)
        btn_frame.pack(pady=10)
        
        self.btn_start_tracking = ttk.Button(btn_frame, text="Start Dynamic Tracking", 
                                           command=self._start_tracking)
        self.btn_start_tracking.pack(side="left", padx=5)
        
        self.btn_stop_tracking = ttk.Button(btn_frame, text="Stop Tracking", 
                                          command=self._stop_tracking, state="disabled")
        self.btn_stop_tracking.pack(side="left", padx=5)
        
        ttk.Button(btn_frame, text="Single GoTo", command=self._single_goto).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Return to Mission", command=self._return_to_mission).pack(side="left", padx=5)
    
    def _start_gimbal_lock(self):
        """Start gimbal lock"""
        success, message = self.target_system.start_gimbal_lock()
        
        if success:
            self.btn_start_lock.config(state="disabled")
            self.btn_stop_lock.config(state="normal")
        else:
            messagebox.showerror("Error", message)
    
    def _stop_gimbal_lock(self):
        """Stop gimbal lock"""
        self.target_system.stop_gimbal_lock()
        self.btn_start_lock.config(state="normal")
        self.btn_stop_lock.config(state="disabled")
    
    def _start_tracking(self):
        """Start dynamic tracking"""
        try:
            radius = self.radius_var.get()
            interval = self.update_var.get()
            movement = self.movement_var.get()
        except (ValueError, AttributeError):
            messagebox.showerror("Error", "Invalid parameter values")
            return
        
        success, message = self.target_system.start_dynamic_tracking(radius, interval, movement)
        
        if success:
            self.btn_start_tracking.config(state="disabled")
            self.btn_stop_tracking.config(state="normal")
        else:
            messagebox.showerror("Error", message)
    
    def _stop_tracking(self):
        """Stop dynamic tracking"""
        self.target_system.stop_dynamic_tracking()
        self.btn_start_tracking.config(state="normal")
        self.btn_stop_tracking.config(state="disabled")
    
    def _single_goto(self):
        """Execute single goto command"""
        target_lat, target_lon, target_alt = self.target_system.target_manager.get_current_target()
        
        if target_lat is None:
            messagebox.showerror("Error", "No target available")
            return
        if not self.mavlink.connected:
            messagebox.showerror("Error", "No MAVLink connection")
            return
        
        try:
            radius = max(50.0, min(5000.0, float(self.radius_var.get())))
            # Use aircraft altitude for loiter
            alt = self.target_system.aircraft_state.get('alt_amsl', 100.0)
            
            self.mavlink.set_loiter_mode(target_lat, target_lon, alt, radius)
            
        except ValueError:
            messagebox.showerror("Error", "Invalid radius value")
    
    def _return_to_mission(self):
        """Return to mission mode"""
        if self.mavlink.set_mission_mode():
            # Stop tracking
            self.target_system.stop_dynamic_tracking()
            self.btn_start_tracking.config(state="normal")
            self.btn_stop_tracking.config(state="disabled")
        else:
            messagebox.showerror("Error", "Mission mode command failed")

class IntegratedTargetUI:
    """Complete integrated UI for target management"""
    
    def __init__(self, parent_frame, target_system: FixedTargetSystem, mavlink_handler):
        self.target_system = target_system
        self.mavlink = mavlink_handler
        
        # Create UI panels
        self.mode_panel = TargetModePanel(parent_frame, target_system, self._on_mode_change)
        self.display_panel = TargetDisplayPanel(parent_frame, target_system)
        self.control_panel = TargetControlPanel(parent_frame, target_system, mavlink_handler)
    
    def _on_mode_change(self, mode: str):
        """Handle target mode changes"""
        # Additional handling can be added here
        pass
    
    def update_aircraft_state(self, aircraft_state: Dict[str, float]):
        """Update aircraft state across all UI components"""
        self.target_system.update_aircraft_state(aircraft_state)
        self.mode_panel.update_aircraft_state(aircraft_state)
    
    def update_gimbal_target(self, result):
        """Update gimbal calculated target"""
        self.target_system.update_gimbal_target(result)
    
    def cleanup(self):
        """Cleanup UI components"""
        self.target_system.cleanup()

# Example of how to integrate into the main application
def add_target_ui_to_app(app, gimbal, mavlink):
    """
    Example function showing how to add target UI to main application
    
    Add this to your main application class:
    
    def __init__(self):
        # ... existing initialization ...
        
        # Create target system
        self.target_system = FixedTargetSystem(self.gimbal, self.mavlink)
        
        # Create integrated target UI
        self.target_ui = IntegratedTargetUI(self.root, self.target_system, self.mavlink)
    
    def _main_loop(self):
        # ... existing main loop code ...
        
        # Update target system with aircraft state
        if self.mavlink.connected:
            position = self.mavlink.get_position()
            if position:
                self.aircraft_state.update(position)
            heading = self.mavlink.get_attitude()
            if heading is not None:
                self.aircraft_state['heading'] = heading
            
            # Update target UI
            self.target_ui.update_aircraft_state(self.aircraft_state)
        
        # Update gimbal target if in gimbal mode
        if hasattr(self, 'target_ui'):
            gimbal_result = self._calculate_target()  # Your existing function
            if gimbal_result:
                self.target_ui.update_gimbal_target(gimbal_result)
    
    def shutdown(self):
        # ... existing shutdown code ...
        
        if hasattr(self, 'target_ui'):
            self.target_ui.cleanup()
    """
    pass