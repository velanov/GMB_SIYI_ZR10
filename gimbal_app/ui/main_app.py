from ..shared import *
from ..gimbal.siyi_gimbal import SiyiGimbal
from ..gimbal.locker import GimbalLocker
from ..mavlink.handler import MAVLinkHandler
from ..adsb.sbs_publisher import SBSPublisher
from ..tracking.dynamic_tracker import DynamicTracker
from ..calc.target_calculator import TargetCalculator
from .settings_dialog import SettingsDialog
from .manual_control import ManualControlWindow
from ..google_earth.controller import GoogleEarthController, GoogleEarthConfig
from ..google_earth.waypoint_manager import TrackingMode

class GimbalGPSAppV2:
    """Enhanced version with both gimbal tracking and fixed coordinate modes"""
    
    def __init__(self):
        # Load persisted settings before creating systems
        SettingsStore.load_into()

        # Initialize core systems with (possibly) loaded config
        self.gimbal = SiyiGimbal(Config.SIYI_IP, Config.SIYI_PORT)
        self.sbs = SBSPublisher(Config.SBS_BIND, Config.SBS_PORT)
        self.mavlink = MAVLinkHandler(Config.MAVLINK_ADDRESS, Config.MAVLINK_TX_ADDRESS or None)
        self.tracker = DynamicTracker(self.mavlink)
        self.gimbal_locker = GimbalLocker(self.gimbal)  # NEW: Gimbal lock system
        
        # Google Earth integration
        try:
            ge_config = GoogleEarthConfig()
            self.google_earth = GoogleEarthController(ge_config)
        except Exception as e:
            print(f"Warning: Failed to initialize Google Earth integration: {e}")
            import traceback
            traceback.print_exc()
            self.google_earth = None
        
        # Application state
        self.aircraft_state = {
            'lat': 47.3977508, 'lon': 8.5455938,
            'alt_amsl': 500.0, 'alt_agl': 100.0, 'heading': 0.0
        }
        
        # NEW: Target mode selection
        self.target_mode = "gimbal"  # "gimbal", "fixed", or "waypoint"
        
        # Gimbal target state (original)
        self.gimbal_target_state = {
            'lat': None, 'lon': None, 'distance': 0.0,
            'calculation_result': None
        }
        
        # Fixed target state (NEW)
        self.fixed_target_state = {
            'lat': None, 'lon': None, 'alt': None
        }
        
        # Setup Google Earth integration callbacks
        if self.google_earth:
            self._setup_google_earth_integration()
        
        self.ui_settings = {
            'calc_rate_hz': 10.0,
            'angle_threshold': 0.5,
            'loiter_radius': Config.DEFAULT_LOITER_RADIUS,
            'update_interval': Config.TRACKING_UPDATE_S,
            'min_movement': Config.MIN_MOVEMENT_THRESHOLD
        }
        self.last_calc_time = 0
        self.last_angles = (None, None)
        self._closing = False
        self.manual_window = None
        
        self._setup_gui()
        self._start_workers()
        self.root.after(Config.GUI_UPDATE_MS, self._main_loop)
    
    def _setup_gui(self):
        self.root = tk.Tk()
        self.root.title("Gimbal GPS V2 - Gimbal + Fixed Coordinate Tracking")
        self.root.geometry("800x800")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        # Menubar with Settings
        menubar = tk.Menu(self.root)
        appmenu = tk.Menu(menubar, tearoff=0)
        appmenu.add_command(label="Settings…", command=self._open_settings)
        appmenu.add_separator()
        appmenu.add_command(label="Exit", command=self._on_close)
        menubar.add_cascade(label="Application", menu=appmenu)
        self.root.config(menu=menubar)
        
        self._create_mavlink_panel()
        self._create_gimbal_panel()
        self._create_target_mode_panel()  # NEW
        self._create_google_earth_panel()  # NEW: Google Earth panel
        self._create_target_display_panel()
        self._create_tracking_panel()
        self._create_controls_panel()
    
    def _create_mavlink_panel(self):
        mavlink_frame = ttk.LabelFrame(self.root, text="MAVLink Status")
        mavlink_frame.pack(padx=10, pady=5, fill="x")
        
        status_text = "Connected ✓" if self.mavlink.connected else "Disconnected (auto-reconnecting...)"
        ttk.Label(mavlink_frame, text=status_text).pack()
        
        self.lbl_telemetry = ttk.Label(mavlink_frame, text=f"RX: {Config.MAVLINK_ADDRESS} | TX: {Config.MAVLINK_TX_ADDRESS or '(same link)'} | Waiting for telemetry...")
        self.lbl_telemetry.pack()
    
    def _create_gimbal_panel(self):
        gimbal_frame = ttk.LabelFrame(self.root, text="SIYI ZR10 Gimbal")
        gimbal_frame.pack(padx=10, pady=5, fill="x")
        
        main_container = ttk.Frame(gimbal_frame); main_container.pack(fill="x", padx=5, pady=5)
        
        left_frame = ttk.Frame(main_container); left_frame.pack(side="left", fill="both", expand=True)
        
        self.var_gimbal = tk.BooleanVar()
        ttk.Checkbutton(left_frame, text=f"Connect SIYI ({Config.SIYI_IP}:{Config.SIYI_PORT})",
                       variable=self.var_gimbal, command=self._toggle_gimbal).pack(anchor="w")
        
        self.lbl_gimbal_status = ttk.Label(left_frame, text="Disconnected")
        self.lbl_gimbal_status.pack(anchor="w", pady=2)
        
        self.lbl_gimbal_info = ttk.Label(left_frame, text="Mount: — | Mode: —")
        self.lbl_gimbal_info.pack(anchor="w")
        
        center_frame = ttk.Frame(main_container); center_frame.pack(side="left", padx=20)
        ttk.Button(center_frame, text="Center", command=self._center_gimbal).pack(pady=2)
        ttk.Label(center_frame, text="Speed:", font=("Arial", 8)).pack(pady=(10,0))
        self.speed_var = tk.IntVar(value=50)
        speed_scale = ttk.Scale(center_frame, from_=10, to=100, variable=self.speed_var,
                               orient="horizontal", length=100)
        speed_scale.pack()
        speed_label = ttk.Label(center_frame, text="50"); speed_label.pack()
        speed_scale.config(command=lambda v: speed_label.config(text=f"{int(float(v))}"))
        
        right_frame = ttk.Frame(main_container); right_frame.pack(side="right")
        yaw_frame = ttk.Frame(right_frame); yaw_frame.pack(pady=5)
        yaw_btn_frame = ttk.Frame(yaw_frame); yaw_btn_frame.pack()
        self._create_control_button(yaw_btn_frame, "< L", 
                                   lambda: (-self.speed_var.get(), 0)).pack(side="left", padx=2)
        self._create_control_button(yaw_btn_frame, "R >", 
                                   lambda: (self.speed_var.get(), 0)).pack(side="left", padx=2)
        
        pitch_frame = ttk.Frame(right_frame); pitch_frame.pack()
        self._create_control_button(pitch_frame, "^ Up", 
                                   lambda: (0, self.speed_var.get())).pack(pady=1)
        self._create_control_button(pitch_frame, "v Down", 
                                   lambda: (0, -self.speed_var.get())).pack(pady=1)
    
    def _create_target_mode_panel(self):
        """NEW: Target mode selection panel"""
        mode_frame = ttk.LabelFrame(self.root, text="Target Mode Selection")
        mode_frame.pack(padx=10, pady=5, fill="x")
        
        # Mode selection
        mode_container = ttk.Frame(mode_frame); mode_container.pack(fill="x", padx=5, pady=5)
        
        self.target_mode_var = tk.StringVar(value="gimbal")
        ttk.Radiobutton(mode_container, text="Gimbal Pointing Mode", 
                       variable=self.target_mode_var, value="gimbal",
                       command=self._on_mode_change).pack(side="left", padx=10)
        ttk.Radiobutton(mode_container, text="Fixed Coordinate Mode", 
                       variable=self.target_mode_var, value="fixed",
                       command=self._on_mode_change).pack(side="left", padx=10)
        ttk.Radiobutton(mode_container, text="Waypoint Mission Mode", 
                       variable=self.target_mode_var, value="waypoint",
                       command=self._on_mode_change).pack(side="left", padx=10)
        
        # Fixed coordinate input panel (initially hidden)
        self.fixed_coord_frame = ttk.Frame(mode_frame)
        self.fixed_coord_frame.pack(fill="x", padx=5, pady=5)
        
        coord_row1 = ttk.Frame(self.fixed_coord_frame); coord_row1.pack(fill="x", pady=2)
        ttk.Label(coord_row1, text="Latitude:", width=10).pack(side="left")
        self.fixed_lat_var = tk.StringVar(value="47.3980000")
        ttk.Entry(coord_row1, textvariable=self.fixed_lat_var, width=15).pack(side="left", padx=5)
        
        ttk.Label(coord_row1, text="Longitude:", width=10).pack(side="left", padx=(20,0))
        self.fixed_lon_var = tk.StringVar(value="8.5470000") 
        ttk.Entry(coord_row1, textvariable=self.fixed_lon_var, width=15).pack(side="left", padx=5)
        
        coord_row2 = ttk.Frame(self.fixed_coord_frame); coord_row2.pack(fill="x", pady=2)
        ttk.Label(coord_row2, text="Altitude (m):", width=10).pack(side="left")
        self.fixed_alt_var = tk.StringVar(value="0")
        ttk.Entry(coord_row2, textvariable=self.fixed_alt_var, width=15).pack(side="left", padx=5)
        
        ttk.Button(coord_row2, text="Set Fixed Target", 
                  command=self._set_fixed_target).pack(side="left", padx=(20,0))
        ttk.Button(coord_row2, text="Use Current Aircraft Position", 
                  command=self._use_aircraft_position).pack(side="left", padx=5)
        
        # Initially hide fixed coordinate controls
        self.fixed_coord_frame.pack_forget()
    
    def _create_google_earth_panel(self):
        """Google Earth integration panel"""
        try:
            ge_frame = ttk.LabelFrame(self.root, text="Google Earth Integration")
            ge_frame.pack(padx=10, pady=5, fill="x")
            
            # File loading section
            file_frame = ttk.Frame(ge_frame); file_frame.pack(fill="x", padx=5, pady=5)
            
            ttk.Label(file_frame, text="KML/KMZ File:").pack(side="left")
            self.kml_file_var = tk.StringVar()
            ttk.Entry(file_frame, textvariable=self.kml_file_var, width=50).pack(side="left", padx=5)
            ttk.Button(file_frame, text="Browse", command=self._browse_kml_file).pack(side="left", padx=2)
            ttk.Button(file_frame, text="Load Mission", command=self._load_kml_mission).pack(side="left", padx=2)
            
            # Mission control section
            mission_frame = ttk.Frame(ge_frame); mission_frame.pack(fill="x", padx=5, pady=5)
            
            # Tracking mode selection
            mode_frame = ttk.Frame(mission_frame); mode_frame.pack(fill="x", pady=2)
            ttk.Label(mode_frame, text="Mode:", width=12).pack(side="left")
            self.waypoint_mode_var = tk.StringVar(value="sequential")
            ttk.Combobox(mode_frame, textvariable=self.waypoint_mode_var, width=15,
                        values=["sequential", "manual_select", "loiter_all"], 
                        state="readonly").pack(side="left", padx=5)
            
            # Mission controls
            control_frame = ttk.Frame(mission_frame); control_frame.pack(fill="x", pady=2)
            self.btn_start_mission = ttk.Button(control_frame, text="Start Mission", 
                                              command=self._start_waypoint_mission)
            self.btn_start_mission.pack(side="left", padx=5)
            
            self.btn_stop_mission = ttk.Button(control_frame, text="Stop Mission", 
                                             command=self._stop_waypoint_mission, state="disabled")
            self.btn_stop_mission.pack(side="left", padx=5)
            
            ttk.Button(control_frame, text="Next Waypoint", 
                      command=self._next_waypoint).pack(side="left", padx=5)
            
            # Telemetry feed controls
            feed_frame = ttk.Frame(mission_frame); feed_frame.pack(fill="x", pady=2)
            self.var_ge_feed = tk.BooleanVar(value=True)
            ttk.Checkbutton(feed_frame, text="Real-time KML Feed", 
                           variable=self.var_ge_feed, command=self._toggle_ge_feed).pack(side="left")
            
            ttk.Button(feed_frame, text="Open in Google Earth", 
                      command=self._open_google_earth).pack(side="left", padx=10)
            
            # Mission status
            self.lbl_mission_status = ttk.Label(ge_frame, text="No mission loaded")
            self.lbl_mission_status.pack(padx=5, pady=2)
            
            # Initially hide this panel (show only when waypoint mode is selected)
            ge_frame.pack_forget()
            self.ge_panel = ge_frame
            
        except Exception as e:
            print(f"Warning: Failed to create Google Earth panel: {e}")
            import traceback
            traceback.print_exc()
            # Create a dummy panel to prevent crashes
            ge_frame = ttk.LabelFrame(self.root, text="Google Earth Integration (Error)")
            ttk.Label(ge_frame, text="Google Earth panel failed to load").pack(pady=10)
            ge_frame.pack_forget()
            self.ge_panel = ge_frame
    
    def _create_target_display_panel(self):
        target_frame = ttk.LabelFrame(self.root, text="Target Information")
        target_frame.pack(padx=10, pady=5, fill="x")
        
        self.txt_target = tk.Text(target_frame, height=8, wrap=tk.WORD, 
                                 font=("Consolas", 9), relief="flat", borderwidth=0)
        self.txt_target.pack(fill="x", padx=5, pady=5)
    
    def _create_tracking_panel(self):
        tracking_frame = ttk.LabelFrame(self.root, text="Dynamic Loiter Tracking")
        tracking_frame.pack(padx=10, pady=5, fill="x")
        
        params_frame = ttk.Frame(tracking_frame); params_frame.pack(fill="x", padx=5, pady=5)
        
        radius_frame = ttk.Frame(params_frame); radius_frame.pack(fill="x", pady=3)
        ttk.Label(radius_frame, text="Loiter Radius:", width=15).pack(side="left")
        self.radius_var = tk.DoubleVar(value=Config.DEFAULT_LOITER_RADIUS)
        radius_scale = ttk.Scale(radius_frame, from_=300.0, to=4000.0, variable=self.radius_var,
                                orient="horizontal", length=200)
        radius_scale.pack(side="left", padx=5)
        self.lbl_radius = ttk.Label(radius_frame, text=f"{Config.DEFAULT_LOITER_RADIUS:.0f}m")
        self.lbl_radius.pack(side="left", padx=5)
        radius_scale.config(command=lambda v: self.lbl_radius.config(text=f"{float(v):.0f}m"))
        
        update_frame = ttk.Frame(params_frame); update_frame.pack(fill="x", pady=3)
        ttk.Label(update_frame, text="Update Interval:", width=15).pack(side="left")
        self.update_var = tk.DoubleVar(value=Config.TRACKING_UPDATE_S)
        update_scale = ttk.Scale(update_frame, from_=0.5, to=5.0, variable=self.update_var,
                                orient="horizontal", length=200)
        update_scale.pack(side="left", padx=5)
        self.lbl_update = ttk.Label(update_frame, text=f"{Config.TRACKING_UPDATE_S:.1f}s")
        self.lbl_update.pack(side="left", padx=5)
        update_scale.config(command=lambda v: self.lbl_update.config(text=f"{float(v):.1f}s"))
        
        movement_frame = ttk.Frame(params_frame); movement_frame.pack(fill="x", pady=3)
        ttk.Label(movement_frame, text="Min Movement:", width=15).pack(side="left")
        self.movement_var = tk.DoubleVar(value=Config.MIN_MOVEMENT_THRESHOLD)
        movement_scale = ttk.Scale(movement_frame, from_=10.0, to=200.0, variable=self.movement_var,
                                  orient="horizontal", length=200)
        movement_scale.pack(side="left", padx=5)
        self.lbl_movement = ttk.Label(movement_frame, text=f"{Config.MIN_MOVEMENT_THRESHOLD:.0f}m")
        self.lbl_movement.pack(side="left", padx=5)
        movement_scale.config(command=lambda v: self.lbl_movement.config(text=f"{float(v):.0f}m"))
        
        # NEW: Gimbal lock controls
        lock_frame = ttk.Frame(params_frame); lock_frame.pack(fill="x", pady=3)
        self.gimbal_lock_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(lock_frame, text="Enable Gimbal Lock (Point gimbal at target during tracking)", 
                       variable=self.gimbal_lock_var).pack(anchor="w")
        
        btn_frame = ttk.Frame(tracking_frame); btn_frame.pack(pady=10)
        self.btn_start_tracking = ttk.Button(btn_frame, text="Start Dynamic Tracking", 
                                           command=self._start_tracking)
        self.btn_start_tracking.pack(side="left", padx=5)
        self.btn_stop_tracking = ttk.Button(btn_frame, text="Stop Tracking", 
                                          command=self._stop_tracking, state="disabled")
        self.btn_stop_tracking.pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Single GoTo", command=self._single_goto).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Return to Mission", command=self._return_to_mission).pack(side="left", padx=5)
    
    def _create_controls_panel(self):
        controls_frame = ttk.LabelFrame(self.root, text="System Controls")
        controls_frame.pack(padx=10, pady=5, fill="x")
        
        controls_container = ttk.Frame(controls_frame); controls_container.pack(fill="x", padx=5, pady=5)
        
        perf_frame = ttk.Frame(controls_container); perf_frame.pack(side="left")
        ttk.Label(perf_frame, text="Calc Rate:").pack(side="left")
        self.calc_rate_var = tk.DoubleVar(value=10.0)
        calc_scale = ttk.Scale(perf_frame, from_=1.0, to=20.0, variable=self.calc_rate_var,
                              orient="horizontal", length=150, command=self._update_calc_rate)
        calc_scale.pack(side="left", padx=5)
        ttk.Label(perf_frame, text="Hz").pack(side="left")
        
        sbs_frame = ttk.Frame(controls_container); sbs_frame.pack(side="right")
        self.var_sbs = tk.BooleanVar()
        ttk.Checkbutton(sbs_frame, text=f"SBS Publisher ({Config.SBS_BIND}:{Config.SBS_PORT})",
                       variable=self.var_sbs, command=self._toggle_sbs).pack()
    
    # =============================================================================
    # NEW: TARGET MODE HANDLING
    # =============================================================================
    
    def _on_mode_change(self):
        """Handle target mode change"""
        self.target_mode = self.target_mode_var.get()
        
        if self.target_mode == "fixed":
            self.fixed_coord_frame.pack(fill="x", padx=5, pady=5)
            self.ge_panel.pack_forget()
        elif self.target_mode == "waypoint":
            try:
                self.fixed_coord_frame.pack_forget()
                self.ge_panel.pack(padx=10, pady=5, fill="x")
            except Exception as e:
                print(f"Error showing Google Earth panel: {e}")
                import traceback
                traceback.print_exc()
                messagebox.showerror("Error", "Failed to show Google Earth panel. Check console for details.")
        else:
            self.fixed_coord_frame.pack_forget()
            self.ge_panel.pack_forget()
    
    def _set_fixed_target(self):
        """Set the fixed coordinate target"""
        try:
            lat = float(self.fixed_lat_var.get())
            lon = float(self.fixed_lon_var.get()) 
            alt = float(self.fixed_alt_var.get())
            
            # Validate coordinates
            if not (-90 <= lat <= 90):
                messagebox.showerror("Error", "Latitude must be between -90 and 90 degrees")
                return
            if not (-180 <= lon <= 180):
                messagebox.showerror("Error", "Longitude must be between -180 and 180 degrees") 
                return
            
            self.fixed_target_state['lat'] = lat
            self.fixed_target_state['lon'] = lon
            self.fixed_target_state['alt'] = alt
            
            # Log target setting
            self.gimbal.logger.log_target_set(lat, lon, alt, "fixed_coordinates_ui")
            
            messagebox.showinfo("Success", f"Fixed target set to:\nLat: {lat:.7f}\nLon: {lon:.7f}\nAlt: {alt}m")
            
        except ValueError:
            messagebox.showerror("Error", "Please enter valid numeric coordinates")
    
    def _use_aircraft_position(self):
        """Use current aircraft position as fixed target"""
        if self.aircraft_state['lat'] and self.aircraft_state['lon']:
            self.fixed_lat_var.set(f"{self.aircraft_state['lat']:.7f}")
            self.fixed_lon_var.set(f"{self.aircraft_state['lon']:.7f}")
            self.fixed_alt_var.set("0")  # Ground level
            self._set_fixed_target()
        else:
            messagebox.showerror("Error", "No aircraft position available")
    
    def _get_current_target(self) -> Tuple[Optional[float], Optional[float], Optional[float]]:
        """Get current target coordinates based on selected mode"""
        if self.target_mode == "fixed":
            return (self.fixed_target_state['lat'], 
                    self.fixed_target_state['lon'],
                    self.fixed_target_state['alt'])
        elif self.target_mode == "waypoint":
            # Get current waypoint from Google Earth controller
            if self.google_earth:
                coordinates = self.google_earth.get_current_target_coordinates()
                if coordinates:
                    return coordinates
        else:
            # Gimbal mode
            if self.gimbal_target_state['lat'] is not None:
                return (self.gimbal_target_state['lat'],
                        self.gimbal_target_state['lon'], 
                        0.0)  # Ground level for gimbal targets
        return None, None, None
    
    # =============================================================================
    # SETTINGS HOOKS
    # =============================================================================

    def _open_settings(self):
        SettingsDialog(self.root, self._apply_settings)

    def _apply_settings(self, new_cfg: Dict[str, Any]):
        """Re-create or reconnect subsystems to new addresses/ports."""
        try:
            # Recreate SIYI with new IP/port
            was_on = bool(self.var_gimbal.get())
            if was_on:
                try: self.gimbal.stop()
                except: pass
            self.gimbal = SiyiGimbal(Config.SIYI_IP, Config.SIYI_PORT)
            if was_on:
                self.var_gimbal.set(True)
                self.gimbal.start()

            # Update MAVLink (RX/TX)
            self.mavlink.set_connection_strings(Config.MAVLINK_ADDRESS, Config.MAVLINK_TX_ADDRESS or None)

            # Restart SBS if active
            if self.var_sbs.get():
                try: self.sbs.stop()
                except: pass
                self.sbs = SBSPublisher(Config.SBS_BIND, Config.SBS_PORT)
                self.sbs.start()
            else:
                self.sbs = SBSPublisher(Config.SBS_BIND, Config.SBS_PORT)

            # Update labels
            self.lbl_telemetry.config(text=f"RX: {Config.MAVLINK_ADDRESS} | TX: {Config.MAVLINK_TX_ADDRESS or '(same link)'} | Waiting for telemetry...")
        except Exception as e:
            messagebox.showerror("Error", f"Error updating subsystems: {e}")
    
    # =============================================================================
    # GOOGLE EARTH INTEGRATION
    # =============================================================================
    
    def _setup_google_earth_integration(self):
        """Setup Google Earth integration callbacks"""
        if not self.google_earth:
            return
            
        # Set callback for target changes
        self.google_earth.set_target_changed_callback(self._on_ge_target_changed)
        
        # Set telemetry source
        self.google_earth.set_telemetry_source(self._get_telemetry_for_ge)
    
    def _get_telemetry_for_ge(self) -> dict:
        """Get telemetry data for Google Earth feed"""
        if not self.mavlink.connected:
            return {}
        
        # Get gimbal angles if available
        gimbal_yaw = 0.0
        gimbal_pitch = 0.0
        gimbal_roll = 0.0
        
        if self.gimbal and self.gimbal.is_connected:
            gimbal_yaw = self.gimbal.yaw_abs or 0.0
            gimbal_pitch = self.gimbal.pitch_norm or 0.0  
            gimbal_roll = self.gimbal.roll or 0.0
        
        return {
            'latitude': self.aircraft_state.get('lat', 0.0),
            'longitude': self.aircraft_state.get('lon', 0.0),
            'altitude': self.aircraft_state.get('alt_amsl', 0.0),
            'heading': self.aircraft_state.get('heading', 0.0),
            'speed': 0.0,  # Could get from MAVLink if available
            'battery_voltage': 0.0,  # Could get from MAVLink if available
            'flight_mode': 'UNKNOWN',  # Could get from MAVLink if available
            'gimbal_yaw': gimbal_yaw,
            'gimbal_pitch': gimbal_pitch,
            'gimbal_roll': gimbal_roll
        }
    
    def _on_ge_target_changed(self, lat: float, lon: float, alt: float):
        """Handle Google Earth target change"""
        # This is called when the waypoint manager changes the current target
        # Log target setting to trigger loiter behavior (same as fixed coordinate mode)
        if self.gimbal and self.gimbal.logger:
            self.gimbal.logger.log_target_set(lat, lon, alt, "google_earth_waypoint")
    
    def _browse_kml_file(self):
        """Browse for KML/KMZ file"""
        from tkinter import filedialog
        
        filetypes = [
            ("KML files", "*.kml"),
            ("KMZ files", "*.kmz"),
            ("All Google Earth", "*.kml *.kmz"),
            ("All files", "*.*")
        ]
        
        filename = filedialog.askopenfilename(
            title="Select KML/KMZ file",
            filetypes=filetypes
        )
        
        if filename:
            self.kml_file_var.set(filename)
    
    def _load_kml_mission(self):
        """Load waypoints from KML/KMZ file"""
        if not self.google_earth:
            messagebox.showerror("Error", "Google Earth integration not available")
            return
            
        file_path = self.kml_file_var.get().strip()
        
        if not file_path:
            messagebox.showerror("Error", "Please select a KML/KMZ file first")
            return
        
        try:
            result = self.google_earth.load_mission_from_kml(file_path)
            
            if result['success']:
                waypoint_count = result['waypoint_count']
                self.lbl_mission_status.config(
                    text=f"Mission loaded: {waypoint_count} waypoints from {os.path.basename(file_path)}"
                )
                
                # Show waypoint details
                waypoints_info = "\n".join([
                    f"{i+1}. {wp['name']} ({wp['latitude']:.6f}, {wp['longitude']:.6f})"
                    for i, wp in enumerate(result['waypoints'][:5])  # Show first 5
                ])
                if waypoint_count > 5:
                    waypoints_info += f"\n... and {waypoint_count - 5} more waypoints"
                
                messagebox.showinfo("Mission Loaded", 
                                  f"Successfully loaded {waypoint_count} waypoints:\n\n{waypoints_info}")
            else:
                messagebox.showerror("Error", f"Failed to load mission: {result['error']}")
                self.lbl_mission_status.config(text="Mission load failed")
                
        except Exception as e:
            messagebox.showerror("Error", f"Error loading mission: {e}")
            self.lbl_mission_status.config(text="Mission load failed")
    
    def _start_waypoint_mission(self):
        """Start the waypoint mission"""
        if not self.google_earth:
            messagebox.showerror("Error", "Google Earth integration not available")
            return
            
        # Check if mission is loaded
        if self.google_earth.waypoint_manager.get_waypoint_count() == 0:
            messagebox.showerror("Error", "No mission loaded. Please load a KML/KMZ file first.")
            return
        
        if not self.mavlink.connected:
            messagebox.showerror("Error", "No MAVLink connection")
            return
        
        try:
            # Get tracking mode
            mode_str = self.waypoint_mode_var.get()
            mode_map = {
                'sequential': TrackingMode.SEQUENTIAL,
                'manual_select': TrackingMode.MANUAL_SELECT,
                'loiter_all': TrackingMode.LOITER_ALL
            }
            tracking_mode = mode_map.get(mode_str, TrackingMode.SEQUENTIAL)
            
            # Start mission
            if self.google_earth.start_mission(tracking_mode):
                self.btn_start_mission.configure(state="disabled")
                self.btn_stop_mission.configure(state="normal")
                
                self.lbl_mission_status.config(text=f"Mission active - {mode_str} mode")
                
                # Start Google Earth telemetry feed if enabled
                if self.var_ge_feed.get():
                    self._toggle_ge_feed()
                
            else:
                messagebox.showerror("Error", "Failed to start waypoint mission")
                
        except Exception as e:
            messagebox.showerror("Error", f"Error starting mission: {e}")
    
    def _stop_waypoint_mission(self):
        """Stop the waypoint mission"""
        if not self.google_earth:
            return
            
        self.google_earth.stop_mission()
        self.btn_start_mission.configure(state="normal")
        self.btn_stop_mission.configure(state="disabled")
        self.lbl_mission_status.config(text="Mission stopped")
    
    def _next_waypoint(self):
        """Move to next waypoint in mission"""
        if not self.google_earth:
            messagebox.showerror("Error", "Google Earth integration not available")
            return
            
        if not self.google_earth.waypoint_manager.is_mission_active():
            messagebox.showerror("Error", "No active mission")
            return
        
        next_wp = self.google_earth.next_waypoint()
        if next_wp:
            messagebox.showinfo("Next Waypoint", 
                              f"Moving to: {next_wp['name']}\n"
                              f"Coordinates: {next_wp['latitude']:.6f}, {next_wp['longitude']:.6f}")
        else:
            messagebox.showinfo("Mission Complete", "Mission completed - no more waypoints")
    
    def _toggle_ge_feed(self):
        """Toggle Google Earth telemetry feed"""
        if not self.google_earth:
            messagebox.showerror("Error", "Google Earth integration not available")
            self.var_ge_feed.set(False)
            return
            
        if self.var_ge_feed.get():
            # Start telemetry feed
            try:
                self.google_earth.telemetry_feed.start_auto_update(1.0)  # 1 second updates
                print(f"Google Earth telemetry feed started. Network link: {self.google_earth.telemetry_feed.get_network_link_path()}")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to start telemetry feed: {e}")
                self.var_ge_feed.set(False)
        else:
            # Stop telemetry feed
            self.google_earth.telemetry_feed.stop_auto_update()
    
    def _open_google_earth(self):
        """Open Google Earth with the live feed"""
        if not self.google_earth:
            messagebox.showerror("Error", "Google Earth integration not available")
            return
            
        try:
            network_link_path = self.google_earth.telemetry_feed.get_network_link_path()
            
            if not os.path.exists(network_link_path):
                # Generate initial feed if it doesn't exist
                if self.var_ge_feed.get():
                    self._toggle_ge_feed()
                else:
                    messagebox.showwarning("Warning", "No live feed available. Enable 'Real-time KML Feed' first.")
                    return
            
            # Try to open with default program (Google Earth)
            import subprocess
            import platform
            
            if platform.system() == 'Windows':
                os.startfile(network_link_path)
            elif platform.system() == 'Darwin':  # macOS
                subprocess.call(['open', network_link_path])
            else:  # Linux
                subprocess.call(['xdg-open', network_link_path])
                
            messagebox.showinfo("Google Earth", 
                              f"Opening Google Earth with live feed.\n\n"
                              f"If Google Earth doesn't open automatically, "
                              f"manually open this file:\n{network_link_path}")
                              
        except Exception as e:
            messagebox.showerror("Error", f"Could not open Google Earth: {e}")
            messagebox.showinfo("Manual Instructions", 
                              f"Please manually open this file in Google Earth:\n"
                              f"{self.google_earth.telemetry_feed.get_network_link_path()}")
    
    # =============================================================================
    # EVENT HANDLERS
    # =============================================================================
    
    def _toggle_gimbal(self):
        if self.var_gimbal.get():
            if not self.gimbal.start():
                self.var_gimbal.set(False)
                messagebox.showerror("Error", f"Could not connect to gimbal ({Config.SIYI_IP}:{Config.SIYI_PORT})")
        else:
            self.gimbal.stop()
    
    def _request_gimbal_attitude(self):
        if self.var_gimbal.get() and not self._closing:
            self.gimbal.request_attitude()
            self.root.after(Config.ATTITUDE_REQUEST_MS, self._request_gimbal_attitude)
    
    def _center_gimbal(self):
        if self.gimbal.is_connected:
            self.gimbal.center()
        else:
            tk.messagebox.showerror("Error", "Connect gimbal first")
    
    def _create_control_button(self, parent, text, speed_func):
        btn = ttk.Button(parent, text=text, width=6)
        moving = {"active": False}
        def start_movement():
            moving["active"] = True
            def move():
                if moving["active"] and self.gimbal.is_connected:
                    yaw_speed, pitch_speed = speed_func()
                    self.gimbal.jog(yaw_speed, pitch_speed)
                    self.root.after(50, move)
            move()
        def stop_movement():
            moving["active"] = False
            if self.gimbal.is_connected:
                self.gimbal.jog(0, 0)
        btn.bind("<Button-1>", lambda e: start_movement())
        btn.bind("<ButtonRelease-1>", lambda e: stop_movement())
        return btn
    
    def _start_tracking(self):
        target_lat, target_lon, target_alt = self._get_current_target()
        
        if target_lat is None:
            if self.target_mode == "gimbal":
                tk.messagebox.showerror("Error", "No gimbal target calculated - point gimbal downward first")
            elif self.target_mode == "fixed":
                tk.messagebox.showerror("Error", "No fixed target set - set coordinates first")
            elif self.target_mode == "waypoint":
                tk.messagebox.showerror("Error", "No waypoint target available - load KML file and start mission first")
            return
            
        if not self.mavlink.connected:
            tk.messagebox.showerror("Error", "No MAVLink connection")
            return
            
        try:
            radius = self.radius_var.get()
            interval = self.update_var.get()
            movement = self.movement_var.get()
        except (ValueError, AttributeError):
            tk.messagebox.showerror("Error", "Invalid parameter values")
            return
            
        if self.tracker.start_tracking(
            target_lat, target_lon, self.aircraft_state['alt_amsl'], 
            radius, interval, movement
        ):
            self.btn_start_tracking.configure(state="disabled")
            self.btn_stop_tracking.configure(state="normal")
            
            # NEW: Start gimbal lock if enabled
            if self.gimbal_lock_var.get() and self.gimbal.is_connected:
                # Use target altitude or default to ground level
                lock_target_alt = 0.0
                if self.target_mode == "fixed" and self.fixed_target_state['alt'] is not None:
                    lock_target_alt = self.fixed_target_state['alt']
                elif self.target_mode == "waypoint" and target_alt is not None:
                    lock_target_alt = target_alt
                self.gimbal_locker.start_locking(target_lat, target_lon, lock_target_alt)
                self.gimbal_locker.update_aircraft_state(self.aircraft_state)
        else:
            tk.messagebox.showerror("Error", "Failed to start tracking")
    
    def _stop_tracking(self):
        self.tracker.stop_tracking()
        # NEW: Stop gimbal lock when tracking stops
        self.gimbal_locker.stop_locking()
        self.btn_start_tracking.configure(state="normal")
        self.btn_stop_tracking.configure(state="disabled")
    
    def _single_goto(self):
        target_lat, target_lon, target_alt = self._get_current_target()
        
        if target_lat is None:
            tk.messagebox.showerror("Error", "No target available")
            return
        if not self.mavlink.connected:
            tk.messagebox.showerror("Error", "No MAVLink connection")
            return
        try:
            radius = max(50.0, min(5000.0, float(self.radius_var.get())))
        except ValueError:
            radius = Config.DEFAULT_LOITER_RADIUS
        self.mavlink.set_loiter_mode(
            target_lat, target_lon, self.aircraft_state['alt_amsl'], radius
        )
    
    def _return_to_mission(self):
        if self.mavlink.set_mission_mode():
            self.tracker.stop_tracking()
            # NEW: Stop gimbal lock when returning to mission
            self.gimbal_locker.stop_locking()
            self.btn_start_tracking.configure(state="normal")
            self.btn_stop_tracking.configure(state="disabled")
        else:
            tk.messagebox.showerror("Error", "Mission mode command failed")
    
    def _toggle_sbs(self):
        if self.var_sbs.get():
            if not self.sbs.start():
                tk.messagebox.showerror("Error", "Failed to start SBS publisher")
                self.var_sbs.set(False)
        else:
            self.sbs.stop()
    
    def _update_calc_rate(self, value):
        Config.CALC_THROTTLE_S = 1.0 / float(value)
    
    # =============================================================================
    # CORE LOGIC
    # =============================================================================
    
    def _calculate_gimbal_target(self):
        """Original gimbal target calculation (kept as reference/example)"""
        current_time = time.time()
        if current_time - self.last_calc_time < Config.CALC_THROTTLE_S:
            return self.gimbal_target_state.get('calculation_result')
        
        pitch, yaw = self.gimbal.get_corrected_angles(self.aircraft_state['heading'])
        if pitch is None or yaw is None:
            self.gimbal_target_state['calculation_result'] = None
            return None
        
        if self.last_angles[0] is not None:
            pitch_change = abs(pitch - self.last_angles[0])
            yaw_change = abs(yaw - self.last_angles[1])
            if yaw_change > 180: yaw_change = 360 - yaw_change
            if (pitch_change < Config.ANGLE_CHANGE_THRESHOLD and 
                yaw_change < Config.ANGLE_CHANGE_THRESHOLD):
                return self.gimbal_target_state.get('calculation_result')
        
        self.last_calc_time = current_time
        self.last_angles = (pitch, yaw)
        
        result = TargetCalculator.calculate_target(
            self.aircraft_state['lat'], self.aircraft_state['lon'],
            self.aircraft_state['alt_agl'], pitch, yaw
        )
        if result:
            self.gimbal_target_state['lat'] = result['lat']
            self.gimbal_target_state['lon'] = result['lon']
            self.gimbal_target_state['distance'] = result['distance']
        else:
            self.gimbal_target_state['lat'] = None
            self.gimbal_target_state['lon'] = None
            self.gimbal_target_state['distance'] = 0.0
        self.gimbal_target_state['calculation_result'] = result
        return result
    
    def _update_displays(self):
        if self.mavlink.connected:
            self.lbl_telemetry.config(
                text=f"RX: {Config.MAVLINK_ADDRESS} | TX: {Config.MAVLINK_TX_ADDRESS or '(same link)'} | "
                     f"{self.aircraft_state['lat']:.6f}, {self.aircraft_state['lon']:.6f} | "
                     f"Alt: {self.aircraft_state['alt_agl']:.1f}m | Hdg: {self.aircraft_state['heading']:.1f}°"
            )
        
        if self.gimbal.is_connected:
            pitch, yaw = self.gimbal.get_corrected_angles(self.aircraft_state['heading'])
            raw_yaw = self.gimbal.yaw_abs or 0
            age = time.time() - self.gimbal.last_update
            status = f"P:{pitch:.1f}° Y:{yaw:.1f}° (Raw:{raw_yaw:.1f}°+AC:{self.aircraft_state['heading']:.1f}°) ({age:.1f}s)"
            color = "green" if age < 1.0 else "orange"
            self.lbl_gimbal_status.config(text=status, foreground=color)
            mount_info = f"Mount: {self.gimbal.mount_dir or '—'} | Mode: {self.gimbal.motion_mode or '—'}"
            self.lbl_gimbal_info.config(text=mount_info)
        else:
            self.lbl_gimbal_status.config(text=f"Disconnected ({Config.SIYI_IP}:{Config.SIYI_PORT})", foreground="red")
            self.lbl_gimbal_info.config(text="Mount: — | Mode: —")
        
        # Calculate gimbal target (for reference/example)
        gimbal_result = self._calculate_gimbal_target()
        
        # Update target display based on mode
        self._update_target_display(gimbal_result)
    
    def _update_target_display(self, gimbal_result):
        """Updated target display that shows both modes"""
        self.txt_target.delete(1.0, tk.END)
        
        info = f"=== TARGET MODE: {self.target_mode.upper()} ===\n"
        
        if self.target_mode == "waypoint":
            # Waypoint mission mode
            if not self.google_earth:
                info += "Google Earth integration not available\n"
                info += "Check console for initialization errors\n"
            else:
                status = self.google_earth.get_mission_status()
                
                if status['total_waypoints'] > 0:
                    info += f"Mission: {status['total_waypoints']} waypoints loaded\n"
                    info += f"Progress: {status['visited_waypoints']}/{status['total_waypoints']} ({status['progress_percentage']:.1f}%)\n"
                    
                    if status['current_waypoint']:
                        wp = status['current_waypoint']
                        info += f"Current Target: {wp.name}\n"
                        info += f"Coordinates: {wp.latitude:.7f}, {wp.longitude:.7f}\n"
                        info += f"Altitude: {wp.altitude:.1f}m\n"
                        
                        # Calculate distance to current waypoint
                        if self.aircraft_state['lat'] and self.aircraft_state['lon']:
                            distance = calculate_distance(
                                self.aircraft_state['lat'], self.aircraft_state['lon'],
                                wp.latitude, wp.longitude
                            )
                            info += f"Distance to Waypoint: {distance:.0f}m\n"
                    else:
                        info += "No active waypoint\n"
                    
                    if status['controller_running']:
                        info += f"Mission Status: ACTIVE ({status['tracking_mode']})\n"
                        if self.tracker.active:
                            stats = self.tracker.get_stats()
                            info += f"Tracking: {stats['duration']:.0f}s | {stats['updates']} updates\n"
                            info += f"Next update: {stats['next_update']:.1f}s\n"
                            
                            # Show gimbal lock status
                            lock_info = self.gimbal_locker.get_lock_info()
                            if lock_info['active']:
                                angles = lock_info['required_angles']
                                if angles:
                                    info += f"GIMBAL LOCK: P:{angles['pitch']:.1f}° Y:{angles['yaw']:.1f}° (D:{angles['distance_2d']:.0f}m)\n"
                                else:
                                    info += "GIMBAL LOCK: Calculating angles...\n"
                            elif self.gimbal_lock_var.get():
                                info += "GIMBAL LOCK: Waiting for gimbal connection...\n"
                    else:
                        info += "Mission Status: STOPPED\n"
                        
                    if status['telemetry_feed_active']:
                        info += f"Live KML: {status['kml_output_dir']}\n"
                        
                else:
                    info += "No waypoint mission loaded\n"
                    info += "Load a KML/KMZ file to start waypoint tracking\n"
        
        elif self.target_mode == "fixed":
            if self.fixed_target_state['lat'] is not None:
                # Fixed coordinate mode
                info += f"Fixed Target: {self.fixed_target_state['lat']:.7f}, {self.fixed_target_state['lon']:.7f}\n"
                info += f"Fixed Altitude: {self.fixed_target_state['alt']:.1f}m\n"
                
                # Calculate distance to fixed target
                if self.aircraft_state['lat'] and self.aircraft_state['lon']:
                    distance = calculate_distance(
                        self.aircraft_state['lat'], self.aircraft_state['lon'],
                        self.fixed_target_state['lat'], self.fixed_target_state['lon']
                    )
                    info += f"Distance to Fixed Target: {distance:.0f}m\n"
                
                info += "Status: Fixed coordinate target set\n"
                
                if self.tracker.active:
                    stats = self.tracker.get_stats()
                    info += f"TRACKING FIXED TARGET: {stats['duration']:.0f}s | {stats['updates']} updates\n"
                    info += f"Next update: {stats['next_update']:.1f}s\n"
                    
                    # NEW: Show gimbal lock status
                    lock_info = self.gimbal_locker.get_lock_info()
                    if lock_info['active']:
                        angles = lock_info['required_angles']
                        if angles:
                            info += f"GIMBAL LOCK: P:{angles['pitch']:.1f}° Y:{angles['yaw']:.1f}° (D:{angles['distance_2d']:.0f}m)\n"
                        else:
                            info += "GIMBAL LOCK: Calculating angles...\n"
                    elif self.gimbal_lock_var.get():
                        info += "GIMBAL LOCK: Waiting for gimbal connection...\n"
                else:
                    info += "Ready for fixed coordinate tracking\n"
            else:
                info += "No fixed target set - enter coordinates and click 'Set Fixed Target'\n"
                
        else:
            # Gimbal mode (original behavior)
            if gimbal_result:
                info += f"Gimbal Target: {gimbal_result['lat']:.7f}, {gimbal_result['lon']:.7f}\n"
                info += f"Distance: {gimbal_result['distance']:.0f}m | {gimbal_result['note']}\n"
                info += f"Gimbal: P:{gimbal_result['pitch']:.1f}° Y:{gimbal_result['yaw']:.1f}°\n"
                if self.tracker.active:
                    stats = self.tracker.get_stats()
                    info += f"TRACKING GIMBAL TARGET: {stats['duration']:.0f}s | {stats['updates']} updates\n"
                    info += f"Next update: {stats['next_update']:.1f}s\n"
                    
                    # NEW: Show gimbal lock status for gimbal mode too
                    lock_info = self.gimbal_locker.get_lock_info()
                    if lock_info['active']:
                        angles = lock_info['required_angles']
                        if angles:
                            info += f"GIMBAL LOCK: P:{angles['pitch']:.1f}° Y:{angles['yaw']:.1f}° (D:{angles['distance_2d']:.0f}m)\n"
                        else:
                            info += "GIMBAL LOCK: Calculating angles...\n"
                    elif self.gimbal_lock_var.get():
                        info += "GIMBAL LOCK: Waiting for gimbal connection...\n"
                else:
                    info += "Ready for gimbal tracking\n"
            else:
                info += "No gimbal target - Connect gimbal and point downward\n"
        
        # Always show aircraft state
        info += f"\nAircraft: {self.aircraft_state['lat']:.6f}, {self.aircraft_state['lon']:.6f}\n"
        info += f"Alt: {self.aircraft_state['alt_agl']:.1f}m | Hdg: {self.aircraft_state['heading']:.1f}°\n"
        
        self.txt_target.insert(1.0, info)
    
    # =============================================================================
    # BACKGROUND WORKERS & MAIN LOOP
    # =============================================================================
    
    def _start_workers(self):
        threading.Thread(target=self._sbs_worker, daemon=True).start()
        # also start periodic gimbal attitude poller
        self.root.after(Config.ATTITUDE_REQUEST_MS, self._request_gimbal_attitude)
    
    def _sbs_worker(self):
        while not self._closing:
            try:
                if self.var_sbs.get():
                    target_lat, target_lon, target_alt = self._get_current_target()
                    if target_lat is not None:
                        self.sbs.publish(
                            target_lat, target_lon, self.aircraft_state['alt_amsl']
                        )
                time.sleep(Config.SBS_UPDATE_S)
            except Exception:
                pass
    
    def _main_loop(self):
        if self._closing:
            return
        try:
            if self.mavlink.connected:
                position = self.mavlink.get_position()
                if position:
                    self.aircraft_state.update(position)
                heading = self.mavlink.get_attitude()
                if heading is not None:
                    self.aircraft_state['heading'] = heading
                    
            # Update tracker with current target (works for both modes)
            if self.tracker.active:
                target_lat, target_lon, target_alt = self._get_current_target()
                if target_lat is not None:
                    self.tracker.update_target(target_lat, target_lon, self.aircraft_state['alt_amsl'])
                    
                    # NEW: Update gimbal lock target and aircraft state
                    if self.gimbal_locker.active:
                        lock_target_alt = target_alt if target_alt is not None else 0.0
                        self.gimbal_locker.update_target(target_lat, target_lon, lock_target_alt)
                        self.gimbal_locker.update_aircraft_state(self.aircraft_state)
                    
            self._update_displays()
        except Exception:
            pass
        self.root.after(Config.GUI_UPDATE_MS, self._main_loop)
    
    def _on_close(self):
        self._closing = True
        if self.manual_window and self.manual_window.window.winfo_exists():
            self.manual_window.window.destroy()
        self.tracker.cleanup()
        # NEW: Cleanup gimbal locker
        self.gimbal_locker.cleanup()
        # Cleanup Google Earth integration
        if self.google_earth:
            self.google_earth.cleanup()
        try:
            self.gimbal.stop()
        except Exception:
            pass
        try:
            self.sbs.stop()
        except Exception:
            pass
        if getattr(self.gimbal, "is_connected", False):
            try:
                self.gimbal.jog(0, 0)
            except Exception:
                pass
        self.root.destroy()
    
    def run(self):
        self.root.mainloop()
