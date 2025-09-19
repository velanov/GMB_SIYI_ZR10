#!/usr/bin/env python3
"""
Modern PySide6 GUI for Gimbal GPS V2
Features lateral menu design with dark theme
"""

import sys
import os
import time
import json
import math
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QHBoxLayout, QVBoxLayout, QWidget, 
    QScrollArea, QGroupBox, QLabel, QPushButton, QComboBox, QSpinBox,
    QDoubleSpinBox, QSlider, QCheckBox, QLineEdit, QTextEdit, QFrame,
    QSplitter, QListWidget, QListWidgetItem, QFileDialog
)
from PySide6.QtCore import Qt, QTimer, Signal, QThread
from PySide6.QtGui import QFont, QPalette, QColor, QIcon, QPixmap, QWheelEvent, QMouseEvent

# Import the existing backend systems
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from gimbal_app.shared import *
from gimbal_app.gimbal.siyi_gimbal import SiyiGimbal
from gimbal_app.gimbal.locker import GimbalLocker
from gimbal_app.gimbal.camera_stream import SiyiCameraStream
from gimbal_app.mavlink.handler import MAVLinkHandler
from gimbal_app.adsb.sbs_publisher import SBSPublisher
from gimbal_app.tracking.dynamic_tracker import DynamicTracker
from gimbal_app.calc.target_calculator import TargetCalculator
from gimbal_app.google_earth.controller import GoogleEarthController, GoogleEarthConfig
from gimbal_app.google_earth.waypoint_manager import TrackingMode
# Avoid circular import - import session logger only when needed


class ModernGimbalApp(QMainWindow):
    """Modern PySide6 interface with lateral menu design"""
    
    def __init__(self):
        super().__init__()
        
        # Load persisted settings before creating systems
        SettingsStore.load_into()
        
        # Initialize backend systems (same as original)
        self.init_backend_systems()
        
        # Initialize session logging
        # Import session logger to avoid circular import issues
        from gimbal_app.session_logging.session_logger import get_session_logger
        self.session_logger = get_session_logger()
        print(f"[APP] Session logging initialized: {self.session_logger.session_id}")
        
        # Initialize camera stream placeholder
        self.camera_stream = None
        
        # Setup modern UI first
        self.setup_ui()
        self.setup_dark_theme()
        
        # Initialize camera stream after UI elements exist
        self.init_camera_stream()
        
        # Start backend workers
        self.start_workers()
        
        # Setup update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_displays)
        self.update_timer.start(Config.GUI_UPDATE_MS)
        
        # Setup connection health checker
        self.connection_timer = QTimer()
        self.connection_timer.timeout.connect(self.check_connections)
        self.connection_timer.start(5000)  # Check every 5 seconds
    
    def init_backend_systems(self):
        """Initialize all backend systems (same as tkinter version)"""
        self.gimbal = SiyiGimbal(Config.SIYI_IP, Config.SIYI_PORT)
        self.sbs = SBSPublisher(Config.SBS_BIND, Config.SBS_PORT)
        self.mavlink = MAVLinkHandler(Config.MAVLINK_ADDRESS, Config.MAVLINK_TX_ADDRESS or None)
        self.tracker = DynamicTracker(self.mavlink)
        self.gimbal_locker = GimbalLocker(self.gimbal)
        
        # Notification system
        # self.notification_manager = NotificationManager()  # TODO: Implement NotificationManager
        # self.notification_manager.add_notification_callback(self.display_notification)  # TODO: Implement
        
        # Track previous target for change notifications
        self.previous_target_name = None
        
        # Google Earth integration
        try:
            ge_config = GoogleEarthConfig()
            self.google_earth = GoogleEarthController(ge_config)
            self.setup_google_earth_integration()
        except Exception as e:
            print(f"Warning: Failed to initialize Google Earth integration: {e}")
            self.google_earth = None
        
        # Application state
        self.aircraft_state = {
            'lat': 47.3977508, 'lon': 8.5455938,
            'alt_amsl': 500.0, 'alt_agl': 100.0, 'heading': 0.0
        }
        
        # Target mode and states
        self.target_mode = "gimbal"  # "gimbal", "fixed", or "waypoint"
        
        self.gimbal_target_state = {
            'lat': None, 'lon': None, 'distance': 0.0,
            'calculation_result': None,
            'selected': False  # Track if target was deliberately selected
        }
        
        # Current gimbal pointing (real-time, not for navigation)
        self.gimbal_current_pointing = {
            'lat': None, 'lon': None, 'distance': 0.0
        }
        
        # Gimbal tracking state
        self.gimbal_tracking_active = False
        self.centering_active = False  # Track if centering is in progress
        
        self.fixed_target_state = {
            'lat': None, 'lon': None, 'alt': None
        }
        
        # UI settings
        self.ui_settings = {
            'calc_rate_hz': 10.0,
            'angle_threshold': 0.5,
            'loiter_radius': Config.DEFAULT_LOITER_RADIUS,
            'update_interval': Config.TRACKING_UPDATE_S,
            'min_movement': Config.MIN_MOVEMENT_THRESHOLD
        }
        
        self._closing = False
        
        # Camera stream state
        self.camera_connected = False
    
    def setup_ui(self):
        """Setup the modern lateral menu interface"""
        self.setWindowTitle("Gimbal Control System V2")
        self.setGeometry(100, 100, 1400, 800)  # Optimized for notebook screens
        
        # Main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        # Create lateral menu (left side)
        self.create_lateral_menu()
        
        # Create main content area (right side - for camera view)
        self.create_main_content()
        
        # Add to main layout
        main_layout.addWidget(self.lateral_menu, 0)  # Fixed width
        main_layout.addWidget(self.main_content, 1)  # Expandable
    
    def create_lateral_menu(self):
        """Create the lateral menu with all controls"""
        # Collapsible lateral menu widget
        self.lateral_menu = QWidget()
        self.menu_collapsed = False
        self.menu_width_expanded = 350  # Reduced width
        self.menu_width_collapsed = 50
        self.lateral_menu.setFixedWidth(self.menu_width_expanded)
        self.lateral_menu.setObjectName("lateralMenu")
        
        # Scrollable area for menu content
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        
        # Menu toggle button
        self.menu_toggle_btn = QPushButton("☰")
        self.menu_toggle_btn.setObjectName("menuToggle")
        self.menu_toggle_btn.setFixedSize(40, 40)
        self.menu_toggle_btn.clicked.connect(self.toggle_menu)
        
        menu_content = QWidget()
        self.menu_layout = QVBoxLayout(menu_content)
        self.menu_layout.setContentsMargins(10, 10, 10, 10)
        self.menu_layout.setSpacing(5)  # Reduced spacing
        
        # Add toggle button at top
        self.menu_layout.addWidget(self.menu_toggle_btn, 0, Qt.AlignCenter)
        
        # Container for collapsible content
        self.menu_content_widget = QWidget()
        self.menu_content_layout = QVBoxLayout(self.menu_content_widget)
        self.menu_content_layout.setContentsMargins(0, 0, 0, 0)
        self.menu_content_layout.setSpacing(5)
        
        # Add all sections to collapsible content
        self.create_status_section(self.menu_content_layout)
        self.create_gimbal_control_section(self.menu_content_layout)
        self.create_tracking_section(self.menu_content_layout)
        self.create_status_display_section(self.menu_content_layout)
        
        # Add collapsible content to main menu
        self.menu_layout.addWidget(self.menu_content_widget)
        
        # Add stretch to push everything to top
        self.menu_layout.addStretch()
        
        scroll.setWidget(menu_content)
        
        # Layout for lateral menu
        lateral_layout = QVBoxLayout(self.lateral_menu)
        lateral_layout.setContentsMargins(0, 0, 0, 0)
        lateral_layout.addWidget(scroll)
    
    def create_status_section(self, parent_layout):
        """Create status section: MAVLink, SIYI Gimbal, System Controls"""
        status_group = QGroupBox("System Status")
        status_group.setObjectName("statusGroup")
        status_layout = QVBoxLayout(status_group)
        
        # MAVLink Status
        mavlink_frame = QFrame()
        mavlink_frame.setObjectName("statusFrame")
        mavlink_layout = QVBoxLayout(mavlink_frame)
        
        mavlink_title = QLabel("Telemetry")
        mavlink_title.setObjectName("sectionTitle")
        self.lbl_mavlink_status = QLabel("DISCONNECTED")
        self.lbl_mavlink_status.setObjectName("statusLabel")
        self.lbl_telemetry_details = QLabel("Waiting for telemetry...")
        self.lbl_telemetry_details.setObjectName("detailsLabel")
        
        mavlink_layout.addWidget(mavlink_title)
        mavlink_layout.addWidget(self.lbl_mavlink_status)
        mavlink_layout.addWidget(self.lbl_telemetry_details)
        
        # SIYI Gimbal Status
        gimbal_frame = QFrame()
        gimbal_frame.setObjectName("statusFrame")
        gimbal_layout = QVBoxLayout(gimbal_frame)
        
        gimbal_title = QLabel("Gimbal")
        gimbal_title.setObjectName("sectionTitle")
        self.chk_gimbal_connect = QCheckBox(f"Connect SIYI ({Config.SIYI_IP}:{Config.SIYI_PORT})")
        self.chk_gimbal_connect.toggled.connect(self.toggle_gimbal)
        self.lbl_gimbal_status = QLabel("DISCONNECTED")
        self.lbl_gimbal_status.setObjectName("statusLabel")
        self.lbl_gimbal_details = QLabel("Mount: — | Mode: —")
        self.lbl_gimbal_details.setObjectName("detailsLabel")
        
        gimbal_layout.addWidget(gimbal_title)
        gimbal_layout.addWidget(self.chk_gimbal_connect)
        gimbal_layout.addWidget(self.lbl_gimbal_status)
        gimbal_layout.addWidget(self.lbl_gimbal_details)
        
        # System Controls
        controls_frame = QFrame()
        controls_frame.setObjectName("statusFrame")
        controls_layout = QVBoxLayout(controls_frame)
        
        controls_title = QLabel("Controls")
        controls_title.setObjectName("sectionTitle")
        
        # SBS Publisher section
        self.chk_sbs_publisher = QCheckBox(f"SBS Publisher ({Config.SBS_BIND}:{Config.SBS_PORT})")
        self.chk_sbs_publisher.toggled.connect(self.toggle_sbs)
        
        self.lbl_sbs_status = QLabel("ADS-B: STOPPED")
        self.lbl_sbs_status.setObjectName("detailsLabel")
        
        controls_layout.addWidget(controls_title)
        controls_layout.addWidget(self.chk_sbs_publisher)
        controls_layout.addWidget(self.lbl_sbs_status)
        
        # Add all frames to status section
        status_layout.addWidget(mavlink_frame)
        status_layout.addWidget(gimbal_frame)
        status_layout.addWidget(controls_frame)
        
        parent_layout.addWidget(status_group)
    
    def create_gimbal_control_section(self, parent_layout):
        """Create gimbal control section"""
        gimbal_group = QGroupBox("Gimbal Control")
        gimbal_group.setObjectName("controlGroup")
        gimbal_layout = QVBoxLayout(gimbal_group)
        
        # Speed control
        speed_frame = QFrame()
        speed_layout = QHBoxLayout(speed_frame)
        speed_layout.addWidget(QLabel("Speed:"))
        
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(1, 100)
        self.speed_slider.setValue(20)
        self.speed_label = QLabel("20")
        self.speed_slider.valueChanged.connect(lambda v: self.speed_label.setText(str(v)))
        
        speed_layout.addWidget(self.speed_slider)
        speed_layout.addWidget(self.speed_label)
        
        # Joystick-style D-pad controls
        controls_frame = QFrame()
        controls_frame.setMinimumHeight(180)
        controls_frame.setObjectName("joystickFrame")
        
        # Use responsive grid layout for D-pad
        controls_layout = QVBoxLayout(controls_frame)
        controls_layout.setContentsMargins(10, 10, 10, 10)
        
        # Top row with YAW label
        top_row = QHBoxLayout()
        top_row.addStretch()
        yaw_label = QLabel("YAW")
        yaw_label.setObjectName("axisLabel")
        yaw_label.setAlignment(Qt.AlignCenter)
        top_row.addWidget(yaw_label)
        top_row.addStretch()
        controls_layout.addLayout(top_row)
        
        # Up button row
        up_row = QHBoxLayout()
        up_row.addStretch()
        self.btn_up = QPushButton("▲")
        self.btn_up.setObjectName("dpadButton")
        up_row.addWidget(self.btn_up)
        up_row.addStretch()
        controls_layout.addLayout(up_row)
        
        # Middle row with left, center, right
        middle_row = QHBoxLayout()
        middle_row.addStretch()
        
        self.btn_left = QPushButton("◄")
        self.btn_left.setObjectName("dpadButton")
        middle_row.addWidget(self.btn_left)
        
        self.btn_center = QPushButton("●")
        self.btn_center.setObjectName("centerButton")
        middle_row.addWidget(self.btn_center)
        
        self.btn_right = QPushButton("►")
        self.btn_right.setObjectName("dpadButton")
        middle_row.addWidget(self.btn_right)
        
        middle_row.addStretch()
        controls_layout.addLayout(middle_row)
        
        # Down button row
        down_row = QHBoxLayout()
        down_row.addStretch()
        self.btn_down = QPushButton("▼")
        self.btn_down.setObjectName("dpadButton")
        down_row.addWidget(self.btn_down)
        down_row.addStretch()
        controls_layout.addLayout(down_row)
        
        # Zoom controls row
        zoom_row = QHBoxLayout()
        zoom_row.addStretch()
        
        self.btn_zoom_out = QPushButton("🔍➖")
        self.btn_zoom_out.setObjectName("zoomButton")
        self.btn_zoom_out.setToolTip("Zoom Out")
        zoom_row.addWidget(self.btn_zoom_out)
        
        zoom_label = QLabel("ZOOM")
        zoom_label.setObjectName("axisLabel")
        zoom_label.setAlignment(Qt.AlignCenter)
        zoom_label.setMinimumWidth(60)
        zoom_row.addWidget(zoom_label)
        
        self.btn_zoom_in = QPushButton("🔍➕")
        self.btn_zoom_in.setObjectName("zoomButton")
        self.btn_zoom_in.setToolTip("Zoom In")
        zoom_row.addWidget(self.btn_zoom_in)
        
        zoom_row.addStretch()
        controls_layout.addLayout(zoom_row)
        
        # Bottom row with PITCH label
        bottom_row = QHBoxLayout()
        bottom_row.addStretch()
        pitch_label = QLabel("PITCH")
        pitch_label.setObjectName("axisLabel")
        pitch_label.setAlignment(Qt.AlignCenter)
        bottom_row.addWidget(pitch_label)
        bottom_row.addStretch()
        controls_layout.addLayout(bottom_row)
        
        controls_layout.addStretch()
        
        # Connect button events
        self.setup_gimbal_button_events()
        
        gimbal_layout.addWidget(speed_frame)
        gimbal_layout.addWidget(controls_frame)
        
        parent_layout.addWidget(gimbal_group)
    
    def create_target_mode_section(self, parent_layout):
        """Create target mode selection section with dropdown"""
        target_group = QGroupBox("Target Mode")
        target_group.setObjectName("controlGroup")
        target_layout = QVBoxLayout(target_group)
        
        # Mode selection moved to camera area - remove this duplicate
        target_layout.addLayout(mode_layout)
        
        # Stacked widget for mode-specific controls
        self.mode_stack = QWidget()
        self.mode_stack_layout = QVBoxLayout(self.mode_stack)
        
        # Create mode-specific panels (moved to camera section where layout exists)
        # self.create_gimbal_pointing_panel()  # Moved to camera setup
        # self.create_fixed_coordinate_panel()  # Moved to camera setup
        # self.create_waypoint_mission_panel()  # Moved to camera setup
        
        target_layout.addWidget(self.mode_stack)
        parent_layout.addWidget(target_group)
        
        # Set initial mode
        self.on_target_mode_changed("gimbal")
    
    def create_gimbal_pointing_panel(self):
        """Create gimbal pointing control panel"""
        self.gimbal_panel = QFrame()
        self.gimbal_panel.setObjectName("modePanel")
        gimbal_layout = QHBoxLayout(self.gimbal_panel)
        
        # Current pointing display (read-only)
        gimbal_layout.addWidget(QLabel("Aiming:"))
        self.lbl_current_pointing = QLabel("---.------, ---.------")
        self.lbl_current_pointing.setObjectName("coordinateLabel") 
        self.lbl_current_pointing.setMinimumWidth(180)
        gimbal_layout.addWidget(self.lbl_current_pointing)
        
        # Start Target Tracking button
        self.btn_start_gimbal_tracking = QPushButton("START TARGET TRACKING")
        self.btn_start_gimbal_tracking.setObjectName("primaryButton")
        self.btn_start_gimbal_tracking.setToolTip("Start loitering around current gimbal target")
        self.btn_start_gimbal_tracking.clicked.connect(self.start_gimbal_tracking)
        gimbal_layout.addWidget(self.btn_start_gimbal_tracking)
        
        # Stop Target Tracking button
        self.btn_stop_gimbal_tracking = QPushButton("STOP TRACKING")
        self.btn_stop_gimbal_tracking.setObjectName("actionButton")
        self.btn_stop_gimbal_tracking.setToolTip("Unlock gimbal but continue loitering")
        self.btn_stop_gimbal_tracking.clicked.connect(self.stop_gimbal_tracking)
        self.btn_stop_gimbal_tracking.setEnabled(False)
        gimbal_layout.addWidget(self.btn_stop_gimbal_tracking)
        
        # Select Target button
        self.btn_select_target = QPushButton("SELECT TARGET")
        self.btn_select_target.setObjectName("actionButton")
        self.btn_select_target.setToolTip("Lock gimbal on current target")
        self.btn_select_target.clicked.connect(self.select_gimbal_target)
        gimbal_layout.addWidget(self.btn_select_target)
        
        # Clear target button  
        self.btn_clear_target = QPushButton("CLEAR")
        self.btn_clear_target.setObjectName("actionButton")
        self.btn_clear_target.setToolTip("Clear selected target")
        self.btn_clear_target.clicked.connect(self.clear_gimbal_target)
        gimbal_layout.addWidget(self.btn_clear_target)
        
        # Selected target display
        gimbal_layout.addWidget(QLabel("Target:"))
        self.lbl_selected_target = QLabel("None selected")
        self.lbl_selected_target.setObjectName("detailsLabel")
        self.lbl_selected_target.setMinimumWidth(150)
        gimbal_layout.addWidget(self.lbl_selected_target)
        
        # Add to mode controls frame
        self.mode_controls_layout.addWidget(self.gimbal_panel)
        self.gimbal_panel.hide()  # Initially hidden
    
    def create_fixed_coordinate_panel(self):
        """Create fixed coordinate input panel"""
        self.fixed_panel = QFrame()
        self.fixed_panel.setObjectName("modePanel")
        fixed_layout = QHBoxLayout(self.fixed_panel)  # Horizontal for top placement
        
        # Compact coordinate inputs in horizontal layout
        fixed_layout.addWidget(QLabel("Lat:"))
        self.fixed_lat_input = QLineEdit("47.3977508")
        self.fixed_lat_input.setMaximumWidth(100)
        fixed_layout.addWidget(self.fixed_lat_input)
        
        fixed_layout.addWidget(QLabel("Lon:"))
        self.fixed_lon_input = QLineEdit("8.5455938")
        self.fixed_lon_input.setMaximumWidth(100)
        fixed_layout.addWidget(self.fixed_lon_input)
        
        fixed_layout.addWidget(QLabel("Alt:"))
        self.fixed_alt_input = QSpinBox()
        self.fixed_alt_input.setRange(0, 10000)
        self.fixed_alt_input.setValue(500)
        self.fixed_alt_input.setSuffix("m")
        self.fixed_alt_input.setMaximumWidth(80)
        fixed_layout.addWidget(self.fixed_alt_input)
        
        # Compact buttons
        self.btn_set_fixed = QPushButton("SET")
        self.btn_set_fixed.setObjectName("actionButton")
        self.btn_set_fixed.clicked.connect(self.set_fixed_target)
        fixed_layout.addWidget(self.btn_set_fixed)
        
        self.btn_use_aircraft = QPushButton("USE AC")
        self.btn_use_aircraft.setObjectName("actionButton")
        self.btn_use_aircraft.clicked.connect(self.use_aircraft_position)
        fixed_layout.addWidget(self.btn_use_aircraft)
        
        fixed_layout.addStretch()
        
        # Add to mode controls frame
        self.mode_controls_layout.addWidget(self.fixed_panel)
        
        self.fixed_panel.hide()  # Initially hidden
    
    def create_waypoint_mission_panel(self):
        """Create compact waypoint mission panel"""
        self.waypoint_panel = QFrame()
        self.waypoint_panel.setObjectName("modePanel")
        waypoint_layout = QHBoxLayout(self.waypoint_panel)  # Horizontal for top placement
        
        # Compact mission controls
        self.btn_load_kml = QPushButton("LOAD KML")
        self.btn_load_kml.setObjectName("actionButton")
        self.btn_load_kml.clicked.connect(self.load_kml_mission)
        waypoint_layout.addWidget(self.btn_load_kml)
        
        waypoint_layout.addWidget(QLabel("Mission:"))
        self.missions_combo = QComboBox()
        self.missions_combo.addItem("No missions loaded")
        self.missions_combo.setMaximumWidth(120)
        waypoint_layout.addWidget(self.missions_combo)
        
        # Delete mission button
        self.btn_delete_mission = QPushButton("✖")
        self.btn_delete_mission.setObjectName("actionButton")
        self.btn_delete_mission.setMaximumWidth(25)
        self.btn_delete_mission.setToolTip("Delete selected mission")
        self.btn_delete_mission.clicked.connect(self.delete_mission)
        waypoint_layout.addWidget(self.btn_delete_mission)
        
        waypoint_layout.addWidget(QLabel("Mode:"))
        self.waypoint_mode_combo = QComboBox()
        self.waypoint_mode_combo.addItem("Sequential", "sequential")
        self.waypoint_mode_combo.addItem("Manual Select", "manual_select")
        self.waypoint_mode_combo.addItem("Loiter All", "loiter_all")
        self.waypoint_mode_combo.setMaximumWidth(120)
        waypoint_layout.addWidget(self.waypoint_mode_combo)
        
        self.btn_start_mission = QPushButton("START")
        self.btn_start_mission.setObjectName("actionButton")
        self.btn_start_mission.clicked.connect(self.start_waypoint_mission)
        waypoint_layout.addWidget(self.btn_start_mission)
        
        self.btn_stop_mission = QPushButton("STOP")
        self.btn_stop_mission.setObjectName("actionButton")
        self.btn_stop_mission.clicked.connect(self.stop_waypoint_mission)
        waypoint_layout.addWidget(self.btn_stop_mission)
        
        # Mission status
        self.lbl_mission_status = QLabel("No mission")
        self.lbl_mission_status.setObjectName("detailsLabel")
        self.lbl_mission_status.setMaximumWidth(150)
        waypoint_layout.addWidget(self.lbl_mission_status)
        
        # Waypoint dropdown with radius control
        waypoint_layout.addWidget(QLabel("Target:"))
        
        # Create waypoint dropdown
        self.waypoints_combo = QComboBox()
        self.waypoints_combo.addItem("No waypoints loaded")
        self.waypoints_combo.currentIndexChanged.connect(self.on_waypoint_combo_changed)
        self.waypoints_combo.setMaximumWidth(200)
        waypoint_layout.addWidget(self.waypoints_combo)
        
        # Loiter radius control
        radius_layout = QHBoxLayout()
        radius_layout.addWidget(QLabel("Radius:"))
        
        self.waypoint_radius_spin = QSpinBox()
        self.waypoint_radius_spin.setRange(10, 1000)
        self.waypoint_radius_spin.setValue(100)
        self.waypoint_radius_spin.setSuffix("m")
        self.waypoint_radius_spin.setMaximumWidth(80)
        self.waypoint_radius_spin.valueChanged.connect(self.on_waypoint_radius_changed)
        radius_layout.addWidget(self.waypoint_radius_spin)
        
        radius_layout.addStretch()
        waypoint_layout.addLayout(radius_layout)
        
        # Navigation buttons
        nav_layout = QHBoxLayout()
        
        self.btn_prev_waypoint = QPushButton("◀ PREV")
        self.btn_prev_waypoint.setObjectName("actionButton")
        self.btn_prev_waypoint.clicked.connect(self.prev_waypoint)
        self.btn_prev_waypoint.setMaximumWidth(70)
        nav_layout.addWidget(self.btn_prev_waypoint)
        
        self.btn_next_waypoint = QPushButton("NEXT ▶")
        self.btn_next_waypoint.setObjectName("actionButton")
        self.btn_next_waypoint.clicked.connect(self.next_waypoint)
        self.btn_next_waypoint.setMaximumWidth(70)
        nav_layout.addWidget(self.btn_next_waypoint)
        
        self.lbl_current_waypoint = QLabel("WP: 0/0")
        self.lbl_current_waypoint.setObjectName("detailsLabel")
        nav_layout.addWidget(self.lbl_current_waypoint)
        
        nav_layout.addStretch()
        waypoint_layout.addLayout(nav_layout)
        
        waypoint_layout.addStretch()
        
        # Initialize mission storage
        self.stored_missions = {}  # Dict to store loaded missions
        self.missions_file = "saved_missions.json"  # Persistent storage file
        self.waypoint_data = []  # Store current waypoint objects
        
        # Load saved missions from file
        self.load_saved_missions()
        
        # Add to mode controls frame
        self.mode_controls_layout.addWidget(self.waypoint_panel)
        
        self.waypoint_panel.hide()  # Initially hidden
    
    def create_tracking_section(self, parent_layout):
        """Create dynamic tracking section"""
        tracking_group = QGroupBox("Dynamic Tracking")
        tracking_group.setObjectName("controlGroup")
        tracking_layout = QVBoxLayout(tracking_group)
        
        # Parameters
        params_frame = QFrame()
        params_layout = QVBoxLayout(params_frame)
        
        # Radius
        radius_layout = QHBoxLayout()
        radius_layout.addWidget(QLabel("Radius:"))
        self.radius_spin = QSpinBox()
        self.radius_spin.setRange(50, 500)
        self.radius_spin.setValue(Config.DEFAULT_LOITER_RADIUS)
        self.radius_spin.setSuffix(" m")
        radius_layout.addWidget(self.radius_spin)
        
        # Update interval
        update_layout = QHBoxLayout()
        update_layout.addWidget(QLabel("Update:"))
        self.update_spin = QSpinBox()
        self.update_spin.setRange(5, 60)
        self.update_spin.setValue(Config.TRACKING_UPDATE_S)
        self.update_spin.setSuffix(" s")
        update_layout.addWidget(self.update_spin)
        
        # Movement distance
        movement_layout = QHBoxLayout()
        movement_layout.addWidget(QLabel("Movement:"))
        self.movement_spin = QSpinBox()
        self.movement_spin.setRange(10, 200)
        self.movement_spin.setValue(Config.MIN_MOVEMENT_THRESHOLD)
        self.movement_spin.setSuffix(" m")
        movement_layout.addWidget(self.movement_spin)
        
        # Gimbal lock
        self.chk_gimbal_lock = QCheckBox("Enable Gimbal Lock")
        self.chk_gimbal_lock.setChecked(True)
        
        params_layout.addLayout(radius_layout)
        params_layout.addLayout(update_layout)
        params_layout.addLayout(movement_layout)
        params_layout.addWidget(self.chk_gimbal_lock)
        
        # Control buttons
        buttons_layout = QVBoxLayout()
        
        self.btn_start_tracking = QPushButton("ENGAGE")
        self.btn_start_tracking.setObjectName("primaryButton")
        self.btn_start_tracking.clicked.connect(self.start_tracking)
        
        self.btn_stop_tracking = QPushButton("DISENGAGE")
        self.btn_stop_tracking.setObjectName("actionButton")
        self.btn_stop_tracking.clicked.connect(self.stop_tracking)
        self.btn_stop_tracking.setEnabled(False)
        
        self.btn_single_goto = QPushButton("GOTO")
        self.btn_single_goto.setObjectName("actionButton")
        self.btn_single_goto.clicked.connect(self.single_goto)
        
        self.btn_return_mission = QPushButton("RTM")
        self.btn_return_mission.setObjectName("actionButton")
        self.btn_return_mission.clicked.connect(self.return_to_mission)
        
        buttons_layout.addWidget(self.btn_start_tracking)
        buttons_layout.addWidget(self.btn_stop_tracking)
        buttons_layout.addWidget(self.btn_single_goto)
        buttons_layout.addWidget(self.btn_return_mission)
        
        tracking_layout.addWidget(params_frame)
        tracking_layout.addLayout(buttons_layout)
        
        parent_layout.addWidget(tracking_group)
    
    def create_status_display_section(self, parent_layout):
        """Create compact status display section"""
        status_group = QGroupBox("System Status")
        status_group.setObjectName("controlGroup")
        status_layout = QVBoxLayout(status_group)
        
        # Compact status display
        self.compact_status = QTextEdit()
        self.compact_status.setObjectName("compactStatus")
        self.compact_status.setMaximumHeight(200)
        self.compact_status.setReadOnly(True)
        
        status_layout.addWidget(self.compact_status)
        parent_layout.addWidget(status_group)
    
    def display_notification(self, notification):
        """Display notification in the UI - placeholder for compatibility"""
        timestamp = notification['timestamp']
        message = notification['message']
        severity = notification['severity']
        
        # Military-style severity prefixes
        severity_prefixes = {
            'info': '[INFO]',
            'success': '[CONFIRM]',
            'warning': '[CAUTION]',
            'alert': '[ALERT]',
            'critical': '[PRIORITY]',
            'error': '[ERROR]'
        }
        
        prefix = severity_prefixes.get(severity, '[INFO]')
        formatted_message = f"{timestamp} {prefix} {message}"
        
        # For now, just print to console (can be connected to UI element later)
        print(f"[NOTIFICATION] {formatted_message}")
    
    def get_current_target_name(self):
        """Get descriptive name for current target"""
        if self.target_mode == "waypoint":
            try:
                if hasattr(self, 'waypoints_combo') and self.waypoints_combo.currentText():
                    waypoint_name = self.waypoints_combo.currentText()
                    return f"WP-{waypoint_name}"
                else:
                    return "WP-UNKNOWN"
            except:
                return "WP-UNKNOWN"
        elif self.target_mode == "fixed":
            if self.fixed_target_state.get('lat') and self.fixed_target_state.get('lon'):
                lat = self.fixed_target_state['lat']
                lon = self.fixed_target_state['lon']
                return f"FIXED-{lat:.4f},{lon:.4f}"
            else:
                return "FIXED-UNSET"
        elif self.target_mode == "gimbal":
            if self.gimbal_target_state.get('lat') and self.gimbal_target_state.get('lon'):
                lat = self.gimbal_target_state['lat']
                lon = self.gimbal_target_state['lon']
                return f"GIMBAL-{lat:.4f},{lon:.4f}"
            else:
                return "GIMBAL-UNSET"
        else:
            return "UNKNOWN"
    
    def create_main_content(self):
        """Create main content area (for camera view)"""
        self.main_content = QFrame()
        self.main_content.setObjectName("mainContent")
        
        main_layout = QVBoxLayout(self.main_content)
        main_layout.setContentsMargins(20, 20, 20, 20)
        
        # Camera view placeholder
        camera_frame = QFrame()
        camera_frame.setObjectName("cameraFrame")
        camera_frame.setMinimumHeight(550)  # Ensure adequate space for camera
        
        camera_layout = QVBoxLayout(camera_frame)
        
        # Mode selection at top of camera area
        mode_frame = QFrame()
        mode_frame.setObjectName("modeFrame")
        mode_layout = QHBoxLayout(mode_frame)
        mode_layout.setContentsMargins(10, 5, 10, 5)
        
        QLabel("Mode:").setObjectName("modeLabel")
        mode_layout.addWidget(QLabel("Mode:"))
        
        self.target_mode_combo = QComboBox()
        self.target_mode_combo.addItem("Gimbal Pointing", "gimbal")
        self.target_mode_combo.addItem("Fixed Coordinate", "fixed")
        self.target_mode_combo.addItem("Waypoint Mission", "waypoint")
        self.target_mode_combo.currentTextChanged.connect(lambda text: self.on_target_mode_changed(self.target_mode_combo.currentData()))
        mode_layout.addWidget(self.target_mode_combo)
        
        mode_layout.addStretch()
        
        # Camera controls in same row
        self.btn_camera_connect = QPushButton("CONNECT CAM")
        self.btn_camera_connect.setObjectName("actionButton")
        self.btn_camera_connect.clicked.connect(self.toggle_camera_stream)
        mode_layout.addWidget(self.btn_camera_connect)
        
        self.lbl_camera_status = QLabel("CAMERA: DISCONNECTED")
        self.lbl_camera_status.setObjectName("detailsLabel")
        mode_layout.addWidget(self.lbl_camera_status)
        
        # Camera video display with proper aspect ratio
        self.camera_display = QLabel("CAMERA FEED\n[CONNECTING...]")
        self.camera_display.setObjectName("cameraDisplay")
        self.camera_display.setAlignment(Qt.AlignCenter)
        self.camera_display.setMinimumSize(640, 480)  # Set reasonable minimum size
        self.camera_display.setScaledContents(False)  # Don't stretch content
        
        
        # Create a container for camera
        self.camera_container = QFrame()
        self.camera_container.setObjectName("cameraContainer")
        camera_container_layout = QVBoxLayout(self.camera_container)
        camera_container_layout.setContentsMargins(0, 0, 0, 0)  # No margins for full scaling
        camera_container_layout.addWidget(self.camera_display)
        
        # Set proper size policy for camera display
        from PySide6.QtWidgets import QSizePolicy
        self.camera_display.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # Mode-specific controls frame (for fixed coordinates, waypoint loading, etc)
        self.mode_controls_frame = QFrame()
        self.mode_controls_frame.setObjectName("modeControlsFrame")
        self.mode_controls_layout = QVBoxLayout(self.mode_controls_frame)
        self.mode_controls_layout.setContentsMargins(10, 5, 10, 5)
        
        # Create mode-specific panels
        self.create_gimbal_pointing_panel()
        self.create_fixed_coordinate_panel()
        self.create_waypoint_mission_panel()
        
        # Initially hide mode controls
        self.mode_controls_frame.hide()
        
        camera_layout.addWidget(mode_frame)
        camera_layout.addWidget(self.mode_controls_frame)
        camera_layout.addWidget(self.camera_container, 1)  # Give camera container stretch factor
        
        # Target information display
        self.target_info = QTextEdit()
        self.target_info.setObjectName("targetInfo")
        self.target_info.setMaximumHeight(200)
        self.target_info.setReadOnly(True)
        
        main_layout.addWidget(camera_frame)  # Only camera in main content now
    
    def setup_dark_theme(self):
        """Setup modern dark theme similar to reference image"""
        self.setStyleSheet("""
        /* Main window */
        QMainWindow {
            background-color: #1a1a1a;
            color: #e0e0e0;
            font-family: 'Consolas', 'Courier New', monospace;
        }
        
        /* Lateral menu */
        #lateralMenu {
            background-color: #2a2a2a;
            border-right: 3px solid #404040;
        }
        
        /* Group boxes */
        QGroupBox {
            font-weight: bold;
            font-size: 14px;
            color: #e0e0e0;
            border: 2px solid #404040;
            border-radius: 4px;
            margin-top: 10px;
            padding-top: 10px;
            text-transform: uppercase;
            letter-spacing: 1px;
        }
        
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 8px 0 8px;
            background-color: #2a2a2a;
            color: #b0b0b0;
        }
        
        #statusGroup {
            background-color: #333333;
            border-color: #606060;
        }
        
        #controlGroup {
            background-color: #333333;
            border-color: #606060;
        }
        
        /* Status frames */
        #statusFrame {
            background-color: #3a3a3a;
            border: 1px solid #505050;
            border-radius: 2px;
            padding: 8px;
            margin: 4px;
        }
        
        #modePanel {
            background-color: #3a3a3a;
            border: 1px solid #505050;
            border-radius: 2px;
            padding: 8px;
            margin: 4px;
        }
        
        /* Labels */
        #sectionTitle {
            font-weight: bold;
            font-size: 13px;
            color: #90c090;
            margin-bottom: 5px;
            text-transform: uppercase;
            letter-spacing: 1px;
        }
        
        #statusLabel {
            font-weight: bold;
            color: #e0e0e0;
            font-family: 'Consolas', 'Courier New', monospace;
        }
        
        #detailsLabel {
            font-size: 12px;
            color: #b0b0b0;
            font-family: 'Consolas', 'Courier New', monospace;
        }
        
        #mainTitle {
            font-size: 18px;
            font-weight: bold;
            color: #90c090;
            margin: 10px;
            text-transform: uppercase;
            letter-spacing: 2px;
            font-family: 'Consolas', 'Courier New', monospace;
        }
        
        #placeholder {
            font-size: 14px;
            color: #808080;
            background-color: #222222;
            border: 2px dashed #404040;
            border-radius: 4px;
            padding: 20px;
            font-family: 'Consolas', 'Courier New', monospace;
            text-transform: uppercase;
        }
        
        /* Buttons */
        QPushButton {
            background-color: #404040;
            border: 2px solid #606060;
            border-radius: 2px;
            padding: 6px 12px;
            color: #e0e0e0;
            font-weight: bold;
            font-family: 'Consolas', 'Courier New', monospace;
            text-transform: uppercase;
            letter-spacing: 1px;
            font-size: 12px;
        }
        
        QPushButton:hover {
            background-color: #505050;
            border-color: #90c090;
        }
        
        QPushButton:pressed {
            background-color: #303030;
        }
        
        #controlButton {
            background-color: #4a4a4a;
            min-width: 80px;
            min-height: 35px;
            border-color: #707070;
        }
        
        #centerButton {
            background-color: #606060;
            color: #e0e0e0;
            min-height: 40px;
            border-color: #808080;
        }
        
        #centerButton:hover {
            background-color: #707070;
        }
        
        #actionButton {
            background-color: #4a5a4a;
            border-color: #6a7a6a;
        }
        
        #actionButton:hover {
            background-color: #5a6a5a;
        }
        
        #primaryButton {
            background-color: #6a5a4a;
            min-height: 35px;
            border-color: #8a7a6a;
        }
        
        #primaryButton:hover {
            background-color: #7a6a5a;
        }
        
        /* Joystick-style D-pad buttons */
        #joystickFrame {
            background-color: #2a2a2a;
            border: 2px solid #404040;
            border-radius: 8px;
        }
        
        #dpadButton {
            background: qradialgradient(cx: 0.5, cy: 0.5, radius: 0.5,
                stop: 0 #505050, stop: 0.8 #404040, stop: 1 #303030);
            border: 2px solid #606060;
            border-radius: 15px;
            font-size: 14px;
            font-weight: bold;
            color: #00ff00;
            min-width: 30px;
            min-height: 30px;
            max-width: 30px;
            max-height: 30px;
        }
        
        #dpadButton:hover {
            background: qradialgradient(cx: 0.5, cy: 0.5, radius: 0.5,
                stop: 0 #606060, stop: 0.8 #505050, stop: 1 #404040);
            border-color: #00ff00;
            color: #00ff00;
        }
        
        #dpadButton:pressed {
            background: qradialgradient(cx: 0.5, cy: 0.5, radius: 0.5,
                stop: 0 #707070, stop: 0.8 #606060, stop: 1 #505050);
            border-color: #00cc00;
            color: #00cc00;
        }
        
        #centerButton {
            background: qradialgradient(cx: 0.5, cy: 0.5, radius: 0.5,
                stop: 0 #606060, stop: 0.8 #505050, stop: 1 #404040);
            border: 2px solid #808080;
            border-radius: 15px;
            font-size: 16px;
            font-weight: bold;
            color: #ffaa00;
            min-width: 30px;
            min-height: 30px;
            max-width: 30px;
            max-height: 30px;
        }
        
        #centerButton:hover {
            background: qradialgradient(cx: 0.5, cy: 0.5, radius: 0.5,
                stop: 0 #707070, stop: 0.8 #606060, stop: 1 #505050);
            border-color: #ffaa00;
            color: #ffaa00;
        }
        
        #centerButton:pressed {
            background: qradialgradient(cx: 0.5, cy: 0.5, radius: 0.5,
                stop: 0 #808080, stop: 0.8 #707070, stop: 1 #606060);
            border-color: #ff8800;
            color: #ff8800;
        }
        
        #zoomButton {
            background: qradialgradient(cx: 0.5, cy: 0.5, radius: 0.5,
                stop: 0 #404060, stop: 0.8 #303050, stop: 1 #202040);
            border: 2px solid #4080ff;
            border-radius: 12px;
            font-size: 12px;
            font-weight: bold;
            color: #4080ff;
            min-width: 40px;
            min-height: 25px;
            max-width: 40px;
            max-height: 25px;
        }
        
        #zoomButton:hover {
            background: qradialgradient(cx: 0.5, cy: 0.5, radius: 0.5,
                stop: 0 #5070a0, stop: 0.8 #406090, stop: 1 #305080);
            border-color: #60a0ff;
            color: #60a0ff;
        }
        
        #zoomButton:pressed {
            background: qradialgradient(cx: 0.5, cy: 0.5, radius: 0.5,
                stop: 0 #6080b0, stop: 0.8 #5070a0, stop: 1 #406090);
            border-color: #80c0ff;
            color: #80c0ff;
        }
        
        #axisLabel {
            color: #808080;
            font-size: 10px;
            font-weight: bold;
            background: transparent;
            border: none;
        }
        
        /* Cyberpunk Status Panel */
        #unifiedStatusGroup {
            background-color: #1a1a1a;
            border: 2px solid #00ff00;
            border-radius: 4px;
            color: #00ff00;
            font-weight: bold;
            text-transform: uppercase;
            letter-spacing: 1px;
        }
        
        #unifiedStatusGroup::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 8px 0 8px;
            background-color: #1a1a1a;
            color: #00ff00;
        }
        
        #cyberpunkStatusFrame {
            background-color: #001100;
            border: 1px solid #00aa00;
            border-radius: 2px;
            padding: 8px;
        }
        
        #cyberpunkStatusLabel {
            color: #00ff00;
            font-weight: bold;
            font-family: 'Consolas', 'Courier New', monospace;
            font-size: 12px;
        }
        
        #cyberpunkDetailsLabel {
            color: #00aa00;
            font-family: 'Consolas', 'Courier New', monospace;
            font-size: 10px;
        }
        
        #cyberpunkCheckbox {
            color: #00cc00;
            font-family: 'Consolas', 'Courier New', monospace;
            font-size: 11px;
            spacing: 8px;
        }
        
        #cyberpunkCheckbox::indicator {
            width: 12px;
            height: 12px;
        }
        
        #cyberpunkCheckbox::indicator:unchecked {
            background-color: #003300;
            border: 1px solid #00aa00;
        }
        
        #cyberpunkCheckbox::indicator:checked {
            background-color: #00ff00;
            border: 1px solid #00ff00;
        }
        
        /* Input widgets */
        QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {
            background-color: #404040;
            border: 2px solid #606060;
            border-radius: 2px;
            padding: 5px;
            color: #e0e0e0;
            font-family: 'Consolas', 'Courier New', monospace;
            font-size: 11px;
        }
        
        QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {
            border-color: #90c090;
        }
        
        /* Checkboxes */
        QCheckBox {
            color: #e0e0e0;
            spacing: 8px;
            font-family: 'Consolas', 'Courier New', monospace;
            font-size: 11px;
        }
        
        QCheckBox::indicator {
            width: 16px;
            height: 16px;
            border: 2px solid #606060;
            border-radius: 1px;
            background-color: #404040;
        }
        
        QCheckBox::indicator:checked {
            background-color: #90c090;
            border-color: #90c090;
        }
        
        /* Sliders */
        QSlider::groove:horizontal {
            height: 6px;
            background-color: #666666;
            border-radius: 3px;
        }
        
        QSlider::handle:horizontal {
            background-color: #00aaff;
            border: 2px solid #0099ff;
            width: 16px;
            margin: -5px 0;
            border-radius: 8px;
        }
        
        QSlider::sub-page:horizontal {
            background-color: #00aaff;
            border-radius: 3px;
        }
        
        /* List widgets */
        QListWidget {
            background-color: #505050;
            border: 2px solid #666666;
            border-radius: 4px;
            color: #ffffff;
        }
        
        QListWidget::item {
            padding: 5px;
            border-bottom: 1px solid #666666;
        }
        
        QListWidget::item:selected {
            background-color: #00aaff;
        }
        
        QListWidget::item:hover {
            background-color: #606060;
        }
        
        /* Text edit */
        #targetInfo {
            background-color: #1f1f1f;
            border: 2px solid #505050;
            border-radius: 2px;
            color: #e0e0e0;
            font-family: 'Consolas', 'Monaco', monospace;
            font-size: 11px;
            selection-background-color: #90c090;
            selection-color: #000000;
        }
        
        /* Camera display */
        #cameraDisplay {
            background-color: #0a0a0a;
            border: 3px solid #404040;
            border-radius: 4px;
            color: #808080;
            font-family: 'Consolas', 'Courier New', monospace;
            font-size: 14px;
            text-transform: uppercase;
            /* Allow camera to expand but maintain aspect ratio */
            min-width: 640px;
            min-height: 480px;
        }
        
        /* Status overlay */
        #statusOverlay {
            background-color: rgba(0, 0, 0, 180);
            color: #e0e0e0;
            font-family: 'Consolas', 'Courier New', monospace;
            font-size: 13px;
            font-weight: bold;
            border: 1px solid #404040;
            border-radius: 4px;
            padding: 8px;
        }
        
        /* Menu toggle button */
        #menuToggle {
            background-color: #606060;
            border: 2px solid #808080;
            border-radius: 4px;
            color: #ffffff;
            font-size: 16px;
            font-weight: bold;
        }
        
        #menuToggle:hover {
            background-color: #707070;
        }
        
        /* Mode controls frame */
        #modeControlsFrame {
            background-color: rgba(42, 42, 42, 200);
            border: 1px solid #505050;
            border-radius: 4px;
        }
        
        /* Compact status display */
        #compactStatus {
            background-color: #1f1f1f;
            border: 1px solid #404040;
            border-radius: 2px;
            color: #e0e0e0;
            font-family: 'Consolas', 'Monaco', monospace;
            font-size: 12px;
        }
        
        /* Main content */
        #mainContent {
            background-color: #1a1a1a;
        }
        
        #cameraFrame {
            background-color: #2a2a2a;
            border: 3px solid #505050;
            border-radius: 4px;
        }
        
        #cameraContainer {
            background-color: #1a1a1a;
            border: 2px solid #404040;
            border-radius: 4px;
        }
        
        /* Scroll areas */
        QScrollArea {
            background-color: transparent;
            border: none;
        }
        
        QScrollBar:vertical {
            background-color: #505050;
            width: 12px;
            border-radius: 6px;
        }
        
        QScrollBar::handle:vertical {
            background-color: #777777;
            border-radius: 6px;
            min-height: 20px;
        }
        
        QScrollBar::handle:vertical:hover {
            background-color: #888888;
        }
        """)
    
    def setup_google_earth_integration(self):
        """Setup Google Earth integration callbacks"""
        if not self.google_earth:
            return
            
        # Set callback for target changes
        self.google_earth.set_target_changed_callback(self.on_ge_target_changed)
        
        # Set telemetry source
        self.google_earth.set_telemetry_source(self.get_telemetry_for_ge)
    
    # Backend method implementations (same logic as tkinter version)
    def start_workers(self):
        """Start background workers"""
        # Start attitude request timer
        self.attitude_timer = QTimer()
        self.attitude_timer.timeout.connect(self.request_gimbal_attitude)
        self.attitude_timer.start(Config.ATTITUDE_REQUEST_MS)
        
        # Start SBS publisher worker
        self.sbs_timer = QTimer()
        self.sbs_timer.timeout.connect(self.update_sbs_targets)
        self.sbs_timer.start(int(Config.SBS_UPDATE_S * 1000))  # Convert to milliseconds
    
    def init_camera_stream(self):
        """Initialize GStreamer camera stream (much better than OpenCV)"""
        try:
            from ..gimbal.gstreamer_camera import GStreamerCameraStream
            
            # Create GStreamer camera stream
            rtsp_url = f"rtsp://{Config.SIYI_IP}:{Config.SIYI_CAMERA_PORT}/main.264"
            self.camera_stream = GStreamerCameraStream(rtsp_url=rtsp_url)
            
            # Check if dependencies are available
            if hasattr(self.camera_stream, '_dependencies_available') and not self.camera_stream._dependencies_available:
                print("[UI] GStreamer dependencies not available")
                self.camera_stream = None
                self.camera_display.setText("TACTICAL CAMERA FEED\n[GSTREAMER NOT AVAILABLE]\nInstall: sudo apt install python3-gi gstreamer1.0-plugins-*")
                self.btn_camera_connect.setText("GSTREAMER UNAVAILABLE")
                self.btn_camera_connect.setEnabled(False)
                return
            
            # Replace camera display with GStreamer video widget + Qt overlay
            if self.camera_stream._dependencies_available:
                # Create GStreamer video widget
                gst_widget = self.camera_stream.create_video_widget(self.camera_container)
                
                # Hide the old QLabel camera display
                self.camera_display.hide()
                
                # Add GStreamer widget to camera container with full scaling
                camera_container_layout = self.camera_container.layout()
                camera_container_layout.addWidget(gst_widget)
                
                # Set video widget to expand and fill all available space
                from PySide6.QtWidgets import QSizePolicy
                gst_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
                gst_widget.setMinimumSize(320, 240)  # Smaller minimum for better scaling
                
                self.gst_video_widget = gst_widget
                
                # Pass references for real-time overlay data
                self.camera_stream.set_aircraft_state(self.aircraft_state)
                self.camera_stream.set_gimbal(self.gimbal)
                self.camera_stream.set_gimbal_locker(self.gimbal_locker)
            
            # Connect status signal (no frame_ready signal needed - GStreamer handles display directly)
            self.camera_stream.connection_status_changed.connect(self.on_camera_status_changed)
            print(f"[UI] GStreamer camera stream initialized for {Config.SIYI_IP}")
            
        except Exception as e:
            print(f"[UI] Failed to initialize GStreamer camera stream: {e}")
            import traceback
            traceback.print_exc()
            self.camera_stream = None
            self.camera_display.setText("TACTICAL CAMERA FEED\n[GSTREAMER INITIALIZATION FAILED]")
            self.btn_camera_connect.setText("CAM ERROR")
            self.btn_camera_connect.setEnabled(False)
    
    def toggle_camera_stream(self):
        """Toggle camera stream on/off"""
        if not self.camera_stream:
            print("[UI] Camera stream not available")
            return
            
        if self.camera_connected:
            # Stop camera
            self.camera_stream.stop_stream()
            self.btn_camera_connect.setText("CONNECT CAM")
            self.camera_display.setText("TACTICAL CAMERA FEED\n[DISCONNECTED]")
        else:
            # Start camera
            self.btn_camera_connect.setText("CONNECTING...")
            self.btn_camera_connect.setEnabled(False)
            self.camera_display.setText("TACTICAL CAMERA FEED\n[CONNECTING...]")
            self.camera_stream.start_stream()
    
    # on_camera_frame method removed - GStreamer handles video display directly
    
    def update_camera_overlay(self):
        """Update GStreamer camera overlay with target coordinates"""
        if not self.camera_stream or not hasattr(self.camera_stream, 'set_target_coordinates'):
            return
        
        try:
            # Update aircraft state reference in camera stream for real-time overlay
            if hasattr(self.camera_stream, 'set_aircraft_state'):
                self.camera_stream.set_aircraft_state(self.aircraft_state)
            # Calculate target coordinates using the same math as before
            if (self.aircraft_state and self.aircraft_state.get('lat') is not None and 
                self.gimbal and self.gimbal.is_connected):
                
                # Debug logging for troubleshooting
                print(f"[DEBUG] Calculating target - Aircraft: {self.aircraft_state.get('lat'):.6f}, {self.aircraft_state.get('lon'):.6f}, Gimbal connected: {self.gimbal.is_connected}")
                
                from ..calc.target_calculator import TargetCalculator
                
                # Get current aircraft and gimbal data
                aircraft_lat = self.aircraft_state['lat']
                aircraft_lon = self.aircraft_state['lon']
                aircraft_alt_agl = self.aircraft_state.get('alt_agl', 0)
                
                gimbal_pitch = self.gimbal.pitch_norm or 0
                gimbal_yaw = self.gimbal.yaw_abs or 0
                
                # Calculate where camera center is pointing
                aircraft_heading = self.aircraft_state.get('heading', 0)
                
                # Log basic coordinates BEFORE 3D Euler transformations
                basic_result = TargetCalculator.calculate_target_basic(
                    aircraft_lat=aircraft_lat,
                    aircraft_lon=aircraft_lon,
                    aircraft_alt_agl=aircraft_alt_agl,
                    pitch_deg=gimbal_pitch,
                    yaw_deg=gimbal_yaw,
                    aircraft_heading_deg=aircraft_heading
                )
                
                # Calculate with full 3D transformations
                target_result = TargetCalculator.calculate_target(
                    aircraft_lat=aircraft_lat,
                    aircraft_lon=aircraft_lon,
                    aircraft_alt_agl=aircraft_alt_agl,
                    pitch_deg=gimbal_pitch,
                    yaw_deg=gimbal_yaw,
                    aircraft_yaw_deg=aircraft_heading
                )
                
                # Log comparison between basic and 3D methods
                if basic_result and target_result:
                    basic_lat, basic_lon = basic_result['lat'], basic_result['lon']
                    full_lat, full_lon = target_result['lat'], target_result['lon']
                    
                    # Calculate difference in meters
                    lat_diff_m = (full_lat - basic_lat) * 111320.0
                    lon_diff_m = (full_lon - basic_lon) * 111320.0 * math.cos(math.radians(aircraft_lat))
                    total_diff_m = math.sqrt(lat_diff_m**2 + lon_diff_m**2)
                    
                    print(f"[COORD] BEFORE 3D: {basic_lat:.6f},{basic_lon:.6f} | AFTER 3D: {full_lat:.6f},{full_lon:.6f} | 3D IMPACT: {total_diff_m:.1f}m")
                    
                    # Log coordinate calculation to session logger
                    self.session_logger.log_coordinate_calculation(
                        aircraft_lat=aircraft_lat,
                        aircraft_lon=aircraft_lon,
                        aircraft_alt=aircraft_alt_agl,
                        aircraft_heading=aircraft_heading,
                        gimbal_pitch=gimbal_pitch,
                        gimbal_yaw=gimbal_yaw,
                        coord_before_lat=basic_lat,
                        coord_before_lon=basic_lon,
                        coord_after_lat=full_lat,
                        coord_after_lon=full_lon,
                        coordinate_diff_meters=total_diff_m,
                        terrain_elevation=0.0  # TODO: Add terrain elevation if available
                    )
                
                if target_result:
                    # Update current pointing coordinates (real-time, not navigation target)
                    self.gimbal_current_pointing['lat'] = target_result['lat']
                    self.gimbal_current_pointing['lon'] = target_result['lon']
                    self.gimbal_current_pointing['distance'] = target_result['distance']
                    
                    # Update gimbal panel display if in gimbal mode
                    if (self.target_mode == "gimbal" and 
                        hasattr(self, 'lbl_current_pointing')):
                        lat = target_result['lat']
                        lon = target_result['lon']
                        self.lbl_current_pointing.setText(f"{lat:.6f}, {lon:.6f}")
                        
                        # Update gimbal target for ADSB display when in gimbal mode
                        if self.target_mode == "gimbal":
                            # Only update coordinates if target is not selected (locked)
                            if not self.gimbal_target_state['selected']:
                                self.gimbal_target_state['lat'] = lat
                                self.gimbal_target_state['lon'] = lon
                                self.gimbal_target_state['distance'] = target_result['distance']
                            
                            # Update display based on state
                            if self.gimbal_target_state['selected']:
                                # Show locked target coordinates (don't update from gimbal)
                                locked_lat = self.gimbal_target_state['lat']
                                locked_lon = self.gimbal_target_state['lon']
                                self.lbl_selected_target.setText(f"Locked: {locked_lat:.6f}, {locked_lon:.6f}")
                            elif hasattr(self, 'gimbal_tracking_active') and self.gimbal_tracking_active:
                                self.lbl_selected_target.setText(f"Loitering: {lat:.6f}, {lon:.6f}")
                            else:
                                self.lbl_selected_target.setText(f"Aiming: {lat:.6f}, {lon:.6f}")
                    
                    # Update GStreamer overlay with target coordinates
                    self.camera_stream.set_target_coordinates(
                        target_result['lat'],
                        target_result['lon'],
                        target_result['distance']
                    )
            else:
                # Debug why target calculation failed
                if not self.aircraft_state or self.aircraft_state.get('lat') is None:
                    print("[DEBUG] Target calculation skipped - no aircraft position")
                elif not self.gimbal:
                    print("[DEBUG] Target calculation skipped - no gimbal object")
                elif not self.gimbal.is_connected:
                    print("[DEBUG] Target calculation skipped - gimbal not connected")
                    
        except Exception as e:
            print(f"[UI] Error updating camera overlay: {e}")
    
    def on_camera_status_changed(self, connected: bool):
        """Handle camera connection status change"""
        self.camera_connected = connected
        
        if connected:
            self.btn_camera_connect.setText("DISCONNECT CAM")
            self.lbl_camera_status.setText("CAMERA: CONNECTED")
            self.lbl_camera_status.setStyleSheet("color: #90c090;")
        else:
            self.btn_camera_connect.setText("CONNECT CAM")
            self.lbl_camera_status.setText("CAMERA: DISCONNECTED")
            self.lbl_camera_status.setStyleSheet("color: #c09090;")
            self.camera_display.setText("TACTICAL CAMERA FEED\n[DISCONNECTED]")
            
        self.btn_camera_connect.setEnabled(True)
    
    def update_displays(self):
        """Update all display elements"""
        # Update MAVLink status
        if self.mavlink.connected:
            self.lbl_mavlink_status.setText("CONNECTED")
            self.lbl_mavlink_status.setStyleSheet("color: #00ff00;")
            
            telemetry_text = (f"LAT: {self.aircraft_state['lat']:.6f}, LON: {self.aircraft_state['lon']:.6f}\n"
                            f"ALT: {self.aircraft_state['alt_agl']:.1f}m | HDG: {self.aircraft_state['heading']:.1f}°")
            self.lbl_telemetry_details.setText(telemetry_text)
        else:
            self.lbl_mavlink_status.setText("DISCONNECTED")
            self.lbl_mavlink_status.setStyleSheet("color: #ff0000;")
            self.lbl_telemetry_details.setText("Auto-reconnecting...")
        
        # Update gimbal status
        if self.gimbal.is_connected:
            pitch, yaw = self.gimbal.get_corrected_angles(self.aircraft_state['heading'])
            age = time.time() - self.gimbal.last_update
            
            if age < 1.0:
                self.lbl_gimbal_status.setText(f"CONNECTED\nP:{pitch:.1f}° Y:{yaw:.1f}°")
                self.lbl_gimbal_status.setStyleSheet("color: #00ff00;")
            else:
                self.lbl_gimbal_status.setText(f"CONNECTED (STALE)\nP:{pitch:.1f}° Y:{yaw:.1f}° ({age:.1f}s old)")
                self.lbl_gimbal_status.setStyleSheet("color: #ffaa00;")
            
            mount_info = f"Mount: {self.gimbal.mount_dir or '—'} | Mode: {self.gimbal.motion_mode or '—'}"
            self.lbl_gimbal_details.setText(mount_info)
        else:
            self.lbl_gimbal_status.setText(f"DISCONNECTED\n{Config.SIYI_IP}:{Config.SIYI_PORT}")
            self.lbl_gimbal_status.setStyleSheet("color: #ff0000;")
            self.lbl_gimbal_details.setText("Auto-reconnecting...")
        
        # Update target display
        self.update_target_display()
        
        # Update GStreamer camera overlay with target coordinates (if camera is active)
        self.update_camera_overlay()
        
        # Update SBS publisher status
        if self.chk_sbs_publisher.isChecked():
            status = self.sbs.get_status()
            if status['clients_connected'] > 0:
                self.lbl_sbs_status.setText(f"ADS-B: ACTIVE - {status['clients_connected']} clients connected")
                self.lbl_sbs_status.setStyleSheet("color: #90c090;")
            else:
                self.lbl_sbs_status.setText(f"ADS-B: LISTENING - {status['icao_code']} ({status['callsign'].strip()})")
                self.lbl_sbs_status.setStyleSheet("color: #c0c090;")
        
        # Update aircraft state from MAVLink
        if self.mavlink.connected:
            position = self.mavlink.get_position()
            if position:
                self.aircraft_state.update(position)
            heading = self.mavlink.get_attitude()
            if heading is not None:
                self.aircraft_state['heading'] = heading
                
            # Update tracker with current target
            if self.tracker.active:
                target_lat, target_lon, target_alt = self.get_current_target()
                if target_lat is not None:
                    self.tracker.update_target(target_lat, target_lon, self.aircraft_state['alt_amsl'])
                    
                    # Update gimbal lock target and aircraft state
                    if self.gimbal_locker.active:
                        lock_target_alt = target_alt if target_alt is not None else 0.0
                        self.gimbal_locker.update_target(target_lat, target_lon, lock_target_alt)
                        self.gimbal_locker.update_aircraft_state(self.aircraft_state)
    
    # All the backend methods from tkinter version will be added here...
    # (I'll add the key methods to keep this response manageable)
    
    def toggle_gimbal(self, checked):
        """Toggle gimbal connection"""
        if checked:
            if not self.gimbal.start():
                self.chk_gimbal_connect.setChecked(False)
                # Could show error dialog here
        else:
            self.gimbal.stop()
    
    def toggle_sbs(self, checked):
        """Toggle SBS publisher"""
        if checked:
            if self.sbs.start():
                status = self.sbs.get_status()
                self.lbl_sbs_status.setText(f"ADS-B: ACTIVE - {status['icao_code']} ({status['callsign'].strip()})")
                self.lbl_sbs_status.setStyleSheet("color: #90c090;")
                print(f"[UI] SBS Publisher started: {status['host']}:{status['port']}")
            else:
                self.chk_sbs_publisher.setChecked(False)
                self.lbl_sbs_status.setText("ADS-B: FAILED TO START")
                self.lbl_sbs_status.setStyleSheet("color: #c09090;")
        else:
            self.sbs.stop()
            self.lbl_sbs_status.setText("ADS-B: STOPPED")
            self.lbl_sbs_status.setStyleSheet("color: #b0b0b0;")
            print("[UI] SBS Publisher stopped")
    
    def on_target_mode_changed(self, mode_data):
        """Handle target mode change"""
        self.target_mode = mode_data
        
        # Show/hide mode-specific controls
        if mode_data == "fixed":
            self.mode_controls_frame.show()
            self.fixed_panel.show()
            if hasattr(self, 'waypoint_panel'):
                self.waypoint_panel.hide()
            if hasattr(self, 'gimbal_panel'):
                self.gimbal_panel.hide()
        elif mode_data == "waypoint":
            self.mode_controls_frame.show()
            if hasattr(self, 'waypoint_panel'):
                self.waypoint_panel.show()
            if hasattr(self, 'fixed_panel'):
                self.fixed_panel.hide()
            if hasattr(self, 'gimbal_panel'):
                self.gimbal_panel.hide()
        else:  # gimbal mode
            self.mode_controls_frame.show()
            if hasattr(self, 'gimbal_panel'):
                self.gimbal_panel.show()
            if hasattr(self, 'fixed_panel'):
                self.fixed_panel.hide()
            if hasattr(self, 'waypoint_panel'):
                self.waypoint_panel.hide()
    
    def get_current_target(self):
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
                    # Try to get coordinates from selected dropdown item
                    current_index = self.waypoints_combo.currentIndex()
                    if (hasattr(self, 'waypoint_data') and self.waypoint_data and 
                        0 <= current_index < len(self.waypoint_data)):
                        wp = self.waypoint_data[current_index]
                        print(f"[TARGET] Using dropdown waypoint {current_index + 1}: {wp.name}")
                        return (wp.latitude, wp.longitude, wp.altitude)
                    else:
                        print(f"[TARGET] No waypoint available for tracking. Dropdown index: {current_index}")
                        if hasattr(self, 'waypoint_data'):
                            print(f"[TARGET] Waypoint data length: {len(getattr(self, 'waypoint_data', []))}")
            else:
                print("[TARGET] Google Earth controller not available")
        else:
            # Gimbal mode - always return current gimbal target for ADSB display
            if self.gimbal_target_state['lat'] is not None:
                return (self.gimbal_target_state['lat'],
                        self.gimbal_target_state['lon'], 
                        0.0)  # Ground level for gimbal targets
        return None, None, None
    
    def check_connections(self):
        """Periodically check and attempt to repair connections"""
        # Check gimbal connection
        if not self.gimbal.is_connected:
            print("[CONNECTION] Gimbal disconnected, attempting restart...")
            try:
                self.gimbal.stop()
                time.sleep(0.5)
                self.gimbal.start()
            except Exception as e:
                print(f"[CONNECTION] Failed to restart gimbal: {e}")
        
        # MAVLink connection health is handled internally by MAVLinkHandler
    
    def update_target_display(self):
        """Update compact status display in sidebar"""
        info = f"=== MODE: {self.target_mode.upper()} ===\n\n"
        
        if self.target_mode == "waypoint":
            # Waypoint mission mode
            if not self.google_earth:
                info += "Google Earth integration not available\n"
            else:
                status = self.google_earth.get_mission_status()
                
                if status['total_waypoints'] > 0:
                    info += f"Mission: {status['total_waypoints']} waypoints loaded\n"
                    info += f"Progress: {status['visited_waypoints']}/{status['total_waypoints']} ({status['progress_percentage']:.1f}%)\n\n"
                    
                    if status['current_waypoint']:
                        wp = status['current_waypoint']
                        info += f"Current Target: {wp.name}\n"
                        info += f"Coordinates: {wp.latitude:.7f}, {wp.longitude:.7f}\n"
                        info += f"Altitude: {wp.altitude:.1f}m\n\n"
                        
                        # Calculate distance to current waypoint
                        if self.aircraft_state['lat'] and self.aircraft_state['lon']:
                            distance = self.calculate_distance(
                                self.aircraft_state['lat'], self.aircraft_state['lon'],
                                wp.latitude, wp.longitude
                            )
                            info += f"Distance to Target: {distance:.0f}m\n"
                    
                    if self.tracker.active:
                        stats = self.tracker.get_stats()
                        info += f"\nTRACKING WAYPOINT: {stats['duration']:.0f}s | {stats['updates']} updates\n"
                        info += f"Next update: {stats['next_update']:.1f}s\n"
                else:
                    info += "No waypoint mission loaded\n"
        
        elif self.target_mode == "fixed":
            if self.fixed_target_state['lat'] is not None:
                info += f"Fixed Target: {self.fixed_target_state['lat']:.7f}, {self.fixed_target_state['lon']:.7f}\n"
                info += f"Fixed Altitude: {self.fixed_target_state['alt']:.1f}m\n\n"
                
                # Calculate distance to fixed target
                if self.aircraft_state['lat'] and self.aircraft_state['lon']:
                    distance = self.calculate_distance(
                        self.aircraft_state['lat'], self.aircraft_state['lon'],
                        self.fixed_target_state['lat'], self.fixed_target_state['lon']
                    )
                    info += f"Distance to Fixed Target: {distance:.0f}m\n"
                
                if self.tracker.active:
                    stats = self.tracker.get_stats()
                    info += f"\nTRACKING FIXED TARGET: {stats['duration']:.0f}s | {stats['updates']} updates\n"
                    info += f"Next update: {stats['next_update']:.1f}s\n"
            else:
                info += "No fixed target set - enter coordinates and click 'Set Fixed Target'\n"
                
        else:
            # Gimbal mode
            gimbal_result = self.calculate_gimbal_target()
            if gimbal_result:
                info += f"Gimbal Target: {gimbal_result['lat']:.7f}, {gimbal_result['lon']:.7f}\n"
                info += f"Distance: {gimbal_result['distance']:.0f}m\n"
                info += f"Bearing: {gimbal_result['bearing']:.1f}°\n"
                info += f"Elevation: {gimbal_result['elevation']:.1f}°\n\n"
                info += "Status: Gimbal target calculated from current pointing direction\n"
        
        # Add aircraft state
        info += f"\n=== AIRCRAFT STATE ===\n"
        info += f"Position: {self.aircraft_state['lat']:.6f}, {self.aircraft_state['lon']:.6f}\n"
        info += f"Altitude: {self.aircraft_state['alt_agl']:.1f}m AGL | {self.aircraft_state['alt_amsl']:.1f}m AMSL\n"
        info += f"Heading: {self.aircraft_state['heading']:.1f}°\n"
        
        if hasattr(self, 'compact_status'):
            self.compact_status.setPlainText(info)
    
    def update_waypoints_list(self, waypoints):
        """Update the waypoints dropdown widget"""
        self.waypoints_combo.clear()
        
        if not waypoints:
            self.waypoints_combo.addItem("No waypoints loaded")
            return
            
        for i, waypoint in enumerate(waypoints):
            item_text = f"{i+1}. {waypoint.name} ({waypoint.latitude:.6f}, {waypoint.longitude:.6f})"
            self.waypoints_combo.addItem(item_text)
        
        # Store waypoint data
        self.waypoint_data = waypoints
    
    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """Calculate distance between two coordinates"""
        import math
        R = 6371000  # Earth radius in meters
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        return R * 2 * math.asin(math.sqrt(a))
    
    def calculate_gimbal_target(self):
        """Calculate gimbal target (simplified version)"""
        # This would contain the full gimbal calculation logic
        # For now, returning None to indicate no target calculated
        return None
    
    def setup_gimbal_button_events(self):
        """Setup gimbal control button events with press-and-hold behavior"""
        # Movement state tracking
        self.gimbal_movement = {"active": False, "timer": None}
        
        # Mouse drag gimbal control state
        self.mouse_drag = {
            "active": False,
            "last_pos": None,
            "sensitivity": 1.5  # Increased sensitivity for more responsive control
        }
        
        # Connect button press/release events for continuous movement
        self.btn_left.pressed.connect(lambda: self.start_gimbal_movement(-self.speed_slider.value(), 0))
        self.btn_left.released.connect(self.stop_gimbal_movement)
        
        self.btn_right.pressed.connect(lambda: self.start_gimbal_movement(self.speed_slider.value(), 0))
        self.btn_right.released.connect(self.stop_gimbal_movement)
        
        self.btn_up.pressed.connect(lambda: self.start_gimbal_movement(0, self.speed_slider.value()))
        self.btn_up.released.connect(self.stop_gimbal_movement)
        
        self.btn_down.pressed.connect(lambda: self.start_gimbal_movement(0, -self.speed_slider.value()))
        self.btn_down.released.connect(self.stop_gimbal_movement)
        
        self.btn_center.clicked.connect(self.center_gimbal)
        
        # Zoom button events with press-and-hold behavior
        self.btn_zoom_in.pressed.connect(self.start_zoom_in)
        self.btn_zoom_in.released.connect(self.stop_zoom)
        
        self.btn_zoom_out.pressed.connect(self.start_zoom_out)
        self.btn_zoom_out.released.connect(self.stop_zoom)
    
    def start_gimbal_movement(self, yaw_speed, pitch_speed):
        """Start continuous gimbal movement"""
        if not self.gimbal.is_connected:
            return
            
        # Stop centering if manual movement starts
        if self.centering_active:
            self.centering_active = False
            print("[UI] Manual gimbal movement detected - stopping centering")
            
        self.gimbal_movement["active"] = True
        self.gimbal_movement["yaw_speed"] = yaw_speed
        self.gimbal_movement["pitch_speed"] = pitch_speed
        
        # Start movement timer
        if self.gimbal_movement["timer"]:
            self.gimbal_movement["timer"].stop()
        
        self.gimbal_movement["timer"] = QTimer()
        self.gimbal_movement["timer"].timeout.connect(self.continuous_gimbal_move)
        self.gimbal_movement["timer"].start(50)  # 50ms intervals, same as Tkinter version
        
        # Start immediately
        self.continuous_gimbal_move()
    
    def continuous_gimbal_move(self):
        """Continuous movement function called by timer"""
        if self.gimbal_movement["active"] and self.gimbal.is_connected:
            self.gimbal.jog(
                self.gimbal_movement["yaw_speed"], 
                self.gimbal_movement["pitch_speed"]
            )
    
    def stop_gimbal_movement(self):
        """Stop gimbal movement"""
        self.gimbal_movement["active"] = False
        
        if self.gimbal_movement["timer"]:
            self.gimbal_movement["timer"].stop()
            self.gimbal_movement["timer"] = None
        
        if self.gimbal.is_connected:
            self.gimbal.jog(0, 0)  # Stop movement
    
    def start_zoom_in(self):
        """Start continuous zoom in"""
        if self.gimbal.is_connected:
            self.gimbal.zoom_in()
    
    def start_zoom_out(self):
        """Start continuous zoom out"""
        if self.gimbal.is_connected:
            self.gimbal.zoom_out()
    
    def stop_zoom(self):
        """Stop zoom operation"""
        if self.gimbal.is_connected:
            self.gimbal.zoom_hold()
    
    def move_gimbal(self, yaw_speed, pitch_speed):
        """Move gimbal with given speeds (legacy method)"""
        if self.gimbal.is_connected:
            # Stop centering if manual movement detected
            if self.centering_active and (yaw_speed != 0 or pitch_speed != 0):
                self.centering_active = False
                print("[UI] Manual gimbal jog detected - stopping centering")
            self.gimbal.jog(yaw_speed, pitch_speed)
            
            # Log manual gimbal control
            if yaw_speed != 0 or pitch_speed != 0:  # Only log actual movement commands
                try:
                    current_pitch = self.gimbal.pitch_norm or 0
                    current_yaw = self.gimbal.yaw_abs or 0
                    # For manual control, we don't have specific target angles, so log current + speed
                    self.session_logger.log_gimbal_performance(
                        commanded_pitch=current_pitch,  # Current position as "commanded"
                        commanded_yaw=current_yaw,
                        actual_pitch=current_pitch,
                        actual_yaw=current_yaw,
                        gimbal_connected=self.gimbal.is_connected,
                        operation_mode="manual"
                    )
                except Exception as e:
                    print(f"[UI] Gimbal logging error: {e}")
    
    def center_gimbal(self):
        """Center the gimbal with aggressive centering"""
        if self.gimbal.is_connected:
            print("[UI] Starting aggressive gimbal centering...")
            self.centering_active = True  # Set centering flag
            
            # Try multiple centering approaches for better reliability
            
            # Method 1: Use built-in center command
            self.gimbal.center_gimbal()
            
            # Method 2: Also try direct center command
            self.gimbal.center()
            
            # Method 3: Manual centering with movement commands
            # Send gimbal to approximate center position with movement
            QTimer.singleShot(100, lambda: self._force_center_position())
    
    def _force_center_position(self):
        """Force gimbal to center using movement commands"""
        if not self.gimbal.is_connected or not self.centering_active:
            self.centering_active = False  # Clear flag if interrupted
            return
            
        print("[UI] Force centering with movement commands...")
        
        # Get current gimbal position
        current_yaw = self.gimbal.yaw_abs or 0
        current_pitch = self.gimbal.pitch_norm or 0
        
        # Calculate required movement to reach center (0° yaw, 0° pitch)
        # Use shortest path for yaw (handle 360° wraparound)
        yaw_error = -current_yaw
        # Fix wraparound: choose shortest path to 0°
        if yaw_error > 180:
            yaw_error -= 360
        elif yaw_error < -180:
            yaw_error += 360
            
        pitch_error = -current_pitch  # How much we need to move in pitch
        
        print(f"[UI] Current: Yaw={current_yaw:.1f}°, Pitch={current_pitch:.1f}°")
        print(f"[UI] Centering errors: Yaw={yaw_error:.1f}°, Pitch={pitch_error:.1f}°")
        
        # Send movement commands to center (use higher speeds for faster centering)
        center_speed = 50
        yaw_speed = center_speed if yaw_error > 5 else (-center_speed if yaw_error < -5 else 0)
        pitch_speed = center_speed if pitch_error > 5 else (-center_speed if pitch_error < -5 else 0)
        
        if abs(yaw_error) > 2 or abs(pitch_error) > 2:  # If not close to center
            self.gimbal.jog(yaw_speed, pitch_speed)
            # Continue centering in 200ms
            QTimer.singleShot(200, self._force_center_position)
        else:
            # Close enough to center, stop movement
            self.gimbal.jog(0, 0)
            self.centering_active = False  # Clear centering flag
            print("[UI] Gimbal centering completed")
    
    def request_gimbal_attitude(self):
        """Request gimbal attitude data"""
        if self.chk_gimbal_connect.isChecked() and not self._closing:
            self.gimbal.request_attitude()
    
    def update_sbs_targets(self):
        """Update SBS publisher with current target information"""
        if not self.chk_sbs_publisher.isChecked() or self._closing:
            return
        
        try:
            # Get current target coordinates
            target_lat, target_lon, target_alt = self.get_current_target()
            
            if target_lat is not None and target_lon is not None:
                # Use target altitude if available, otherwise use a reasonable default
                target_altitude = target_alt if target_alt is not None else 100.0  # 100m default
                
                # Calculate ground speed and track from aircraft heading
                ground_speed = 0.0  # Target is stationary
                track = self.aircraft_state.get('heading', 0.0)  # Use aircraft heading as reference
                
                # Publish target to SBS
                success = self.sbs.publish(
                    lat=target_lat,
                    lon=target_lon, 
                    alt_amsl=target_altitude,
                    ground_speed=ground_speed,
                    track=track
                )
                
                if not success:
                    print(f"[UI] Failed to publish SBS target: {target_lat:.6f}, {target_lon:.6f}")
            
        except Exception as e:
            print(f"[UI] SBS update error: {e}")
    
    # Placeholder methods for the remaining functionality
    def set_fixed_target(self):
        """Set fixed coordinate target"""
        try:
            lat = float(self.fixed_lat_input.text())
            lon = float(self.fixed_lon_input.text())
            alt = self.fixed_alt_input.value()
            
            self.fixed_target_state['lat'] = lat
            self.fixed_target_state['lon'] = lon
            self.fixed_target_state['alt'] = alt
            
            if self.gimbal and self.gimbal.logger:
                self.gimbal.logger.log_target_set(lat, lon, alt, "fixed_coordinates_ui")
            
            # Log fixed coordinate target selection
            try:
                if self.aircraft_state:
                    aircraft_lat = self.aircraft_state['lat']
                    aircraft_lon = self.aircraft_state['lon']
                    aircraft_alt_agl = self.aircraft_state.get('alt_agl', 0)
                    aircraft_heading = self.aircraft_state.get('heading', 0)
                    
                    # Calculate distance to target
                    import math
                    distance_2d = calculate_distance(aircraft_lat, aircraft_lon, lat, lon)
                    
                    self.session_logger.log_target_selection(
                        target_lat=lat,
                        target_lon=lon,
                        target_alt=alt,
                        aircraft_lat=aircraft_lat,
                        aircraft_lon=aircraft_lon,
                        aircraft_alt_agl=aircraft_alt_agl,
                        aircraft_heading=aircraft_heading,
                        gimbal_pitch=0.0,  # Fixed coordinates don't use gimbal
                        gimbal_yaw=0.0,
                        coord_before_euler_lat=lat,  # Fixed coordinates are direct input
                        coord_before_euler_lon=lon,
                        coord_after_euler_lat=lat,
                        coord_after_euler_lon=lon,
                        transformation_impact_meters=0.0,  # No transformation for manual input
                        target_distance_2d=distance_2d,
                        selection_mode="fixed_coordinate"
                    )
                else:
                    # No aircraft state available
                    self.session_logger.log_target_selection(
                        target_lat=lat,
                        target_lon=lon,
                        target_alt=alt,
                        aircraft_lat=0.0,
                        aircraft_lon=0.0,
                        aircraft_alt_agl=0.0,
                        aircraft_heading=0.0,
                        gimbal_pitch=0.0,
                        gimbal_yaw=0.0,
                        coord_before_euler_lat=lat,
                        coord_before_euler_lon=lon,
                        coord_after_euler_lat=lat,
                        coord_after_euler_lon=lon,
                        transformation_impact_meters=0.0,
                        target_distance_2d=0.0,
                        selection_mode="fixed_coordinate"
                    )
            except Exception as e:
                print(f"[FIXED TARGET] Logging error: {e}")
                
            print(f"Fixed target set to: {lat:.7f}, {lon:.7f}, {alt}m")
        except ValueError:
            print("Error: Invalid coordinates")
    
    def use_aircraft_position(self):
        """Use current aircraft position as fixed target"""
        if self.aircraft_state['lat'] and self.aircraft_state['lon']:
            self.fixed_lat_input.setText(str(self.aircraft_state['lat']))
            self.fixed_lon_input.setText(str(self.aircraft_state['lon']))
            self.fixed_alt_input.setValue(int(self.aircraft_state['alt_amsl']))
            self.set_fixed_target()
        else:
            print("Error: No aircraft position available")
    
    def load_kml_mission(self):
        """Load KML/KMZ mission file with mission storage"""
        from PySide6.QtWidgets import QFileDialog
        
        if not self.google_earth:
            print("Google Earth integration not available")
            return
        
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Load KML/KMZ Mission", "", "KML Files (*.kml *.kmz)"
        )
        
        if file_path:
            try:
                # Load mission using Google Earth controller
                result = self.google_earth.load_mission_from_kml(file_path)
                
                if result['success']:
                    waypoints = result['waypoints']
                    # Store the mission
                    mission_name = os.path.basename(file_path).split('.')[0]
                    self.stored_missions[mission_name] = {
                        'file_path': file_path,
                        'waypoints': waypoints,
                        'name': mission_name
                    }
                    
                    # Update missions dropdown
                    self.update_missions_combo()
                    
                    # Save missions to file for persistence
                    self.save_missions_to_file()
                    
                    # Update waypoints list with proper format
                    waypoint_objects = []
                    for wp_data in waypoints:
                        # Create mock waypoint objects for display
                        class MockWaypoint:
                            def __init__(self, data):
                                self.name = data['name']
                                self.latitude = data['latitude']
                                self.longitude = data['longitude']
                                self.altitude = data['altitude']
                        waypoint_objects.append(MockWaypoint(wp_data))
                    
                    self.update_waypoints_list(waypoint_objects)
                    
                    # Update status
                    self.lbl_mission_status.setText(f"LOADED: {mission_name} ({len(waypoints)} waypoints)")
                    
                    print(f"Mission '{mission_name}' loaded with {len(waypoints)} waypoints")
                else:
                    print(f"Failed to load mission: {result['error']}")
                    
            except Exception as e:
                print(f"Error loading KML: {e}")
    
    def load_saved_missions(self):
        """Load missions from persistent storage"""
        try:
            if os.path.exists(self.missions_file):
                with open(self.missions_file, 'r') as f:
                    saved_data = json.load(f)
                    self.stored_missions = saved_data
                    
                # Update UI
                self.update_missions_combo()
                print(f"Loaded {len(self.stored_missions)} saved missions")
            else:
                print("No saved missions file found")
        except Exception as e:
            print(f"Error loading saved missions: {e}")
            self.stored_missions = {}
    
    def save_missions_to_file(self):
        """Save missions to persistent storage"""
        try:
            with open(self.missions_file, 'w') as f:
                json.dump(self.stored_missions, f, indent=2)
            print(f"Saved {len(self.stored_missions)} missions to {self.missions_file}")
        except Exception as e:
            print(f"Error saving missions: {e}")
    
    def update_missions_combo(self):
        """Update missions dropdown with current missions"""
        self.missions_combo.clear()
        if self.stored_missions:
            for mission_name in self.stored_missions.keys():
                self.missions_combo.addItem(mission_name)
        else:
            self.missions_combo.addItem("No missions loaded")
    
    def delete_mission(self):
        """Delete selected mission with confirmation"""
        from PySide6.QtWidgets import QMessageBox
        
        selected_mission = self.missions_combo.currentText()
        if selected_mission == "No missions loaded" or not selected_mission or selected_mission not in self.stored_missions:
            return
        
        # Confirmation dialog
        reply = QMessageBox.question(
            self, 
            "Delete Mission", 
            f"Are you sure you want to delete mission '{selected_mission}'?\n\nThis action cannot be undone.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            # Delete from stored missions
            del self.stored_missions[selected_mission]
            
            # Update UI
            self.update_missions_combo()
            
            # Save to file
            self.save_missions_to_file()
            
            print(f"Mission '{selected_mission}' deleted")
    
    def start_waypoint_mission(self):
        """Start waypoint mission"""
        if not self.google_earth:
            print("Google Earth integration not available")
            return
            
        selected_mission = self.missions_combo.currentText()
        if selected_mission == "No missions loaded" or not selected_mission or selected_mission not in self.stored_missions:
            print(f"No valid mission selected. Current: '{selected_mission}', Available: {list(self.stored_missions.keys())}")
            return
            
        try:
            mission_data = self.stored_missions[selected_mission]
            mode_str = self.waypoint_mode_combo.currentData()
            
            # Map mode string to TrackingMode enum
            mode_map = {
                'sequential': TrackingMode.SEQUENTIAL,
                'manual_select': TrackingMode.MANUAL_SELECT,
                'loiter_all': TrackingMode.LOITER_ALL
            }
            tracking_mode = mode_map.get(mode_str, TrackingMode.SEQUENTIAL)
            
            # Start the mission (need to re-load the KML first)
            result = self.google_earth.load_mission_from_kml(mission_data['file_path'])
            if not result['success']:
                print(f"Failed to reload mission: {result['error']}")
                return
                
            # Update waypoints dropdown after successful reload
            waypoints = result['waypoints']
            waypoint_objects = []
            for wp_data in waypoints:
                # Create mock waypoint objects for display
                class MockWaypoint:
                    def __init__(self, data):
                        self.name = data['name']
                        self.latitude = data['latitude']
                        self.longitude = data['longitude']
                        self.altitude = data['altitude']
                waypoint_objects.append(MockWaypoint(wp_data))
            
            self.update_waypoints_list(waypoint_objects)
                
            # Start the waypoint mission
            success = self.google_earth.start_mission(tracking_mode)
            
            if success:
                self.lbl_mission_status.setText(f"MISSION STARTED: {selected_mission}")
                print(f"Started waypoint mission: {selected_mission}")
                print(f"Updated waypoints list with {len(waypoint_objects)} waypoints")
            else:
                print("Failed to start waypoint mission")
                
        except KeyError:
            print(f"Mission '{selected_mission}' not found in storage")
        except Exception as e:
            print(f"Error starting mission: {e}")
    
    def next_waypoint(self):
        """Move to next waypoint"""
        if not self.google_earth:
            print("Google Earth integration not available")
            return
            
        next_wp = self.google_earth.next_waypoint()
        if next_wp:
            print(f"Moved to next waypoint: {next_wp['name']}")
            self.update_waypoint_selection()
        else:
            print("Unable to move to next waypoint (end of mission or not started)")
    
    def prev_waypoint(self):
        """Move to previous waypoint"""
        if not self.google_earth:
            print("Google Earth integration not available")
            return
            
        prev_wp = self.google_earth.previous_waypoint()
        if prev_wp:
            print(f"Moved to previous waypoint: {prev_wp['name']}")
            self.update_waypoint_selection()
        else:
            print("Unable to move to previous waypoint (beginning of mission or not started)")
    
    def stop_waypoint_mission(self):
        """Stop waypoint mission"""
        if not self.google_earth:
            print("Google Earth integration not available")
            return
            
        self.google_earth.stop_mission()
        self.lbl_mission_status.setText("MISSION STOPPED")
        print("Waypoint mission stopped")
    
    def on_waypoint_combo_changed(self, index):
        """Handle waypoint selection from dropdown"""
        if not self.google_earth or index < 0:
            return
            
        if self.waypoints_combo.currentText() == "No waypoints loaded":
            return
            
        # First ensure there's a loaded mission
        if not hasattr(self, 'waypoint_data') or not self.waypoint_data:
            print("[WAYPOINT] No waypoint mission loaded. Please start a mission first.")
            return
            
        # For waypoint mode, make sure mission is active for tracking BEFORE selecting waypoint
        if self.target_mode == "waypoint" and not self.google_earth.waypoint_manager.is_mission_active():
            print("[WAYPOINT] Auto-starting waypoint mission for tracking...")
            # Use sequential mode as default for dropdown selection
            from gimbal_app.google_earth.waypoint_manager import TrackingMode
            self.google_earth.waypoint_manager.start_mission(TrackingMode.MANUAL_SELECT)
            self.lbl_mission_status.setText(f"MISSION: AUTO-STARTED")
        
        # Now select the waypoint
        selected_wp = self.google_earth.select_waypoint(index)
        
        if selected_wp:
            print(f"[WAYPOINT] Selected waypoint {index + 1}: {selected_wp['name']}")
            self.update_waypoint_selection()
        else:
            print(f"[WAYPOINT] Failed to select waypoint at index {index}")
    
    def update_waypoint_selection(self):
        """Update UI to reflect current waypoint selection"""
        if not self.google_earth:
            return
            
        try:
            current_wp = self.google_earth.waypoint_manager.get_current_waypoint() if self.google_earth.waypoint_manager else None
            total_waypoints = len(self.google_earth.waypoint_manager.get_waypoints()) if self.google_earth.waypoint_manager else 0
            current_index = self.google_earth.waypoint_manager.get_current_index() if self.google_earth.waypoint_manager else -1
            
            if current_wp and current_index >= 0:
                self.lbl_current_waypoint.setText(f"WP: {current_index + 1}/{total_waypoints}")
                
                # Update dropdown selection without triggering signal
                self.waypoints_combo.blockSignals(True)
                self.waypoints_combo.setCurrentIndex(current_index)
                self.waypoints_combo.blockSignals(False)
                
                
            else:
                self.lbl_current_waypoint.setText(f"WP: 0/{total_waypoints}")
                
        except Exception as e:
            print(f"Error updating waypoint selection: {e}")
            self.lbl_current_waypoint.setText("WP: -/-")
    
    def on_waypoint_radius_changed(self, radius):
        """Handle radius change for current waypoint"""
        print(f"Updated waypoint radius to {radius}m")
        
        # If tracking is active and in waypoint mode, update tracker radius in real-time
        if (self.tracker.active and self.target_mode == "waypoint"):
            self.tracker.update_radius(radius)
            print(f"[TRACKING] Updated active tracker radius to {radius}m")
    
    def start_tracking(self):
        """Start dynamic tracking"""
        # Get parameters from UI
        # For waypoint mode, use waypoint-specific radius if available
        if self.target_mode == "waypoint":
            radius = self.waypoint_radius_spin.value()
            print(f"[TRACKING] Using waypoint-specific radius: {radius}m")
        else:
            radius = self.radius_spin.value()
            
        update_interval = self.update_spin.value()
        min_movement = self.movement_spin.value()
        enable_gimbal_lock = self.chk_gimbal_lock.isChecked()
        
        # Update configuration
        self.ui_settings['loiter_radius'] = radius
        self.ui_settings['update_interval'] = update_interval
        self.ui_settings['min_movement'] = min_movement
        
        # Get current target
        target_lat, target_lon, target_alt = self.get_current_target()
        
        print(f"[TRACKING] Current mode: {self.target_mode}")
        print(f"[TRACKING] Target coordinates: {target_lat}, {target_lon}, {target_alt}")
        
        if target_lat is None or target_lon is None:
            print(f"[TRACKING] Error: No target set for tracking in mode '{self.target_mode}'")
            if self.target_mode == "fixed":
                print(f"[TRACKING] Fixed target state: {self.fixed_target_state}")
            elif self.target_mode == "gimbal":
                print(f"[TRACKING] Gimbal target state: {self.gimbal_target_state}")
            elif self.target_mode == "waypoint":
                if self.google_earth:
                    coords = self.google_earth.get_current_target_coordinates()
                    print(f"[TRACKING] Google Earth coords: {coords}")
                else:
                    print(f"[TRACKING] Google Earth controller not available")
            return
        
        # Start tracking
        target_alt = target_alt if target_alt is not None else 0.0
        success = self.tracker.start_tracking(
            target_lat, target_lon, target_alt,
            radius, update_interval, min_movement
        )
        
        if success:
            # Start gimbal lock if enabled
            if enable_gimbal_lock and self.gimbal.is_connected:
                lock_target_alt = target_alt if target_alt is not None else 0.0
                self.gimbal_locker.start_locking(
                    target_lat, target_lon, lock_target_alt
                )
            
            # Update UI state
            self.btn_start_tracking.setEnabled(False)
            self.btn_stop_tracking.setEnabled(True)
            
            print(f"Started dynamic tracking: {target_lat:.6f}, {target_lon:.6f}")
        else:
            print("Failed to start tracking")
    
    def stop_tracking(self):
        """Stop dynamic tracking"""
        # Stop tracking and gimbal lock
        self.tracker.stop_tracking()
        self.gimbal_locker.stop_locking()
        
        # Update UI state
        self.btn_start_tracking.setEnabled(True)
        self.btn_stop_tracking.setEnabled(False)
        
        print("Stopped dynamic tracking")
    
    def single_goto(self):
        """Single goto command"""
        # Get current target
        target_lat, target_lon, target_alt = self.get_current_target()
        
        if target_lat is None or target_lon is None:
            print("Error: No target set for single goto")
            return
        
        if not self.mavlink.connected:
            print("Error: No MAVLink connection")
            return
        
        # Get radius - use waypoint specific radius if available
        if self.target_mode == "waypoint":
            radius = self.get_current_waypoint_radius()
        else:
            radius = self.radius_spin.value()
            
        # Use current aircraft altitude
        alt = self.aircraft_state.get('alt_amsl', 100.0)
        
        # Send loiter command to current target
        success = self.mavlink.set_loiter_mode(target_lat, target_lon, alt, radius)
        
        if success:
            print(f"Single goto sent: {target_lat:.6f}, {target_lon:.6f} @ {alt}m (R:{radius}m)")
        else:
            print("Failed to send single goto command")
    
    def select_gimbal_target(self):
        """Lock gimbal on current pointing position"""
        if self.gimbal_current_pointing['lat'] is None:
            print("Error: No gimbal pointing coordinates available")
            return
        
        # Capture current gimbal pointing position and lock it
        target_lat = self.gimbal_current_pointing['lat']
        target_lon = self.gimbal_current_pointing['lon']
        target_alt = 0.0  # Ground level for gimbal targets
        
        # Log target selection with coordinate comparison
        print(f"\n{'='*60}")
        print(f"[TARGET SELECT] Selected coordinates: {target_lat:.6f},{target_lon:.6f}")
        
        # Always show coordinate comparison when setting target
        if self.aircraft_state:
            aircraft_lat = self.aircraft_state['lat']
            aircraft_lon = self.aircraft_state['lon']
            aircraft_alt_agl = self.aircraft_state.get('alt_agl', 100)  # Default altitude if not available
            aircraft_heading = self.aircraft_state.get('heading', 0)
            
            # Get gimbal angles (use current or defaults)
            if self.gimbal.is_connected:
                gimbal_pitch = self.gimbal.pitch_norm or 0
                gimbal_yaw = self.gimbal.yaw_abs or 0
                print(f"[TARGET SELECT] Current gimbal: Pitch={gimbal_pitch:.1f}° Yaw={gimbal_yaw:.1f}°")
            else:
                # Use some default angles for demonstration
                gimbal_pitch = 30.0  # Looking down
                gimbal_yaw = 0.0     # Straight ahead
                print(f"[TARGET SELECT] Using default gimbal angles (gimbal not connected)")
            
            # Calculate what basic method would give
            basic_coords = TargetCalculator.calculate_target_basic(
                aircraft_lat, aircraft_lon, aircraft_alt_agl,
                gimbal_pitch, gimbal_yaw, aircraft_heading
            )
            
            if basic_coords:
                print(f"[TARGET SELECT] BEFORE 3D: {basic_coords['lat']:.6f},{basic_coords['lon']:.6f}")
                print(f"[TARGET SELECT] AFTER 3D:  {target_lat:.6f},{target_lon:.6f}")
                
                # Calculate difference in meters
                lat_diff_m = (target_lat - basic_coords['lat']) * 111320.0
                lon_diff_m = (target_lon - basic_coords['lon']) * 111320.0 * math.cos(math.radians(aircraft_lat))
                total_diff_m = math.sqrt(lat_diff_m**2 + lon_diff_m**2)
                
                print(f"[TARGET SELECT] 3D CORRECTION IMPACT: {total_diff_m:.1f} meters")
                print(f"[TARGET SELECT] Lat shift: {lat_diff_m:.1f}m, Lon shift: {lon_diff_m:.1f}m")
                
                # Log comprehensive target selection data to session logger
                try:
                    target_distance = self.gimbal_current_pointing.get('distance', 0.0)
                    self.session_logger.log_target_selection(
                        target_lat=target_lat,
                        target_lon=target_lon,
                        target_alt=target_alt,
                        aircraft_lat=aircraft_lat,
                        aircraft_lon=aircraft_lon,
                        aircraft_alt_agl=aircraft_alt_agl,
                        aircraft_heading=aircraft_heading,
                        gimbal_pitch=gimbal_pitch,
                        gimbal_yaw=gimbal_yaw,
                        coord_before_euler_lat=basic_coords['lat'],
                        coord_before_euler_lon=basic_coords['lon'],
                        coord_after_euler_lat=target_lat,
                        coord_after_euler_lon=target_lon,
                        transformation_impact_meters=total_diff_m,
                        target_distance_2d=target_distance,
                        selection_mode="gimbal_pointing"
                    )
                except Exception as e:
                    print(f"[TARGET SELECT] Logging error: {e}")
            else:
                print(f"[TARGET SELECT] Could not calculate basic coordinates for comparison")
                
                # Still log the target selection even without coordinate comparison
                try:
                    target_distance = self.gimbal_current_pointing.get('distance', 0.0)
                    self.session_logger.log_target_selection(
                        target_lat=target_lat,
                        target_lon=target_lon,
                        target_alt=target_alt,
                        aircraft_lat=aircraft_lat,
                        aircraft_lon=aircraft_lon,
                        aircraft_alt_agl=aircraft_alt_agl,
                        aircraft_heading=aircraft_heading,
                        gimbal_pitch=gimbal_pitch,
                        gimbal_yaw=gimbal_yaw,
                        coord_before_euler_lat=0.0,  # No comparison available
                        coord_before_euler_lon=0.0,
                        coord_after_euler_lat=target_lat,
                        coord_after_euler_lon=target_lon,
                        transformation_impact_meters=0.0,
                        target_distance_2d=target_distance,
                        selection_mode="gimbal_pointing"
                    )
                except Exception as e:
                    print(f"[TARGET SELECT] Logging error: {e}")
        else:
            print(f"[TARGET SELECT] No aircraft state available for coordinate comparison")
            
            # Still log basic target selection
            try:
                target_distance = self.gimbal_current_pointing.get('distance', 0.0)
                self.session_logger.log_target_selection(
                    target_lat=target_lat,
                    target_lon=target_lon,
                    target_alt=target_alt,
                    aircraft_lat=0.0,  # No aircraft state
                    aircraft_lon=0.0,
                    aircraft_alt_agl=0.0,
                    aircraft_heading=0.0,
                    gimbal_pitch=0.0,
                    gimbal_yaw=0.0,
                    coord_before_euler_lat=0.0,
                    coord_before_euler_lon=0.0,
                    coord_after_euler_lat=target_lat,
                    coord_after_euler_lon=target_lon,
                    transformation_impact_meters=0.0,
                    target_distance_2d=target_distance,
                    selection_mode="gimbal_pointing"
                )
            except Exception as e:
                print(f"[TARGET SELECT] Logging error: {e}")
        
        print(f"{'='*60}\n")
        
        # Set this as the locked target (freeze ADSB at this position)
        self.gimbal_target_state['lat'] = target_lat
        self.gimbal_target_state['lon'] = target_lon
        self.gimbal_target_state['distance'] = self.gimbal_current_pointing['distance']
        self.gimbal_target_state['selected'] = True
        
        # Start gimbal TARGET TRACKING (track ground target as aircraft moves)
        if self.gimbal.is_connected:
            # Update aircraft state for gimbal locker
            if self.aircraft_state:
                self.gimbal_locker.update_aircraft_state(self.aircraft_state)
                print(f"[GIMBAL] Aircraft state: {self.aircraft_state.get('lat'):.6f}, {self.aircraft_state.get('lon'):.6f}, heading: {self.aircraft_state.get('heading', 0):.1f}°")
            else:
                print("[GIMBAL] Warning: No aircraft state available for gimbal locking")
                return
                
            # Start tracking the ground target that gimbal is currently pointing at
            self.gimbal_locker.start_locking(target_lat, target_lon, target_alt)
            print(f"[GIMBAL] Target tracking started - ground target: {target_lat:.6f}, {target_lon:.6f}")
            print(f"[GIMBAL] Gimbal will automatically move to keep target in view as aircraft moves")
            
        else:
            print("[GIMBAL] Cannot lock - gimbal not connected")
    
    def clear_gimbal_target(self):
        """Clear selected gimbal target and unlock gimbal"""
        # Stop gimbal target tracking
        self.gimbal_locker.stop_locking()
        
        # Clear target state
        self.gimbal_target_state['lat'] = None
        self.gimbal_target_state['lon'] = None
        self.gimbal_target_state['distance'] = 0.0
        self.gimbal_target_state['selected'] = False
        
        # Update UI display
        self.lbl_selected_target.setText("None selected")
        
        print("[GIMBAL] Target cleared and gimbal unlocked")
    
    def start_gimbal_tracking(self):
        """Start loitering around current gimbal target"""
        if self.gimbal_target_state['lat'] is None:
            print("[GIMBAL] Error: No target available for tracking")
            return
            
        if not self.mavlink.connected:
            print("[GIMBAL] Error: No MAVLink connection")
            return
        
        # Get target coordinates
        target_lat = self.gimbal_target_state['lat']
        target_lon = self.gimbal_target_state['lon']
        
        # Get radius and altitude
        radius = self.radius_spin.value() if hasattr(self, 'radius_spin') else 100
        alt = self.aircraft_state.get('alt_amsl', 100.0)
        
        # Start loitering around target
        success = self.mavlink.set_loiter_mode(target_lat, target_lon, alt, radius)
        
        if success:
            self.gimbal_tracking_active = True
            
            # CRITICAL FIX: Start gimbal locking on the target
            if self.gimbal.is_connected:
                target_alt = self.gimbal_target_state.get('alt', 0.0)
                self.gimbal_locker.start_locking(target_lat, target_lon, target_alt)
                print(f"[GIMBAL] Started gimbal lock on target: {target_lat:.6f}, {target_lon:.6f} @ {target_alt}m")
            else:
                print("[GIMBAL] Warning: Gimbal not connected - only loitering without gimbal lock")
            
            # Update UI state
            self.btn_start_gimbal_tracking.setEnabled(False)
            self.btn_stop_gimbal_tracking.setEnabled(True)
            
            print(f"[GIMBAL] Started loitering around target: {target_lat:.6f}, {target_lon:.6f} @ {alt}m (R:{radius}m)")
        else:
            print("[GIMBAL] Failed to start loitering")
    
    def stop_gimbal_tracking(self):
        """Unlock gimbal but continue loitering"""
        self.gimbal_tracking_active = False
        
        # Stop gimbal lock but keep aircraft loitering
        self.gimbal_locker.stop_locking()
        self.gimbal_target_state['selected'] = False
        
        # Update UI state
        self.btn_start_gimbal_tracking.setEnabled(True)
        self.btn_stop_gimbal_tracking.setEnabled(False)
        
        print("[GIMBAL] Unlocked gimbal - aircraft continues loitering")
    
    def return_to_mission(self):
        """Return to mission"""
        if not self.mavlink.connected:
            print("Error: No MAVLink connection")
            return
            
        success = self.mavlink.set_mission_mode()
        
        if success:
            # Stop any active tracking
            if self.tracker.active:
                self.tracker.stop_tracking()
                self.gimbal_locker.stop_locking()
                self.btn_start_tracking.setEnabled(True)
                self.btn_stop_tracking.setEnabled(False)
            
            print("Returned to mission mode")
        else:
            print("Failed to return to mission (no mission loaded or MAVLink error)")
    
    def on_ge_target_changed(self, lat, lon, alt):
        """Handle Google Earth target change"""
        if self.gimbal and self.gimbal.logger:
            self.gimbal.logger.log_target_set(lat, lon, alt, "google_earth_waypoint")
    
    def get_telemetry_for_ge(self):
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
            'speed': 0.0,
            'battery_voltage': 0.0,
            'flight_mode': 'UNKNOWN',
            'gimbal_yaw': gimbal_yaw,
            'gimbal_pitch': gimbal_pitch,
            'gimbal_roll': gimbal_roll
        }
    
    def closeEvent(self, event):
        """Handle application close"""
        self._closing = True
        
        # Finalize session logging and trigger analysis
        try:
            print("[SHUTDOWN] Finalizing session...")
            # Import to avoid circular import issues
            from gimbal_app.session_logging.session_logger import finalize_current_session
            session_dir = finalize_current_session()
            if session_dir:
                print(f"[SHUTDOWN] Session finalized: {session_dir}")
                
                # Check if session has data to analyze
                import os
                csv_files = [
                    os.path.join(session_dir, "gimbal_performance.csv"),
                    os.path.join(session_dir, "coordinate_calculations.csv"),
                    os.path.join(session_dir, "target_selections.csv")
                ]
                
                has_data = any(os.path.exists(f) and os.path.getsize(f) > 100 for f in csv_files)
                
                if has_data:
                    print("[SHUTDOWN] Session has data, running analysis...")
                    
                    # Calculate correct script path
                    current_file = os.path.abspath(__file__)
                    gimbal_app_dir = os.path.dirname(os.path.dirname(current_file))
                    analysis_script = os.path.join(gimbal_app_dir, "session_logging", "analyze_session.py")
                    
                    print(f"[SHUTDOWN] Analysis script path: {analysis_script}")
                    print(f"[SHUTDOWN] Script exists: {os.path.exists(analysis_script)}")
                    
                    if os.path.exists(analysis_script):
                        try:
                            # Run analysis with visible output for debugging
                            import subprocess
                            print(f"[SHUTDOWN] Starting analysis for: {session_dir}")
                            
                            # Use Python from virtual environment
                            project_root = os.path.dirname(os.path.dirname(os.path.dirname(current_file)))
                            venv_python = os.path.join(project_root, "venv", "bin", "python3")
                            
                            # Fallback to system python if venv not found
                            if not os.path.exists(venv_python):
                                venv_python = "python3"
                            
                            print(f"[SHUTDOWN] Using Python: {venv_python}")
                            result = subprocess.run([
                                venv_python, analysis_script, session_dir
                            ], capture_output=True, text=True, timeout=30)
                            
                            if result.returncode == 0:
                                print(f"[SHUTDOWN] Analysis completed successfully")
                                if result.stdout:
                                    print(f"[SHUTDOWN] Analysis output: {result.stdout}")
                            else:
                                print(f"[SHUTDOWN] Analysis failed with return code: {result.returncode}")
                                if result.stderr:
                                    print(f"[SHUTDOWN] Analysis error: {result.stderr}")
                                    
                        except subprocess.TimeoutExpired:
                            print(f"[SHUTDOWN] Analysis timed out")
                        except Exception as e:
                            print(f"[SHUTDOWN] Failed to start analysis: {e}")
                    else:
                        print(f"[SHUTDOWN] Analysis script not found at: {analysis_script}")
                else:
                    print("[SHUTDOWN] No significant data in session, skipping analysis")
            else:
                print("[SHUTDOWN] No session to finalize")
        except Exception as e:
            print(f"[SHUTDOWN] Session finalization error: {e}")
            import traceback
            traceback.print_exc()
        
        # Stop camera stream first
        if self.camera_stream:
            print("[SHUTDOWN] Stopping camera stream...")
            self.camera_stream.stop_stream()
            time.sleep(0.2)  # Give camera thread time to stop
        
        # Stop all systems with delays to prevent race conditions
        print("[SHUTDOWN] Stopping tracking systems...")
        self.tracker.stop_tracking()
        time.sleep(0.1)
        
        print("[SHUTDOWN] Stopping gimbal locker...")
        self.gimbal_locker.stop_locking()
        time.sleep(0.1)
        
        print("[SHUTDOWN] Stopping gimbal...")
        self.gimbal.stop()
        time.sleep(0.1)
        
        print("[SHUTDOWN] Stopping SBS publisher...")
        self.sbs.stop()
        time.sleep(0.1)
        
        # MAVLinkHandler doesn't need explicit stopping
        
        if self.google_earth:
            print("[SHUTDOWN] Cleaning up Google Earth...")
            self.google_earth.cleanup()
            
        print("[SHUTDOWN] All systems stopped")
        
        event.accept()
    
    def toggle_menu(self):
        """Toggle the lateral menu collapse/expand"""
        if self.menu_collapsed:
            # Expand menu
            self.lateral_menu.setFixedWidth(self.menu_width_expanded)
            self.menu_content_widget.show()
            self.menu_toggle_btn.setText("☰")
            # Reset margins for expanded state
            self.menu_layout.setContentsMargins(10, 10, 10, 10)
            self.menu_collapsed = False
        else:
            # Collapse menu
            self.lateral_menu.setFixedWidth(self.menu_width_collapsed)
            self.menu_content_widget.hide()
            self.menu_toggle_btn.setText("▶")
            # Center the button by adjusting margins symmetrically
            margin = (self.menu_width_collapsed - 40) // 2  # 40 is button width
            self.menu_layout.setContentsMargins(margin, 10, margin, 10)
            self.menu_collapsed = True
    
    def resizeEvent(self, event):
        """Handle window resize to maintain proper camera aspect ratio"""
        super().resizeEvent(event)
        
        # Update status overlay position
        if hasattr(self, 'status_overlay') and self.status_overlay.parent():
            # Keep overlay in top-left corner with some margin
            self.status_overlay.move(10, 10)
            # Let it auto-size based on content
            self.status_overlay.adjustSize()
    
    def wheelEvent(self, event: QWheelEvent):
        """Handle mouse wheel events for zoom control"""
        if not self.gimbal.is_connected:
            super().wheelEvent(event)
            return
        
        # Check if mouse is over the main camera view area (not over menu)
        mouse_pos = event.position().toPoint()
        
        # If mouse is over the left menu area, ignore wheel events for zoom
        if mouse_pos.x() < 300:  # Menu width is approximately 280px
            super().wheelEvent(event)
            return
        
        # Get wheel delta (positive = scroll up = zoom in, negative = scroll down = zoom out)
        delta = event.angleDelta().y()
        
        if delta > 0:
            # Scroll up = Zoom in
            self.gimbal.zoom_in()
            # Increased sensitivity - longer zoom duration
            QTimer.singleShot(500, self.gimbal.zoom_hold)
        elif delta < 0:
            # Scroll down = Zoom out  
            self.gimbal.zoom_out()
            # Increased sensitivity - longer zoom duration
            QTimer.singleShot(500, self.gimbal.zoom_hold)
        
        event.accept()
    
    def mousePressEvent(self, event: QMouseEvent):
        """Handle mouse press events for drag gimbal control and centering"""
        if self.gimbal.is_connected:
            mouse_pos = event.position().toPoint()
            
            # If mouse is over the left menu area, ignore for gimbal control
            if mouse_pos.x() < 300:  # Menu width is approximately 280px
                super().mousePressEvent(event)
                return
            
            if event.button() == Qt.LeftButton:
                # Start drag mode for left click
                self.mouse_drag["active"] = True
                self.mouse_drag["last_pos"] = mouse_pos
                # Stop centering if manual control starts
                if self.centering_active:
                    self.centering_active = False
                    print("[UI] Manual control detected - stopping centering")
                event.accept()
            elif event.button() == Qt.MiddleButton:
                # Center gimbal on middle mouse button (scroll wheel click)
                print("[GIMBAL] Middle click detected - centering gimbal")
                self.gimbal.center_gimbal()
                event.accept()
            else:
                super().mousePressEvent(event)
        else:
            super().mousePressEvent(event)
    
    def mouseReleaseEvent(self, event: QMouseEvent):
        """Handle mouse release events"""
        if event.button() == Qt.LeftButton and self.mouse_drag["active"]:
            # Stop drag mode
            self.mouse_drag["active"] = False
            self.mouse_drag["last_pos"] = None
            
            # Stop gimbal movement
            if self.gimbal.is_connected:
                self.gimbal.jog(0, 0)
            
            event.accept()
        else:
            super().mouseReleaseEvent(event)
    
    def mouseMoveEvent(self, event: QMouseEvent):
        """Handle mouse move events for drag gimbal control"""
        if self.mouse_drag["active"] and self.mouse_drag["last_pos"] and self.gimbal.is_connected:
            current_pos = event.position().toPoint()
            last_pos = self.mouse_drag["last_pos"]
            
            # Calculate movement delta
            dx = current_pos.x() - last_pos.x()
            dy = current_pos.y() - last_pos.y()
            
            # Convert mouse movement to gimbal speeds
            # Positive dx = move right (positive yaw)
            # Positive dy = move down (negative pitch, since screen Y is inverted)
            sensitivity = self.mouse_drag["sensitivity"]
            yaw_speed = int(dx * sensitivity * 3)  # Increased scaling for more responsive control
            pitch_speed = int(-dy * sensitivity * 3)  # Negative because screen Y is inverted
            
            # Clamp speeds to reasonable limits
            yaw_speed = max(-100, min(100, yaw_speed))
            pitch_speed = max(-100, min(100, pitch_speed))
            
            # Send gimbal command
            self.gimbal.jog(yaw_speed, pitch_speed)
            
            # Update last position
            self.mouse_drag["last_pos"] = current_pos
            event.accept()
        else:
            super().mouseMoveEvent(event)


def main():
    """Main entry point"""
    print("[MAIN] Creating QApplication...")
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # Use modern Fusion style
    
    print("[MAIN] Creating ModernGimbalApp window...")
    try:
        window = ModernGimbalApp()
        print("[MAIN] Window created successfully")
        
        print("[MAIN] Showing window...")
        window.show()
        print("[MAIN] Window shown, starting event loop...")
        
        return app.exec()
    except Exception as e:
        print(f"[MAIN] Error creating/showing window: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())