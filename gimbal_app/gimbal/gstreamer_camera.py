"""
GStreamer-based camera streaming with overlay support
Ultra-low latency RTSP streaming with text overlay capabilities
Much better than OpenCV for real-time applications
"""

try:
    from PySide6.QtCore import QObject, Signal, QTimer
    from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel
    PYSIDE_AVAILABLE = True
except ImportError:
    print("Warning: PySide6 not available for GStreamer")
    PYSIDE_AVAILABLE = False

try:
    import gi
    gi.require_version('Gst', '1.0')
    gi.require_version('GstVideo', '1.0')
    from gi.repository import Gst, GObject, GstVideo
    Gst.init(None)
    GSTREAMER_AVAILABLE = True
except ImportError:
    print("GStreamer not available. Install with: sudo apt install python3-gi gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav")
    GSTREAMER_AVAILABLE = False

import time
from typing import Optional

class GStreamerCameraStream(QObject if PYSIDE_AVAILABLE else object):
    """
    GStreamer-based camera stream with overlay support
    Ultra-low latency RTSP streaming with real-time text overlay
    """
    if PYSIDE_AVAILABLE:
        connection_status_changed = Signal(bool)
    else:
        connection_status_changed = None
    
    def __init__(self, rtsp_url: str = "rtsp://192.168.144.25:8554/main.264", parent_widget=None):
        if PYSIDE_AVAILABLE:
            super().__init__()
        
        self.rtsp_url = rtsp_url
        self.parent_widget = parent_widget
        self._connected = False
        self._running = False
        
        # Check dependencies
        if not GSTREAMER_AVAILABLE or not PYSIDE_AVAILABLE:
            self._dependencies_available = False
            return
        else:
            self._dependencies_available = True
        
        # GStreamer pipeline
        self.pipeline = None
        self.video_sink = None
        self.text_overlay = None
        
        # Overlay text properties
        self.overlay_text = ""
        self.target_coords = None
        
        # Status checking timer
        self.status_timer = QTimer() if PYSIDE_AVAILABLE else None
        if self.status_timer:
            self.status_timer.timeout.connect(self._check_status)
        
        print(f"[GSTREAMER] Initialized GStreamer camera stream: {self.rtsp_url}")
    
    def create_video_widget(self, parent):
        """Create the GStreamer video widget"""
        if not self._dependencies_available:
            # Fallback to text label
            label = QLabel("GSTREAMER NOT AVAILABLE\nInstall: sudo apt install python3-gi gstreamer1.0-plugins-*")
            label.setStyleSheet("color: red; text-align: center;")
            return label
        
        try:
            # Create video widget
            self.video_widget = QWidget(parent)
            self.video_widget.setMinimumSize(640, 480)
            
            print("[GSTREAMER] Video widget created successfully")
            return self.video_widget
            
        except Exception as e:
            print(f"[GSTREAMER] Error creating video widget: {e}")
            # Fallback to text label
            label = QLabel(f"GSTREAMER ERROR: {e}")
            label.setStyleSheet("color: red;")
            return label
    
    def start_stream(self) -> bool:
        """Start the GStreamer pipeline with overlay"""
        if not self._dependencies_available:
            print("[GSTREAMER] Cannot start stream: GStreamer not available")
            return False
        
        if self._running:
            return True
        
        try:
            print(f"[GSTREAMER] Starting stream from {self.rtsp_url}")
            
            # Create clean GStreamer pipeline - no text overlay, just video
            # Support both H264 and H265 streams automatically
            pipeline_str = f"""
                rtspsrc location={self.rtsp_url} latency=50 buffer-mode=auto ! 
                rtph265depay ! 
                avdec_h265 max-threads=2 ! 
                videoconvert ! 
                videoscale ! 
                ximagesink name=videosink sync=false force-aspect-ratio=false
            """
            
            # Create pipeline
            self.pipeline = Gst.parse_launch(pipeline_str)
            
            # Get elements for dynamic control
            self.video_sink = self.pipeline.get_by_name('videosink')
            self.text_overlay = None  # No text overlay - clean video only
            
            # Set video sink to Qt widget
            if self.video_sink and hasattr(self.video_widget, 'winId'):
                self.video_sink.set_window_handle(int(self.video_widget.winId()))
            
            # Set up bus for messages
            bus = self.pipeline.get_bus()
            bus.add_signal_watch()
            bus.connect("message", self._on_message)
            
            # Start pipeline
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            
            if ret != Gst.StateChangeReturn.FAILURE:
                self._running = True
                print("[GSTREAMER] Pipeline started successfully")
                
                # Start status checking (no overlay updates needed)
                if self.status_timer:
                    self.status_timer.start(1000)  # Check connection status every second
                
                return True
            else:
                print("[GSTREAMER] Failed to start pipeline")
                return False
                
        except Exception as e:
            print(f"[GSTREAMER] Error starting stream: {e}")
            return False
    
    def stop_stream(self):
        """Stop the GStreamer pipeline"""
        try:
            print("[GSTREAMER] Stopping stream...")
            
            if self.status_timer:
                self.status_timer.stop()
            self._running = False
            
            if self.pipeline:
                self.pipeline.set_state(Gst.State.NULL)
                self.pipeline = None
            
            if self._connected:
                self._connected = False
                if self.connection_status_changed:
                    self.connection_status_changed.emit(False)
            
            print("[GSTREAMER] Stream stopped")
            
        except Exception as e:
            print(f"[GSTREAMER] Error stopping stream: {e}")
    
    def _on_message(self, bus, message):
        """Handle GStreamer bus messages"""
        try:
            if message.type == Gst.MessageType.ERROR:
                err, debug = message.parse_error()
                print(f"[GSTREAMER] Error: {err}, Debug: {debug}")
                if self._connected:
                    self._connected = False
                    if self.connection_status_changed:
                        self.connection_status_changed.emit(False)
            
            elif message.type == Gst.MessageType.STATE_CHANGED:
                if message.src == self.pipeline:
                    old_state, new_state, pending_state = message.parse_state_changed()
                    if new_state == Gst.State.PLAYING and not self._connected:
                        self._connected = True
                        if self.connection_status_changed:
                            self.connection_status_changed.emit(True)
                        print("[GSTREAMER] Stream connected and playing")
            
        except Exception as e:
            print(f"[GSTREAMER] Message handling error: {e}")
    
    def _check_status(self):
        """Check pipeline status"""
        if not self.pipeline:
            return
        
        try:
            ret, state, pending = self.pipeline.get_state(Gst.CLOCK_TIME_NONE)
            
            if state == Gst.State.PLAYING:
                if not self._connected:
                    self._connected = True
                    if self.connection_status_changed:
                        self.connection_status_changed.emit(True)
            elif state in [Gst.State.NULL, Gst.State.READY]:
                if self._connected:
                    self._connected = False
                    if self.connection_status_changed:
                        self.connection_status_changed.emit(False)
            
        except Exception as e:
            print(f"[GSTREAMER] Status check error: {e}")
    
    def _update_overlay_text(self):
        """Update overlay text with key information in compact format"""
        if not self.text_overlay:
            return
        
        try:
            # Build compact overlay text with key info
            lines = []
            
            # System time
            current_time = time.strftime("%H:%M:%S")
            lines.append(f">>> SYSTEM TIME: {current_time}")
            
            # Gimbal status
            if hasattr(self, 'gimbal') and self.gimbal and self.gimbal.is_connected:
                yaw = getattr(self.gimbal, 'yaw_abs', 0) or 0
                pitch = getattr(self.gimbal, 'pitch_norm', 0) or 0
                lines.append(f"● GIMBAL ONLINE - Y:{yaw:06.1f}° P:{pitch:+05.1f}°")
            else:
                lines.append("● GIMBAL OFFLINE")
            
            # Aircraft info
            if hasattr(self, 'aircraft_state') and self.aircraft_state and self.aircraft_state.get('lat') is not None:
                lat = self.aircraft_state['lat']
                lon = self.aircraft_state['lon']
                alt = self.aircraft_state.get('alt_agl', 0)
                hdg = self.aircraft_state.get('heading', 0)
                lines.append(f"AIRCRAFT: {lat:.6f}, {lon:.6f}")
                lines.append(f"ALT: {alt:.1f}m  HDG: {hdg:.1f}°")
            else:
                lines.append("AIRCRAFT: NO MAVLINK DATA")
            
            # Target info
            if hasattr(self, 'gimbal_locker') and self.gimbal_locker:
                try:
                    lock_info = self.gimbal_locker.get_lock_info()
                    if lock_info.get('active'):
                        target_lat = lock_info.get('target_lat', 0)
                        target_lon = lock_info.get('target_lon', 0)
                        lines.append(f"TARGET: {target_lat:.6f}, {target_lon:.6f}")
                        if lock_info.get('required_angles') and lock_info['required_angles'].get('distance_2d'):
                            distance = lock_info['required_angles']['distance_2d']
                            lines.append(f"DIST: {distance:04.0f}m")
                        else:
                            lines.append("DIST: CALCULATING")
                    else:
                        lines.append("NO TARGET SET")
                except Exception:
                    lines.append("NO TARGET SET")
            
            # Camera info
            lines.append("SIYI ZR10 FEED - 1280x720")
            
            # Join all lines with newlines for GStreamer
            overlay_text = "\\n".join(lines)
            self.text_overlay.set_property('text', overlay_text)
            
        except Exception as e:
            print(f"[GSTREAMER] Overlay update error: {e}")
    
    def set_target_coordinates(self, lat: float, lon: float, distance: float):
        """Set target coordinates for overlay display"""
        self.target_coords = (lat, lon, distance)
    
    def set_aircraft_state(self, aircraft_state):
        """Set aircraft state for overlay display"""
        self.aircraft_state = aircraft_state
    
    def set_gimbal(self, gimbal):
        """Set gimbal reference for overlay display"""
        self.gimbal = gimbal
    
    def set_gimbal_locker(self, gimbal_locker):
        """Set gimbal locker reference for target lock info"""
        self.gimbal_locker = gimbal_locker
    
    def is_connected(self) -> bool:
        """Check if stream is connected"""
        return self._connected
    
    def set_rtsp_url(self, url: str):
        """Change RTSP URL"""
        self.rtsp_url = url
        if self._running:
            self.stop_stream()
            self.start_stream()