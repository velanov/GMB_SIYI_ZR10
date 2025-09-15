"""
VLC-based camera streaming for low latency RTSP
Much better than OpenCV for real-time streaming
"""

try:
    from PySide6.QtCore import QObject, Signal, QTimer
    from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel
    PYSIDE_AVAILABLE = True
except ImportError:
    print("Warning: PySide6 not available for VLC streaming")
    PYSIDE_AVAILABLE = False

try:
    import vlc
    VLC_AVAILABLE = True
except ImportError:
    print("VLC not available. Install with: pip install python-vlc")
    VLC_AVAILABLE = False

import time
from typing import Optional

class VLCCameraStream(QObject):
    """
    VLC-based camera stream for ultra-low latency RTSP streaming
    Much better than OpenCV for real-time applications
    """
    connection_status_changed = Signal(bool)
    
    def __init__(self, rtsp_url: str = "rtsp://192.168.144.25:8554/main.264", parent_widget=None):
        super().__init__()
        
        self.rtsp_url = rtsp_url
        self.parent_widget = parent_widget
        self._connected = False
        self._running = False
        
        # Check dependencies
        if not VLC_AVAILABLE or not PYSIDE_AVAILABLE:
            self._dependencies_available = False
            return
        else:
            self._dependencies_available = True
        
        # VLC setup
        self.vlc_instance = None
        self.vlc_player = None
        self.vlc_widget = None
        
        # Status checking timer
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self._check_status)
        
        print(f"[VLC CAMERA] Initialized VLC camera stream: {self.rtsp_url}")
    
    def create_video_widget(self, parent):
        """Create the VLC video widget"""
        if not self._dependencies_available:
            # Fallback to text label
            label = QLabel("VLC NOT AVAILABLE\nInstall with: pip install python-vlc")
            label.setStyleSheet("color: red; text-align: center;")
            return label
        
        try:
            # Create VLC widget
            self.vlc_widget = QWidget(parent)
            self.vlc_widget.setMinimumSize(640, 480)
            
            # Create VLC instance with low-latency options
            vlc_args = [
                "--intf", "dummy",          # No interface
                "--no-audio",               # Disable audio for lower latency
                "--network-caching=50",     # Ultra-low network cache (50ms)
                "--live-caching=50",        # Low live cache
                "--sout-mux-caching=50",    # Low mux cache
                "--rtsp-caching=50",        # Low RTSP cache
                "--no-drop-late-frames",    # Don't drop frames
                "--skip-frames=0",          # Don't skip frames
            ]
            
            self.vlc_instance = vlc.Instance(vlc_args)
            self.vlc_player = self.vlc_instance.media_player_new()
            
            # Set video output to Qt widget
            if hasattr(self.vlc_player, 'set_xwindow'):
                self.vlc_player.set_xwindow(int(self.vlc_widget.winId()))
            elif hasattr(self.vlc_player, 'set_hwnd'):
                self.vlc_player.set_hwnd(int(self.vlc_widget.winId()))
            
            print("[VLC CAMERA] VLC video widget created successfully")
            return self.vlc_widget
            
        except Exception as e:
            print(f"[VLC CAMERA] Error creating VLC widget: {e}")
            # Fallback to text label
            label = QLabel(f"VLC ERROR: {e}")
            label.setStyleSheet("color: red;")
            return label
    
    def start_stream(self) -> bool:
        """Start the VLC stream"""
        if not self._dependencies_available:
            print("[VLC CAMERA] Cannot start stream: VLC not available")
            return False
        
        if self._running:
            return True
        
        try:
            print(f"[VLC CAMERA] Starting stream from {self.rtsp_url}")
            
            # Create media with RTSP URL
            media = self.vlc_instance.media_new(self.rtsp_url)
            
            # Set media options for low latency
            media.add_option(":network-caching=50")
            media.add_option(":rtsp-caching=50") 
            media.add_option(":live-caching=50")
            
            # Set media to player
            self.vlc_player.set_media(media)
            
            # Start playback
            result = self.vlc_player.play()
            
            if result == 0:  # Success
                self._running = True
                print("[VLC CAMERA] Stream started successfully")
                
                # Start status checking
                self.status_timer.start(1000)  # Check every second
                
                return True
            else:
                print(f"[VLC CAMERA] Failed to start stream: {result}")
                return False
                
        except Exception as e:
            print(f"[VLC CAMERA] Error starting stream: {e}")
            return False
    
    def stop_stream(self):
        """Stop the VLC stream"""
        try:
            print("[VLC CAMERA] Stopping stream...")
            
            self.status_timer.stop()
            self._running = False
            
            if self.vlc_player:
                self.vlc_player.stop()
            
            if self._connected:
                self._connected = False
                self.connection_status_changed.emit(False)
            
            print("[VLC CAMERA] Stream stopped")
            
        except Exception as e:
            print(f"[VLC CAMERA] Error stopping stream: {e}")
    
    def _check_status(self):
        """Check VLC player status"""
        if not self.vlc_player:
            return
        
        try:
            state = self.vlc_player.get_state()
            
            # VLC states: 0=NothingSpecial, 1=Opening, 2=Buffering, 3=Playing, 4=Paused, 5=Stopped, 6=Ended, 7=Error
            if state == 3:  # Playing
                if not self._connected:
                    self._connected = True
                    self.connection_status_changed.emit(True)
                    print("[VLC CAMERA] Stream connected and playing")
            elif state in [5, 6, 7]:  # Stopped, Ended, Error
                if self._connected:
                    self._connected = False
                    self.connection_status_changed.emit(False)
                    print(f"[VLC CAMERA] Stream disconnected (state: {state})")
            
        except Exception as e:
            print(f"[VLC CAMERA] Status check error: {e}")
    
    def is_connected(self) -> bool:
        """Check if stream is connected"""
        return self._connected
    
    def set_rtsp_url(self, url: str):
        """Change RTSP URL"""
        self.rtsp_url = url
        if self._running:
            self.stop_stream()
            self.start_stream()