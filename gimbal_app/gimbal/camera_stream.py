"""
SIYI ZR10 Camera Streaming Integration
Handles RTSP video stream from SIYI ZR10 gimbal camera
"""

try:
    import cv2
except ImportError:
    print("Warning: OpenCV not installed. Camera streaming will not be available.")
    cv2 = None

import threading
import time
from typing import Optional, Callable

# Import target calculator for center coordinate calculation
try:
    from ..calc.target_calculator import TargetCalculator
except ImportError:
    print("Warning: TargetCalculator not available. Center target coordinates will not be calculated.")
    TargetCalculator = None

try:
    import numpy as np
except ImportError:
    print("Warning: NumPy not installed. Camera streaming will not be available.")
    np = None

try:
    from PySide6.QtCore import QThread, Signal, QTimer
    from PySide6.QtGui import QImage, QPixmap
except ImportError:
    print("Warning: PySide6 not available. Using stub classes.")
    # Create stub classes for non-GUI environments
    class QThread:
        def __init__(self): pass
        def start(self): pass
        def wait(self, timeout=None): pass
        def isRunning(self): return False
    
    class Signal:
        def __init__(self, *args): pass
        def emit(self, *args): pass
        def connect(self, func): pass
    
    class QTimer:
        def __init__(self): pass
    
    class QImage:
        Format_RGB888 = None
        def __init__(self, *args): pass
    
    class QPixmap:
        @staticmethod
        def fromImage(image): return None


class SiyiCameraStream(QThread):
    """
    SIYI ZR10 Camera RTSP stream handler for PySide6 integration
    """
    # Signal emitted when new frame is available
    frame_ready = Signal(QImage)
    connection_status_changed = Signal(bool)
    
    def __init__(self, gimbal_ip: str = "192.168.144.25", stream_port: int = 8554, gimbal=None, aircraft_state=None, gimbal_locker=None):
        super().__init__()
        
        # Check dependencies
        if cv2 is None or np is None:
            print("[CAMERA] OpenCV or NumPy not available. Camera streaming disabled.")
            self._dependencies_available = False
            return
        else:
            self._dependencies_available = True
        
        # References for status overlay
        self.gimbal = gimbal
        self.aircraft_state = aircraft_state
        self.gimbal_locker = gimbal_locker
        
        # Stream configuration
        self.gimbal_ip = gimbal_ip
        self.stream_port = stream_port
        self.rtsp_url = f"rtsp://{gimbal_ip}:{stream_port}/main.264"
        
        # OpenCV capture object
        self.cap: Optional[cv2.VideoCapture] = None
        
        # Threading control
        self._running = False
        self._connected = False
        
        # Frame processing
        self.frame_rate = 30  # Target FPS
        self.frame_width = 1920
        self.frame_height = 1080
        
        # Retry settings
        self.max_reconnect_attempts = 5
        self.reconnect_delay = 2.0
        
        # Timeout detection
        self.last_frame_time = 0
        self.frame_timeout_threshold = 5.0  # Consider stream dead after 5 seconds of no frames
        
        print(f"[CAMERA] Initialized SIYI camera stream: {self.rtsp_url}")
    
    def start_stream(self) -> bool:
        """Start the camera stream"""
        if not self._dependencies_available:
            print("[CAMERA] Cannot start stream: missing dependencies")
            return False
            
        if self._running:
            return True
            
        print(f"[CAMERA] Starting camera stream from {self.rtsp_url}")
        self._running = True
        self.start()  # Start QThread
        return True
    
    def stop_stream(self):
        """Stop the camera stream"""
        print("[CAMERA] Stopping camera stream")
        self._running = False
        
        if self.cap:
            self.cap.release()
            self.cap = None
            
        if self._connected:
            self._connected = False
            self.connection_status_changed.emit(False)
        
        # Wait for thread to finish
        if self.isRunning():
            self.wait(3000)  # Wait up to 3 seconds
    
    def run(self):
        """Main thread loop for capturing frames"""
        reconnect_attempts = 0
        
        while self._running:
            try:
                # Initialize capture if not connected
                if not self._connected:
                    if not self._connect_to_stream():
                        reconnect_attempts += 1
                        if reconnect_attempts >= self.max_reconnect_attempts:
                            print(f"[CAMERA] Max reconnection attempts reached. Stopping.")
                            break
                        
                        print(f"[CAMERA] Reconnection attempt {reconnect_attempts}/{self.max_reconnect_attempts}")
                        time.sleep(self.reconnect_delay)
                        continue
                    else:
                        reconnect_attempts = 0  # Reset counter on successful connection
                
                # Check for stream timeout (no frames for too long)
                current_time = time.time()
                if self.last_frame_time > 0 and (current_time - self.last_frame_time) > self.frame_timeout_threshold:
                    print(f"[CAMERA] Stream timeout detected ({current_time - self.last_frame_time:.1f}s), reconnecting")
                    self._disconnect()
                    continue
                
                # Capture frame with crash protection
                try:
                    ret, frame = self.cap.read()
                except Exception as read_error:
                    print(f"[CAMERA] Frame read error: {read_error}")
                    ret, frame = False, None
                
                if not ret or frame is None:
                    print("[CAMERA] Failed to read frame, attempting reconnection")
                    self._disconnect()
                    continue
                
                # Update last successful frame time
                self.last_frame_time = current_time
                
                # Add cyberpunk overlay to frame
                frame_with_overlay = self._add_cyberpunk_overlay(frame)
                
                # DEBUG: Print overlay status
                if hasattr(self, '_debug_overlay_count'):
                    self._debug_overlay_count += 1
                else:
                    self._debug_overlay_count = 1
                
                if self._debug_overlay_count % 30 == 0:  # Every 30 frames (1 second at 30fps)
                    print(f"[CAMERA] Overlay applied to frame {self._debug_overlay_count}")
                
                # Convert frame to QImage and emit signal
                qt_image = self._cv_to_qimage(frame_with_overlay)
                if qt_image:
                    self.frame_ready.emit(qt_image)
                
                # Control frame rate
                time.sleep(1.0 / self.frame_rate)
                
            except Exception as e:
                print(f"[CAMERA] Stream error: {e}")
                self._disconnect()
                time.sleep(1.0)
        
        # Cleanup
        self._disconnect()
        print("[CAMERA] Camera stream thread stopped")
    
    def _connect_to_stream(self) -> bool:
        """Attempt to connect to RTSP stream"""
        try:
            print(f"[CAMERA] Connecting to {self.rtsp_url}")
            
            # Check if RTSP URL format is valid
            if not self.rtsp_url.startswith('rtsp://'):
                print(f"[CAMERA] Invalid RTSP URL format: {self.rtsp_url}")
                return False
            
            # Release existing capture safely
            if self.cap:
                try:
                    self.cap.release()
                except Exception as e:
                    print(f"[CAMERA] Error releasing previous capture: {e}")
                self.cap = None
            
            # Create new capture with crash protection - try multiple backends
            backends_to_try = [
                ("GSTREAMER", cv2.CAP_GSTREAMER),
                ("FFMPEG", cv2.CAP_FFMPEG),
                ("V4L2", cv2.CAP_V4L2),
                ("ANY", cv2.CAP_ANY)
            ]
            
            self.cap = None
            for backend_name, backend_id in backends_to_try:
                try:
                    print(f"[CAMERA] Trying {backend_name} backend...")
                    self.cap = cv2.VideoCapture(self.rtsp_url, backend_id)
                    
                    if self.cap.isOpened():
                        print(f"[CAMERA] Successfully opened with {backend_name} backend")
                        break
                    else:
                        print(f"[CAMERA] {backend_name} backend failed to open")
                        if self.cap:
                            self.cap.release()
                        self.cap = None
                        
                except Exception as backend_error:
                    print(f"[CAMERA] {backend_name} backend crashed: {backend_error}")
                    if self.cap:
                        try:
                            self.cap.release()
                        except:
                            pass
                        self.cap = None
                    continue
            
            if not self.cap or not self.cap.isOpened():
                print("[CAMERA] All backends failed to open stream")
                return False
                
            # Set buffer size to reduce latency  
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            # Set timeout to prevent freezing (in milliseconds)
            self.cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 3000)     # 3 second open timeout
            self.cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 2000)     # 2 second read timeout
            
            # Set frame dimensions (optional)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            
            # Force low latency for RTSP
            self.cap.set(cv2.CAP_PROP_FPS, 30)                    # Force 30 FPS
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'H264'))  # Force H264 codec
            
            print("[CAMERA] Testing connection with frame read...")
            # Test connection by reading a frame with protection
            ret, frame = self.cap.read()
                
        except Exception as opencv_error:
            print(f"[CAMERA] OpenCV VideoCapture crash protection: {opencv_error}")
            if self.cap:
                try:
                    self.cap.release()
                except:
                    pass
                self.cap = None
            return False
            
        if ret and frame is not None:
            self._connected = True
            self.connection_status_changed.emit(True)
            self.last_frame_time = time.time()  # Initialize frame timer
            print(f"[CAMERA] Successfully connected to camera stream")
            print(f"[CAMERA] Frame size: {frame.shape[1]}x{frame.shape[0]}")
            return True
        else:
            print("[CAMERA] Failed to read test frame")
            if self.cap:
                self.cap.release()
                self.cap = None
            return False
    
    def _disconnect(self):
        """Disconnect from stream"""
        if self._connected:
            print("[CAMERA] Disconnecting from stream")
            self._connected = False
            self.connection_status_changed.emit(False)
        
        if self.cap:
            self.cap.release()
            self.cap = None
    
    def _add_cyberpunk_overlay(self, frame: np.ndarray) -> np.ndarray:
        """Add cyberpunk-style gimbal status overlay to camera frame"""
        if frame is None:
            return frame
            
        try:
            h, w = frame.shape[:2]
            overlay_frame = frame.copy()
            
            # DEBUG: Print data availability (only once every 100 frames to avoid spam)
            if not hasattr(self, '_debug_count'):
                self._debug_count = 0
            self._debug_count += 1
            
            if self._debug_count % 100 == 0:
                print(f"[OVERLAY DEBUG] Aircraft state: {self.aircraft_state is not None}")
                if self.aircraft_state:
                    print(f"[OVERLAY DEBUG] Aircraft lat: {self.aircraft_state.get('lat')}")
                print(f"[OVERLAY DEBUG] Gimbal locker: {self.gimbal_locker is not None}")
                if self.gimbal_locker:
                    lock_info = self.gimbal_locker.get_lock_info()
                    print(f"[OVERLAY DEBUG] Lock active: {lock_info.get('active', False)}")
                print(f"[OVERLAY DEBUG] Gimbal: {self.gimbal is not None}")
                if self.gimbal:
                    print(f"[OVERLAY DEBUG] Gimbal connected: {self.gimbal.is_connected}")
                    print(f"[OVERLAY DEBUG] Gimbal lock enabled: {getattr(self.gimbal, 'gimbal_lock_enabled', 'N/A')}")
            
            # Cyberpunk colors (BGR format for OpenCV)
            GREEN = (0, 255, 0)
            BRIGHT_GREEN = (0, 255, 50)
            ORANGE = (0, 165, 255)
            RED = (0, 0, 255)
            DARK_GREEN = (0, 80, 0)
            
            # Font settings
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.6
            thickness = 1
            line_spacing = 25
            
            # Top-left corner status
            y_pos = 30
            
            # Connection status
            if self.gimbal and self.gimbal.is_connected:
                age = time.time() - self.gimbal.last_update if self.gimbal.last_update else 999
                if age < 1.0:
                    status_text = "● GIMBAL ONLINE"
                    status_color = GREEN
                else:
                    status_text = f"● GIMBAL TIMEOUT ({age:.1f}s)"
                    status_color = ORANGE
            else:
                status_text = "● GIMBAL OFFLINE"
                status_color = RED
            
            cv2.putText(overlay_frame, status_text, (10, y_pos), font, font_scale, status_color, thickness)
            y_pos += line_spacing
            
            # Gimbal angles
            if self.gimbal and self.gimbal.is_connected and self.gimbal.yaw_abs is not None:
                yaw = self.gimbal.yaw_abs or 0
                pitch = self.gimbal.pitch_norm or 0
                angles_text = f"ORIENTATION: Y:{yaw:06.1f}° P:{pitch:+05.1f}°"
                cv2.putText(overlay_frame, angles_text, (10, y_pos), font, font_scale, BRIGHT_GREEN, thickness)
            else:
                angles_text = "ORIENTATION: Y:---.-° P:---.°"
                cv2.putText(overlay_frame, angles_text, (10, y_pos), font, font_scale, DARK_GREEN, thickness)
            y_pos += line_spacing
            
            # Aircraft position (always show, even if no data)
            if self.aircraft_state and self.aircraft_state.get('lat') is not None:
                lat = self.aircraft_state['lat']
                lon = self.aircraft_state['lon']
                alt = self.aircraft_state.get('alt_agl', 0)
                hdg = self.aircraft_state.get('heading', 0)
                
                pos_text = f"AIRCRAFT: {lat:.6f}, {lon:.6f}"
                cv2.putText(overlay_frame, pos_text, (10, y_pos), font, font_scale, GREEN, thickness)
                y_pos += line_spacing
                
                alt_hdg_text = f"ALT: {alt:.1f}m  HDG: {hdg:.1f}°"
                cv2.putText(overlay_frame, alt_hdg_text, (10, y_pos), font, font_scale, GREEN, thickness)
                y_pos += line_spacing
            else:
                pos_text = "AIRCRAFT: NO MAVLINK DATA"
                cv2.putText(overlay_frame, pos_text, (10, y_pos), font, font_scale, ORANGE, thickness)
                y_pos += line_spacing
                
                alt_hdg_text = "ALT: ---m  HDG: ---°"
                cv2.putText(overlay_frame, alt_hdg_text, (10, y_pos), font, font_scale, DARK_GREEN, thickness)
                y_pos += line_spacing
            
            # Target information (top-right corner) - Always show
            # Target lock status
            if self.gimbal and hasattr(self.gimbal, 'gimbal_lock_enabled'):
                lock_text = "LOCK: ENABLED" if self.gimbal.gimbal_lock_enabled else "LOCK: DISABLED"
                lock_color = GREEN if self.gimbal.gimbal_lock_enabled else ORANGE
            else:
                lock_text = "LOCK: UNKNOWN"
                lock_color = DARK_GREEN
            
            text_size = cv2.getTextSize(lock_text, font, font_scale, thickness)[0]
            cv2.putText(overlay_frame, lock_text, (w - text_size[0] - 10, 30), font, font_scale, lock_color, thickness)
            
            # Target coordinates and distance
            if self.gimbal_locker:
                lock_info = self.gimbal_locker.get_lock_info()
                
                if lock_info['active']:
                    target_text = f"TARGET: {lock_info['target_lat']:.6f}, {lock_info['target_lon']:.6f}"
                    text_size = cv2.getTextSize(target_text, font, font_scale, thickness)[0]
                    cv2.putText(overlay_frame, target_text, (w - text_size[0] - 10, 55), font, font_scale, GREEN, thickness)
                    
                    # Distance to target
                    if lock_info.get('required_angles'):
                        distance = lock_info['required_angles'].get('distance_2d', 0)
                        if distance < 100:
                            dist_text = f"DIST: {distance:03.0f}m"
                            dist_color = ORANGE
                        elif distance < 1000:
                            dist_text = f"DIST: {distance:04.0f}m"
                            dist_color = GREEN
                        else:
                            dist_text = f"DIST: {distance/1000:.1f}km"
                            dist_color = GREEN
                        
                        text_size = cv2.getTextSize(dist_text, font, font_scale, thickness)[0]
                        cv2.putText(overlay_frame, dist_text, (w - text_size[0] - 10, 80), font, font_scale, dist_color, thickness)
                    else:
                        dist_text = "DIST: CALCULATING"
                        text_size = cv2.getTextSize(dist_text, font, font_scale, thickness)[0]
                        cv2.putText(overlay_frame, dist_text, (w - text_size[0] - 10, 80), font, font_scale, ORANGE, thickness)
                else:
                    no_target_text = "NO TARGET SET"
                    text_size = cv2.getTextSize(no_target_text, font, font_scale, thickness)[0]
                    cv2.putText(overlay_frame, no_target_text, (w - text_size[0] - 10, 55), font, font_scale, ORANGE, thickness)
                    
                    dist_text = "DIST: ---"
                    text_size = cv2.getTextSize(dist_text, font, font_scale, thickness)[0]
                    cv2.putText(overlay_frame, dist_text, (w - text_size[0] - 10, 80), font, font_scale, DARK_GREEN, thickness)
            else:
                no_locker_text = "NO GIMBAL LOCKER"
                text_size = cv2.getTextSize(no_locker_text, font, font_scale, thickness)[0]
                cv2.putText(overlay_frame, no_locker_text, (w - text_size[0] - 10, 55), font, font_scale, RED, thickness)
                
                dist_text = "DIST: ---"
                text_size = cv2.getTextSize(dist_text, font, font_scale, thickness)[0]
                cv2.putText(overlay_frame, dist_text, (w - text_size[0] - 10, 80), font, font_scale, DARK_GREEN, thickness)
            
            # Center target crosshairs
            center_x, center_y = w // 2, h // 2
            crosshair_size = 20
            
            # Main crosshairs
            cv2.line(overlay_frame, (center_x - crosshair_size, center_y), (center_x + crosshair_size, center_y), GREEN, 2)
            cv2.line(overlay_frame, (center_x, center_y - crosshair_size), (center_x, center_y + crosshair_size), GREEN, 2)
            
            # Center dot
            cv2.circle(overlay_frame, (center_x, center_y), 3, GREEN, -1)
            
            # Target box around center
            box_size = 60
            cv2.rectangle(overlay_frame, 
                         (center_x - box_size, center_y - box_size), 
                         (center_x + box_size, center_y + box_size), 
                         GREEN, 2)
            
            # Corner markers for precision
            corner_size = 10
            # Top-left
            cv2.line(overlay_frame, (center_x - box_size, center_y - box_size), 
                    (center_x - box_size + corner_size, center_y - box_size), GREEN, 2)
            cv2.line(overlay_frame, (center_x - box_size, center_y - box_size), 
                    (center_x - box_size, center_y - box_size + corner_size), GREEN, 2)
            # Top-right
            cv2.line(overlay_frame, (center_x + box_size, center_y - box_size), 
                    (center_x + box_size - corner_size, center_y - box_size), GREEN, 2)
            cv2.line(overlay_frame, (center_x + box_size, center_y - box_size), 
                    (center_x + box_size, center_y - box_size + corner_size), GREEN, 2)
            # Bottom-left
            cv2.line(overlay_frame, (center_x - box_size, center_y + box_size), 
                    (center_x - box_size + corner_size, center_y + box_size), GREEN, 2)
            cv2.line(overlay_frame, (center_x - box_size, center_y + box_size), 
                    (center_x - box_size, center_y + box_size - corner_size), GREEN, 2)
            # Bottom-right
            cv2.line(overlay_frame, (center_x + box_size, center_y + box_size), 
                    (center_x + box_size - corner_size, center_y + box_size), GREEN, 2)
            cv2.line(overlay_frame, (center_x + box_size, center_y + box_size), 
                    (center_x + box_size, center_y + box_size - corner_size), GREEN, 2)
            
            # Target coordinates calculation (where camera center is pointing)
            target_coords = self._calculate_center_target_coordinates()
            if target_coords:
                lat, lon, distance = target_coords
                
                # Target label with GPS coordinates
                coord_text1 = f"TARGET: {lat:.6f}, {lon:.6f}"
                coord_text2 = f"DIST: {distance:.0f}m"
                
                # Background for better readability
                text_size1 = cv2.getTextSize(coord_text1, font, 0.4, 1)[0]
                text_size2 = cv2.getTextSize(coord_text2, font, 0.4, 1)[0]
                max_width = max(text_size1[0], text_size2[0])
                
                cv2.rectangle(overlay_frame, 
                             (center_x - max_width//2 - 5, center_y + box_size + 10), 
                             (center_x + max_width//2 + 5, center_y + box_size + 45), 
                             (0, 20, 0), -1)
                cv2.rectangle(overlay_frame, 
                             (center_x - max_width//2 - 5, center_y + box_size + 10), 
                             (center_x + max_width//2 + 5, center_y + box_size + 45), 
                             GREEN, 1)
                
                cv2.putText(overlay_frame, coord_text1, (center_x - text_size1[0]//2, center_y + box_size + 25), 
                           font, 0.4, BRIGHT_GREEN, 1)
                cv2.putText(overlay_frame, coord_text2, (center_x - text_size2[0]//2, center_y + box_size + 40), 
                           font, 0.4, GREEN, 1)
            else:
                # No target calculation available
                cv2.putText(overlay_frame, "TARGET: NO DATA", (center_x - 45, center_y + box_size + 25), 
                           font, 0.4, ORANGE, 1)
            
            # Bottom status bar
            status_bg_height = 40
            cv2.rectangle(overlay_frame, (0, h - status_bg_height), (w, h), (0, 20, 0), -1)
            cv2.rectangle(overlay_frame, (0, h - status_bg_height), (w, h), GREEN, 1)
            
            # System time and frame info
            current_time = time.strftime("%H:%M:%S")
            time_text = f">>> SYSTEM TIME: {current_time}  |  CAMERA: {w}x{h}  |  SIYI ZR10 FEED"
            cv2.putText(overlay_frame, time_text, (10, h - 10), font, 0.5, BRIGHT_GREEN, 1)
            
            return overlay_frame
            
        except Exception as e:
            print(f"[CAMERA] Overlay error: {e}")
            return frame
    
    def _calculate_center_target_coordinates(self):
        """
        Calculate GPS coordinates where the center of the camera is pointing
        using the same math as the existing target calculation system
        """
        try:
            # Check if TargetCalculator is available
            if TargetCalculator is None:
                return None
            
            # Get aircraft state
            if not self.aircraft_state or self.aircraft_state.get('lat') is None:
                return None
                
            aircraft_lat = self.aircraft_state['lat']
            aircraft_lon = self.aircraft_state['lon']
            aircraft_alt_agl = self.aircraft_state.get('alt_agl', 0)
            
            # Get gimbal angles
            if not self.gimbal or not self.gimbal.is_connected:
                return None
                
            gimbal_pitch = self.gimbal.pitch_norm or 0  # Normalized pitch
            gimbal_yaw = self.gimbal.yaw_abs or 0      # Absolute yaw
            
            # Use TargetCalculator to find where camera center is pointing
            target_result = TargetCalculator.calculate_target(
                aircraft_lat=aircraft_lat,
                aircraft_lon=aircraft_lon,
                aircraft_alt_agl=aircraft_alt_agl,
                pitch_deg=gimbal_pitch,
                yaw_deg=gimbal_yaw
            )
            
            if target_result:
                return (
                    target_result['lat'],
                    target_result['lon'],
                    target_result['distance']
                )
            else:
                return None
                
        except Exception as e:
            print(f"[CAMERA] Target coordinate calculation error: {e}")
            return None
    
    def _cv_to_qimage(self, cv_img: np.ndarray) -> Optional[QImage]:
        """Convert OpenCV image to QImage"""
        try:
            # Convert BGR to RGB
            rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            
            # Create QImage
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            return qt_image
            
        except Exception as e:
            print(f"[CAMERA] Image conversion error: {e}")
            return None
    
    def is_connected(self) -> bool:
        """Check if camera is connected"""
        return self._connected
    
    def get_stream_info(self) -> dict:
        """Get current stream information"""
        info = {
            'rtsp_url': self.rtsp_url,
            'connected': self._connected,
            'running': self._running
        }
        
        if self.cap and self._connected:
            info.update({
                'width': int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                'height': int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                'fps': self.cap.get(cv2.CAP_PROP_FPS)
            })
        
        return info
    
    def update_rtsp_url(self, gimbal_ip: str, stream_port: int = 8554):
        """Update RTSP URL (requires restart)"""
        was_running = self._running
        
        if was_running:
            self.stop_stream()
        
        self.gimbal_ip = gimbal_ip
        self.stream_port = stream_port
        self.rtsp_url = f"rtsp://{gimbal_ip}:{stream_port}/main.264"
        
        print(f"[CAMERA] Updated RTSP URL to: {self.rtsp_url}")
        
        if was_running:
            self.start_stream()


# CameraStreamTkinter class removed - using PySide6 only