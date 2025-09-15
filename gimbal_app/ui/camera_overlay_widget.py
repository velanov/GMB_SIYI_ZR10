"""
Qt overlay widget for camera display - recreates original OpenCV overlay style
"""

from PySide6.QtWidgets import QWidget, QLabel
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QPainter, QPen, QBrush, QColor, QFont, QFontMetrics
import time

class CameraOverlayWidget(QWidget):
    """
    Qt overlay widget that recreates the original OpenCV camera overlay
    Positioned on top of GStreamer video widget
    """
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Make widget transparent except for drawn elements
        self.setAttribute(Qt.WA_TransparentForMouseEvents, True)
        self.setAttribute(Qt.WA_TranslucentBackground, True)
        self.setAttribute(Qt.WA_NoSystemBackground, True)
        
        # Set transparent background
        self.setStyleSheet("background-color: transparent;")
        
        # Data references (set by main app)
        self.aircraft_state = None
        self.gimbal = None
        self.gimbal_locker = None
        
        # Colors (RGB format for Qt)
        self.GREEN = QColor(0, 255, 0)
        self.BRIGHT_GREEN = QColor(0, 255, 50)
        self.ORANGE = QColor(255, 165, 0)
        self.RED = QColor(255, 0, 0)
        self.DARK_GREEN = QColor(0, 80, 0)
        self.BLACK_TRANSPARENT = QColor(0, 20, 0, 200)  # Semi-transparent black
        
        # Font settings
        self.font = QFont("Monospace", 10, QFont.Bold)
        self.small_font = QFont("Monospace", 8, QFont.Bold)
        
        # Update timer - force repaint every 100ms
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._force_update)
        self.update_timer.start(100)  # Update every 100ms
        
    def set_aircraft_state(self, aircraft_state):
        """Set aircraft state reference"""
        self.aircraft_state = aircraft_state
        
    def set_gimbal(self, gimbal):
        """Set gimbal reference"""
        self.gimbal = gimbal
        
    def set_gimbal_locker(self, gimbal_locker):
        """Set gimbal locker reference"""
        self.gimbal_locker = gimbal_locker
    
    def _force_update(self):
        """Force widget repaint"""
        # Debug: Print data availability every 30 updates (3 seconds)
        if not hasattr(self, '_debug_count'):
            self._debug_count = 0
        self._debug_count += 1
        
        if self._debug_count % 30 == 0:
            print(f"[OVERLAY DEBUG] Aircraft: {self.aircraft_state is not None}")
            print(f"[OVERLAY DEBUG] Gimbal: {self.gimbal is not None}")
            if self.gimbal:
                print(f"[OVERLAY DEBUG] Gimbal connected: {getattr(self.gimbal, 'is_connected', False)}")
        
        self.update()  # Triggers paintEvent
    
    def paintEvent(self, event):
        """Paint the overlay elements"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Get widget dimensions
        w = self.width()
        h = self.height()
        
        if w <= 0 or h <= 0:
            return
            
        try:
            self._draw_top_left_status(painter, w, h)
            self._draw_top_right_target_info(painter, w, h)
            self._draw_center_crosshairs(painter, w, h)
            self._draw_bottom_status_bar(painter, w, h)
        except Exception as e:
            print(f"[OVERLAY] Paint error: {e}")
    
    def _draw_top_left_status(self, painter, w, h):
        """Draw top-left status information"""
        painter.setFont(self.font)
        y_pos = 25
        line_spacing = 20
        
        # Connection status
        if self.gimbal and self.gimbal.is_connected:
            age = time.time() - self.gimbal.last_update if hasattr(self.gimbal, 'last_update') and self.gimbal.last_update else 999
            if age < 1.0:
                status_text = "● GIMBAL ONLINE"
                color = self.GREEN
            else:
                status_text = f"● GIMBAL TIMEOUT ({age:.1f}s)"
                color = self.ORANGE
        else:
            status_text = "● GIMBAL OFFLINE"
            color = self.RED
            
        painter.setPen(QPen(color))
        painter.drawText(10, y_pos, status_text)
        y_pos += line_spacing
        
        # Gimbal angles
        if self.gimbal and self.gimbal.is_connected and hasattr(self.gimbal, 'yaw_abs') and self.gimbal.yaw_abs is not None:
            yaw = self.gimbal.yaw_abs or 0
            pitch = self.gimbal.pitch_norm or 0
            angles_text = f"ORIENTATION: Y:{yaw:06.1f}° P:{pitch:+05.1f}°"
            painter.setPen(QPen(self.BRIGHT_GREEN))
        else:
            angles_text = "ORIENTATION: Y:---.-° P:---.°"
            painter.setPen(QPen(self.DARK_GREEN))
            
        painter.drawText(10, y_pos, angles_text)
        y_pos += line_spacing
        
        # Aircraft position
        if self.aircraft_state and self.aircraft_state.get('lat') is not None:
            lat = self.aircraft_state['lat']
            lon = self.aircraft_state['lon']
            alt = self.aircraft_state.get('alt_agl', 0)
            hdg = self.aircraft_state.get('heading', 0)
            
            painter.setPen(QPen(self.GREEN))
            painter.drawText(10, y_pos, f"AIRCRAFT: {lat:.6f}, {lon:.6f}")
            y_pos += line_spacing
            
            painter.drawText(10, y_pos, f"ALT: {alt:.1f}m  HDG: {hdg:.1f}°")
        else:
            painter.setPen(QPen(self.ORANGE))
            painter.drawText(10, y_pos, "AIRCRAFT: NO MAVLINK DATA")
            y_pos += line_spacing
            
            painter.setPen(QPen(self.DARK_GREEN))
            painter.drawText(10, y_pos, "ALT: ---m  HDG: ---°")
    
    def _draw_top_right_target_info(self, painter, w, h):
        """Draw top-right target information"""
        painter.setFont(self.font)
        fm = QFontMetrics(self.font)
        y_pos = 25
        line_spacing = 20
        
        # Target lock status
        if self.gimbal and hasattr(self.gimbal, 'gimbal_lock_enabled'):
            lock_text = "LOCK: ENABLED" if self.gimbal.gimbal_lock_enabled else "LOCK: DISABLED"
            color = self.GREEN if self.gimbal.gimbal_lock_enabled else self.ORANGE
        else:
            lock_text = "LOCK: UNKNOWN"
            color = self.DARK_GREEN
            
        text_width = fm.horizontalAdvance(lock_text)
        painter.setPen(QPen(color))
        painter.drawText(w - text_width - 10, y_pos, lock_text)
        y_pos += line_spacing
        
        # Target coordinates and distance
        if self.gimbal_locker:
            try:
                lock_info = self.gimbal_locker.get_lock_info()
                
                if lock_info.get('active'):
                    target_lat = lock_info.get('target_lat', 0)
                    target_lon = lock_info.get('target_lon', 0)
                    target_text = f"TARGET: {target_lat:.6f}, {target_lon:.6f}"
                    
                    text_width = fm.horizontalAdvance(target_text)
                    painter.setPen(QPen(self.GREEN))
                    painter.drawText(w - text_width - 10, y_pos, target_text)
                    y_pos += line_spacing
                    
                    # Distance to target
                    if lock_info.get('required_angles') and lock_info['required_angles'].get('distance_2d'):
                        distance = lock_info['required_angles']['distance_2d']
                        if distance < 100:
                            dist_text = f"DIST: {distance:03.0f}m"
                            color = self.ORANGE
                        elif distance < 1000:
                            dist_text = f"DIST: {distance:04.0f}m"
                            color = self.GREEN
                        else:
                            dist_text = f"DIST: {distance/1000:.1f}km"
                            color = self.GREEN
                    else:
                        dist_text = "DIST: CALCULATING"
                        color = self.ORANGE
                        
                    text_width = fm.horizontalAdvance(dist_text)
                    painter.setPen(QPen(color))
                    painter.drawText(w - text_width - 10, y_pos, dist_text)
                else:
                    no_target_text = "NO TARGET SET"
                    text_width = fm.horizontalAdvance(no_target_text)
                    painter.setPen(QPen(self.ORANGE))
                    painter.drawText(w - text_width - 10, y_pos, no_target_text)
                    y_pos += line_spacing
                    
                    dist_text = "DIST: ---"
                    text_width = fm.horizontalAdvance(dist_text)
                    painter.setPen(QPen(self.DARK_GREEN))
                    painter.drawText(w - text_width - 10, y_pos, dist_text)
            except Exception:
                no_target_text = "NO TARGET SET"
                text_width = fm.horizontalAdvance(no_target_text)
                painter.setPen(QPen(self.ORANGE))
                painter.drawText(w - text_width - 10, y_pos, no_target_text)
        else:
            no_locker_text = "NO GIMBAL LOCKER"
            text_width = fm.horizontalAdvance(no_locker_text)
            painter.setPen(QPen(self.RED))
            painter.drawText(w - text_width - 10, y_pos, no_locker_text)
    
    def _draw_center_crosshairs(self, painter, w, h):
        """Draw center crosshairs and target information"""
        center_x, center_y = w // 2, h // 2
        crosshair_size = 20
        box_size = 60
        
        # Main crosshairs
        painter.setPen(QPen(self.GREEN, 2))
        painter.drawLine(center_x - crosshair_size, center_y, center_x + crosshair_size, center_y)
        painter.drawLine(center_x, center_y - crosshair_size, center_x, center_y + crosshair_size)
        
        # Center dot
        painter.setBrush(QBrush(self.GREEN))
        painter.drawEllipse(center_x - 3, center_y - 3, 6, 6)
        
        # Target box around center
        painter.setBrush(QBrush(Qt.transparent))
        painter.setPen(QPen(self.GREEN, 2))
        painter.drawRect(center_x - box_size, center_y - box_size, box_size * 2, box_size * 2)
        
        # Corner markers
        corner_size = 10
        pen = QPen(self.GREEN, 2)
        painter.setPen(pen)
        
        # Top-left corner
        painter.drawLine(center_x - box_size, center_y - box_size, 
                        center_x - box_size + corner_size, center_y - box_size)
        painter.drawLine(center_x - box_size, center_y - box_size, 
                        center_x - box_size, center_y - box_size + corner_size)
        
        # Top-right corner
        painter.drawLine(center_x + box_size, center_y - box_size, 
                        center_x + box_size - corner_size, center_y - box_size)
        painter.drawLine(center_x + box_size, center_y - box_size, 
                        center_x + box_size, center_y - box_size + corner_size)
        
        # Bottom-left corner
        painter.drawLine(center_x - box_size, center_y + box_size, 
                        center_x - box_size + corner_size, center_y + box_size)
        painter.drawLine(center_x - box_size, center_y + box_size, 
                        center_x - box_size, center_y + box_size - corner_size)
        
        # Bottom-right corner
        painter.drawLine(center_x + box_size, center_y + box_size, 
                        center_x + box_size - corner_size, center_y + box_size)
        painter.drawLine(center_x + box_size, center_y + box_size, 
                        center_x + box_size, center_y + box_size - corner_size)
        
        # Target coordinates calculation (where camera center is pointing)
        # This would need the same target calculator logic from the original
        # For now, show placeholder
        painter.setFont(self.small_font)
        painter.setPen(QPen(self.GREEN))
        
        # Background for target info
        info_text1 = "TARGET: CALCULATING..."
        info_text2 = "DIST: ---m"
        
        fm = QFontMetrics(self.small_font)
        text_width1 = fm.horizontalAdvance(info_text1)
        text_width2 = fm.horizontalAdvance(info_text2)
        max_width = max(text_width1, text_width2)
        
        # Background rectangle
        painter.setBrush(QBrush(self.BLACK_TRANSPARENT))
        painter.setPen(QPen(self.GREEN, 1))
        painter.drawRect(center_x - max_width // 2 - 5, center_y + box_size + 10, 
                        max_width + 10, 30)
        
        # Text
        painter.setBrush(QBrush(Qt.transparent))
        painter.setPen(QPen(self.BRIGHT_GREEN))
        painter.drawText(center_x - text_width1 // 2, center_y + box_size + 25, info_text1)
        painter.setPen(QPen(self.GREEN))
        painter.drawText(center_x - text_width2 // 2, center_y + box_size + 40, info_text2)
    
    def _draw_bottom_status_bar(self, painter, w, h):
        """Draw bottom status bar"""
        status_height = 35
        
        # Background
        painter.setBrush(QBrush(self.BLACK_TRANSPARENT))
        painter.setPen(QPen(self.GREEN, 1))
        painter.drawRect(0, h - status_height, w, status_height)
        
        # System time and camera info
        painter.setFont(self.font)
        painter.setBrush(QBrush(Qt.transparent))
        painter.setPen(QPen(self.BRIGHT_GREEN))
        
        current_time = time.strftime("%H:%M:%S")
        status_text = f">>> SYSTEM TIME: {current_time}  |  CAMERA: {w}x{h}  |  SIYI ZR10 FEED"
        painter.drawText(10, h - 10, status_text)