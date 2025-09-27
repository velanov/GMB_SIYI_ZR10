from ..shared import *
import pygame
from typing import Optional, Tuple

class JoystickController:
    """Logitech 3D Pro controller handler for gimbal control"""
    
    def __init__(self):
        self.joystick = None
        self.enabled = False
        self.last_yaw_input = 0
        self.last_pitch_input = 0
        self.last_zoom_input = 0
        
        # Configuration from Config class
        self.yaw_axis = getattr(Config, 'JOYSTICK_YAW_AXIS', 0)
        self.pitch_axis = getattr(Config, 'JOYSTICK_PITCH_AXIS', 1) 
        self.zoom_axis = getattr(Config, 'JOYSTICK_ZOOM_AXIS', 3)
        self.dead_zone = getattr(Config, 'JOYSTICK_DEAD_ZONE', 0.1)
        self.sensitivity = getattr(Config, 'JOYSTICK_SENSITIVITY', 100)
        
        self.initialize()
    
    def initialize(self) -> bool:
        """Initialize pygame and detect joystick"""
        try:
            pygame.init()
            pygame.joystick.init()
            
            if pygame.joystick.get_count() == 0:
                print("[CONTROLLER] No joysticks detected")
                return False
            
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            
            controller_name = self.joystick.get_name()
            print(f"[CONTROLLER] Initialized: {controller_name}")
            print(f"[CONTROLLER] Axes: {self.joystick.get_numaxes()}, Buttons: {self.joystick.get_numbuttons()}")
            
            # Verify we have the expected axes
            if self.joystick.get_numaxes() < 4:
                print(f"[CONTROLLER] Warning: Expected 4+ axes, found {self.joystick.get_numaxes()}")
            
            self.enabled = True
            return True
            
        except Exception as e:
            print(f"[CONTROLLER] Initialization failed: {e}")
            return False
    
    def apply_dead_zone(self, value: float) -> float:
        """Apply dead zone to axis value"""
        if abs(value) < self.dead_zone:
            return 0.0
        
        # Scale the remaining range
        if value > 0:
            return (value - self.dead_zone) / (1.0 - self.dead_zone)
        else:
            return (value + self.dead_zone) / (1.0 - self.dead_zone)
    
    def get_axis_safe(self, axis_index: int) -> float:
        """Safely get axis value with bounds checking"""
        if not self.joystick or axis_index >= self.joystick.get_numaxes():
            return 0.0
        return self.joystick.get_axis(axis_index)
    
    def update(self) -> Tuple[int, int, int]:
        """
        Update controller state and return (yaw_speed, pitch_speed, zoom_command)
        Returns (0, 0, 0) if controller disabled or no significant input
        """
        if not self.enabled or not self.joystick:
            return 0, 0, 0
        
        try:
            pygame.event.pump()  # Process pygame events
            
            # Read raw axis values
            raw_yaw = self.get_axis_safe(self.yaw_axis)
            raw_pitch = self.get_axis_safe(self.pitch_axis)
            raw_zoom = self.get_axis_safe(self.zoom_axis)
            
            # Apply dead zones
            yaw_input = self.apply_dead_zone(raw_yaw)
            pitch_input = self.apply_dead_zone(raw_pitch)  # Normal Y axis - will invert in manager
            zoom_input = self.apply_dead_zone(-raw_zoom)    # Invert throttle: up=positive=zoom in
            
            # Convert to gimbal speeds (-100 to +100)
            yaw_speed = int(yaw_input * self.sensitivity)
            pitch_speed = int(pitch_input * self.sensitivity)
            
            # Zoom command: -1=zoom out, 0=hold, 1=zoom in
            zoom_command = 0
            if abs(zoom_input) > 0.2:  # Threshold for zoom activation
                zoom_command = 1 if zoom_input > 0 else -1
            
            # Store for logging/debugging
            self.last_yaw_input = yaw_speed
            self.last_pitch_input = pitch_speed
            self.last_zoom_input = zoom_command
            
            return yaw_speed, pitch_speed, zoom_command
            
        except Exception as e:
            print(f"[CONTROLLER] Update error: {e}")
            return 0, 0, 0
    
    def has_input(self) -> bool:
        """Check if controller has any significant input"""
        return (abs(self.last_yaw_input) > 5 or 
                abs(self.last_pitch_input) > 5 or 
                abs(self.last_zoom_input) > 0)
    
    def get_status(self) -> str:
        """Get controller status string for UI display"""
        if not self.enabled:
            return "Controller: Disabled"
        if not self.joystick:
            return "Controller: Not Found"
        
        controller_name = self.joystick.get_name()
        if self.has_input():
            return f"Controller: {controller_name} (Active)"
        else:
            return f"Controller: {controller_name} (Ready)"
    
    def cleanup(self):
        """Clean up pygame resources"""
        if self.joystick:
            self.joystick.quit()
        pygame.joystick.quit()
        pygame.quit()
        self.enabled = False
        print("[CONTROLLER] Cleaned up")