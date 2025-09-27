from ..shared import *
from .joystick_controller import JoystickController

class ControllerManager:
    """Minimal controller manager that handles target selection state awareness"""
    
    def __init__(self, gimbal, get_target_state_callback):
        self.gimbal = gimbal
        self.get_target_state = get_target_state_callback  # Callback to check if target is selected
        self.controller = JoystickController()
        self.enabled = Config.JOYSTICK_ENABLED
        self.last_zoom_command = 0
        self.last_yaw_speed = 0
        self.last_pitch_speed = 0
        self.movement_threshold = 5  # Minimum speed to consider as movement
        
        if self.enabled and self.controller.enabled:
            print("[CONTROLLER_MGR] Controller manager initialized successfully")
        else:
            print("[CONTROLLER_MGR] Controller disabled or not found")
    
    def update(self):
        """Update controller input - called from main app timer"""
        if not self.enabled or not self.controller.enabled:
            return
        
        # Check if target is selected - block input if true
        target_state = self.get_target_state()
        if target_state and target_state.get('selected', False):
            return  # Block all controller input when target is locked
        
        # Get controller input
        yaw_speed_raw, pitch_speed_raw, zoom_command = self.controller.update()
        
        # Apply speeds directly (pygame Y-axis already inverted)
        yaw_speed = yaw_speed_raw
        pitch_speed = pitch_speed_raw  # Forward stick already gives negative value for UP
        
        # Only send jog commands when input changes or when stopping movement
        input_changed = (yaw_speed != self.last_yaw_speed or 
                        pitch_speed != self.last_pitch_speed)
        
        has_input = (abs(yaw_speed) >= self.movement_threshold or 
                    abs(pitch_speed) >= self.movement_threshold)
        
        had_input = (abs(self.last_yaw_speed) >= self.movement_threshold or 
                    abs(self.last_pitch_speed) >= self.movement_threshold)
        
        # Send command if:
        # 1. Input values changed, OR
        # 2. We need to stop movement (had input, now don't)
        if input_changed or (had_input and not has_input):
            self.gimbal.jog(yaw_speed, pitch_speed)
            if has_input:
                print(f"[CONTROLLER] Gimbal jog: yaw={yaw_speed}, pitch={pitch_speed}")
            elif had_input and not has_input:
                print(f"[CONTROLLER] Gimbal stop movement")
        
        # Store last values
        self.last_yaw_speed = yaw_speed
        self.last_pitch_speed = pitch_speed
        
        # Handle zoom control
        if zoom_command != self.last_zoom_command:
            if zoom_command == 1:
                self.gimbal.zoom_in()
            elif zoom_command == -1:
                self.gimbal.zoom_out()
            elif zoom_command == 0 and self.last_zoom_command != 0:
                self.gimbal.zoom_hold()
            
            self.last_zoom_command = zoom_command
    
    def get_status(self) -> str:
        """Get controller status for UI display"""
        if not self.enabled:
            return "Controller: Disabled"
        
        target_state = self.get_target_state()
        if target_state and target_state.get('selected', False):
            return "Controller: Blocked (Target Selected)"
        
        return self.controller.get_status()
    
    def cleanup(self):
        """Clean up controller resources"""
        if self.controller:
            self.controller.cleanup()