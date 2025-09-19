from ..shared import *

class SiyiGimbal:
    """SIYI ZR10 gimbal communication handler"""
    
    def __init__(self, ip: str = Config.SIYI_IP, port: int = Config.SIYI_PORT):
        self.ip, self.port = ip, port
        self.sock = None
        self.seq = 1
        self._rx_alive = False
        self._rx_thread = None
        
        # State
        self.yaw_abs = None
        self.pitch_norm = None
        self.roll = None
        self.last_update = 0
        
        # Logger
        self.logger = GimbalLogger()
        self.mount_dir = None
        self.motion_mode = None

        # Stream keepalive
        self._last_enable = 0.0
        self._stream_hz = 10
    
    def _create_frame(self, cmd: int, payload: bytes = b"") -> bytes:
        stx = b"\x55\x66"
        ctrl = b"\x00"
        dlen = struct.pack("<H", len(payload))
        seq = struct.pack("<H", self.seq & 0xFFFF); self.seq = (self.seq + 1) & 0xFFFF
        body = stx + ctrl + dlen + seq + bytes([cmd]) + payload
        return body + struct.pack("<H", crc16_ccitt(body))

    def _enable_stream(self, hz: Optional[int] = None):
        if not self.sock:
            return
        try:
            hz = hz or self._stream_hz
            freq_map = {0:0, 2:1, 4:2, 5:3, 10:4, 20:5, 50:6, 100:7}
            req = min(freq_map.keys(), key=lambda f: abs(f - hz))
            payload = bytes([1, freq_map[req]])  # data_type=1 (attitude)
            self.sock.sendto(self._create_frame(0x25, payload), (self.ip, self.port))
            self._last_enable = time.time()
        except Exception:
            pass

    def _probe_attitude(self):
        if not self.sock:
            return
        try:
            self.sock.sendto(self._create_frame(0x0D), (self.ip, self.port))
        except Exception:
            pass
    
    def start(self) -> bool:
        if self.sock:
            return True
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # Increased timeout from 0.2 to 1.0 seconds for better stability
            self.sock.settimeout(1.0)
            # Set socket buffer sizes to handle network issues
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
            self._rx_alive = True
            self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._rx_thread.start()
            self.request_config()
            self._enable_stream(self._stream_hz)
            print(f"[GIMBAL] Successfully connected to {self.ip}:{self.port}")
            return True
        except Exception as e:
            print(f"[GIMBAL] Failed to start connection: {e}")
            return False
    
    def stop(self):
        self._rx_alive = False
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
    
    def request_attitude(self):
        if self.sock:
            try:
                self.sock.sendto(self._create_frame(0x0D), (self.ip, self.port))
            except Exception:
                pass
    
    def request_config(self):
        if self.sock:
            try:
                self.sock.sendto(self._create_frame(0x0A), (self.ip, self.port))
            except Exception:
                pass
    
    def jog(self, yaw_speed: int, pitch_speed: int):
        if self.sock:
            try:
                y = clamp(yaw_speed, -100, 100)
                p = clamp(pitch_speed, -100, 100)
                payload = struct.pack("<bb", int(y), int(p))
                self.sock.sendto(self._create_frame(0x07, payload), (self.ip, self.port))
            except Exception:
                pass
    
    def center(self):
        if self.sock:
            try:
                self.sock.sendto(self._create_frame(0x08, b"\x01"), (self.ip, self.port))
            except Exception:
                pass
    
    def set_angle(self, yaw_deg: float, pitch_deg: float, speed: int = 50):
        """Set gimbal to specific angles using controlled jogging"""
        if not self.sock or not self.is_connected:
            print(f"[GIMBAL] Cannot set angle - not connected")
            return
        
        # CRITICAL: Clamp angles to safe limits BEFORE any processing
        pitch_deg = max(min(pitch_deg, 89.0), -89.0)  # Prevent ±90° exactly
        yaw_deg = yaw_deg % 360.0  # Normalize yaw
        
        # UPSIDE-DOWN MOUNTING: Invert pitch for upside-down gimbal
        pitch_deg = -pitch_deg  # Flip pitch direction for upside-down mount
            
        try:
            current_yaw = self.yaw_abs if self.yaw_abs is not None else 0
            current_pitch = self.pitch_norm if self.pitch_norm is not None else 0
            
            # Calculate required movement
            yaw_diff = yaw_deg - current_yaw
            pitch_diff = pitch_deg - current_pitch
            
            # CRITICAL FIX: Yaw wraparound - always choose shortest path
            # Normalize to [-180, +180] range for shortest rotation
            while yaw_diff > 180: 
                yaw_diff -= 360
            while yaw_diff < -180: 
                yaw_diff += 360
                
            # Verify we have the shortest path (should be within [-180, +180])
            if abs(yaw_diff) > 180:
                print(f"[GIMBAL] WARNING: Yaw diff {yaw_diff:.1f}° > 180° after normalization!")
                yaw_diff = yaw_diff - 360 if yaw_diff > 0 else yaw_diff + 360
            
            print(f"[GIMBAL] Target: Y={yaw_deg:.1f}° P={pitch_deg:.1f}°")
            print(f"[GIMBAL] Current: Y={current_yaw:.1f}° P={current_pitch:.1f}°") 
            print(f"[GIMBAL] Difference: dY={yaw_diff:.1f}° dP={pitch_diff:.1f}°")
            
            # DEBUG: Log direction logic
            yaw_direction = "RIGHT" if yaw_diff > 0 else "LEFT" if yaw_diff < 0 else "NONE"
            pitch_direction = "UP" if pitch_diff > 0 else "DOWN" if pitch_diff < 0 else "NONE"
            print(f"[GIMBAL] Direction needed: Yaw {yaw_direction}, Pitch {pitch_direction}")
            
            # Check if gimbal is stuck at limits and attempt recovery
            if abs(current_pitch) >= 90.0:
                print(f"[GIMBAL] Warning: Gimbal at pitch limit ({current_pitch:.1f}°), target clamped to {pitch_deg:.1f}°")
                # Attempt automatic recovery with stronger commands
                if abs(pitch_diff) > 50.0:  # Large pitch difference suggests it's stuck
                    print(f"[GIMBAL] Attempting automatic recovery - large pitch difference: {pitch_diff:.1f}°")
                    self.logger.log_recovery_attempt("Large pitch difference", current_pitch, pitch_diff)
                    self.force_pitch_recovery()
                    return
                
            # Yaw movement issue detection and recovery
            if abs(yaw_diff) > 150.0:
                print(f"[GIMBAL] Large yaw difference detected ({yaw_diff:.1f}°), using stepped approach")
                # For very large yaw differences, use reduced speed to avoid overshooting
                step_yaw_speed = 60 if yaw_diff > 0 else -60  # Reduced from 100 to 60
                payload = struct.pack("<bb", step_yaw_speed, 0)  # Moderate yaw speed, no pitch
                self.sock.sendto(self._create_frame(0x07, payload), (self.ip, self.port))
                return
            
            # Only move if difference is significant
            if abs(yaw_diff) < 1.0 and abs(pitch_diff) < 1.0:
                print(f"[GIMBAL] Already close to target, no movement needed")
                return
                
            # Use proportional control for smooth movement
            # Convert angle difference to movement speed - REDUCED MAX SPEED to prevent oscillation
            max_diff = 90.0  # Maximum difference for full speed
            
            yaw_speed = clamp((yaw_diff / max_diff) * 80, -80, 80)  # Increased from 40 to 80 for faster locking
            # PITCH DIRECTION FIX: Invert the calculation
            # When pitch_diff > 0 (target higher than current), we need NEGATIVE speed (up)
            # When pitch_diff < 0 (target lower than current), we need POSITIVE speed (down)
            # Protocol: negative speed = up, positive speed = down
            pitch_speed_raw = -clamp((pitch_diff / max_diff) * 80, -80, 80)  # Increased from 40 to 80 for faster locking
            
            # OVERSHOOT PROTECTION: Decelerate when approaching target
            # Reduce speed when within 10° of target to prevent overshooting (reduced from 20°)
            yaw_decel_factor = 1.0
            pitch_decel_factor = 1.0
            
            if abs(yaw_diff) < 3.0:  # Within 3° of yaw target (tighter deceleration zone)
                yaw_decel_factor = max(0.6, abs(yaw_diff) / 3.0)  # Scale from 60% to 100% (higher minimum speed)
                
            if abs(pitch_diff) < 3.0:  # Within 3° of pitch target (tighter deceleration zone) 
                pitch_decel_factor = max(0.6, abs(pitch_diff) / 3.0)  # Scale from 60% to 100% (higher minimum speed)
                
            # Apply deceleration
            yaw_speed = int(yaw_speed * yaw_decel_factor)
            pitch_speed = int(pitch_speed_raw * pitch_decel_factor)
            
            # MINIMUM SPEED: Prevent hunting with very small speeds
            min_speed = 8  # Minimum speed to ensure movement
            if abs(yaw_speed) > 0 and abs(yaw_speed) < min_speed:
                yaw_speed = min_speed if yaw_speed > 0 else -min_speed
            if abs(pitch_speed) > 0 and abs(pitch_speed) < min_speed:
                pitch_speed = min_speed if pitch_speed > 0 else -min_speed
            
            # LIMIT PROTECTION: Reduce speed when approaching ±85° limits
            if current_pitch <= -85.0 and pitch_speed > 0:  # Approaching -90° limit
                pitch_speed = min(pitch_speed, 20)  # Max speed 20 near bottom limit
                print(f"[GIMBAL] Speed limited near -90° limit: pitch_speed={pitch_speed}")
                
            if current_pitch >= 85.0 and pitch_speed < 0:  # Approaching +90° limit
                pitch_speed = max(pitch_speed, -20)  # Max speed 20 near top limit
                print(f"[GIMBAL] Speed limited near +90° limit: pitch_speed={pitch_speed}")
            
            # Apply speed scaling
            speed_factor = clamp(speed / 100.0, 0.1, 1.0)
            yaw_speed = int(yaw_speed * speed_factor)
            pitch_speed = int(pitch_speed * speed_factor)
            
            # MINIMUM SPEED FIX: Ensure non-zero speeds for significant differences
            if abs(yaw_diff) > 1.0 and abs(yaw_speed) < 5:
                yaw_speed = 5 if yaw_diff > 0 else -5  # Minimum speed increased to 5
            if abs(pitch_diff) > 1.0 and abs(pitch_speed) < 5:
                pitch_speed = 5 if pitch_speed > 0 else -5  # Minimum speed increased to 5
            
            print(f"[GIMBAL] Sending jog command: yaw_speed={yaw_speed}, pitch_speed={pitch_speed}")
            
            # Log the gimbal command
            self.logger.log_gimbal_command(yaw_deg, pitch_deg, current_yaw, current_pitch, 
                                         yaw_speed, pitch_speed)
            
            # Yaw movement diagnostics
            if abs(yaw_diff) > 10 and abs(yaw_speed) >= 30:
                print(f"[GIMBAL] Warning: Large yaw difference ({yaw_diff:.1f}°) with strong command ({yaw_speed})")
                self.logger.log_warning(f"Large yaw difference ({yaw_diff:.1f}°) with strong command ({yaw_speed})")
            
            # Send jog command  
            payload = struct.pack("<bb", yaw_speed, pitch_speed)
            self.sock.sendto(self._create_frame(0x07, payload), (self.ip, self.port))
            
        except Exception as e:
            print(f"[GIMBAL] Error in set_angle: {e}")
    
    def stop_movement(self):
        """Stop any gimbal movement"""
        if self.sock:
            try:
                payload = struct.pack("<bb", 0, 0)  # Stop all movement
                self.sock.sendto(self._create_frame(0x07, payload), (self.ip, self.port))
            except Exception:
                pass
    
    def center_gimbal(self):
        """Center the gimbal (manual recovery from stuck positions)"""
        if self.sock:
            try:
                print("[GIMBAL] Sending center/home command")
                # Send gimbal center command (if supported by firmware)
                self.sock.sendto(self._create_frame(0x05, b""), (self.ip, self.port))
            except Exception as e:
                print(f"[GIMBAL] Center command failed: {e}")
                
    def force_pitch_recovery(self):
        """Force strong upward pitch movement to break free from -90° limit"""
        if self.sock and self.pitch_norm and self.pitch_norm <= -89.5:
            try:
                print("[GIMBAL] Force recovery: sending maximum upward pitch command")
                self.logger.log_recovery_attempt("Force recovery start", self.pitch_norm, 0)
                
                # Send maximum upward pitch for 2 seconds  
                # Use negative pitch speed to go UP (protocol: negative = up, positive = down)
                for i in range(20):  # 20 x 0.1s = 2 seconds
                    payload = struct.pack("<bb", 0, -100)  # 0 yaw, NEGATIVE pitch for upward
                    self.sock.sendto(self._create_frame(0x07, payload), (self.ip, self.port))
                    if i % 5 == 0:  # Log every 0.5 seconds
                        self.logger.log_gimbal_command(0, -89, 0, self.pitch_norm or -90, 0, -100)
                    time.sleep(0.1)
                    
                # Stop movement
                self.stop_movement()
                print("[GIMBAL] Force recovery completed")
                self.logger.log_recovery_attempt("Force recovery completed", self.pitch_norm, 0)
            except Exception as e:
                print(f"[GIMBAL] Force recovery failed: {e}")
                self.logger.log_error("Force recovery failed", e)
    
    def zoom_in(self):
        """Start zooming in (continuous zoom)"""
        if self.sock:
            try:
                # Command 0x05 with payload 1 for zoom in
                payload = struct.pack("<b", 1)
                self.sock.sendto(self._create_frame(0x05, payload), (self.ip, self.port))
                print("[GIMBAL] Zoom in started")
            except Exception as e:
                print(f"[GIMBAL] Zoom in failed: {e}")
    
    def zoom_out(self):
        """Start zooming out (continuous zoom)"""
        if self.sock:
            try:
                # Command 0x05 with payload -1 for zoom out
                payload = struct.pack("<b", -1)
                self.sock.sendto(self._create_frame(0x05, payload), (self.ip, self.port))
                print("[GIMBAL] Zoom out started")
            except Exception as e:
                print(f"[GIMBAL] Zoom out failed: {e}")
    
    def zoom_hold(self):
        """Stop zooming (hold current zoom level)"""
        if self.sock:
            try:
                # Command 0x05 with payload 0 for zoom stop/hold
                payload = struct.pack("<b", 0)
                self.sock.sendto(self._create_frame(0x05, payload), (self.ip, self.port))
                print("[GIMBAL] Zoom hold/stop")
            except Exception as e:
                print(f"[GIMBAL] Zoom hold failed: {e}")
    
    def _parse_packet(self, packet: bytes):
        if len(packet) < 12 or packet[:2] != b"\x55\x66":
            return
        dlen = struct.unpack_from("<H", packet, 3)[0]
        cmd = packet[7]
        payload = packet[8:8+dlen]
        
        if cmd == 0x0D and len(payload) >= 12:
            yaw_i, pitch_i, roll_i = struct.unpack_from("<hhh", payload, 0)[:3]
            self.yaw_abs = (yaw_i / 10.0) % 360.0
            self.pitch_norm = pitch_i / 10.0 - 180.0
            while self.pitch_norm > 180: self.pitch_norm -= 360
            while self.pitch_norm < -180: self.pitch_norm += 360
            self.roll = roll_i / 10.0
            self.last_update = time.time()
        elif cmd == 0x0A and len(payload) >= 6:
            motion_mode = payload[4]
            mount_dir = payload[5]
            self.motion_mode = {0:"Follow", 1:"Lock", 2:"FPV"}.get(motion_mode, f"Unknown({motion_mode})")
            self.mount_dir = {1:"Normal", 2:"UpsideDown"}.get(mount_dir, f"Unknown({mount_dir})")
    
    def _rx_loop(self):
        consecutive_errors = 0
        max_consecutive_errors = 10
        
        while self._rx_alive:
            # More robust stream keepalive - enable more frequently
            if time.time() - self._last_enable > 1.5:  # Reduced from 2.0 to 1.5
                self._enable_stream(self._stream_hz)
            
            try:
                data, _ = self.sock.recvfrom(2048)
                self._parse_packet(data)
                consecutive_errors = 0  # Reset error counter on successful receive
                
            except socket.timeout:
                # Probe attitude more frequently on timeout
                self._probe_attitude()
                consecutive_errors += 1
                if consecutive_errors > max_consecutive_errors:
                    print(f"[GIMBAL] Too many consecutive timeouts ({consecutive_errors}), attempting reconnection")
                    self._attempt_reconnect()
                    consecutive_errors = 0
                continue
                
            except OSError as e:
                error_msg = str(e)
                print(f"[GIMBAL] Socket error in RX loop: {error_msg}")
                
                # Handle specific socket errors differently
                if "Bad file descriptor" in error_msg or "Socket operation on non-socket" in error_msg:
                    print(f"[GIMBAL] Socket closed unexpectedly - attempting restart")
                    break  # Socket is invalid, need to restart connection
                
                consecutive_errors += 1
                if consecutive_errors > 8:  # Increased tolerance from 3 to 8
                    print(f"[GIMBAL] Too many consecutive socket errors ({consecutive_errors}), restarting connection")
                    break
                    
                time.sleep(0.3)  # Increased pause from 0.1 to 0.3 seconds
                continue
                
            except Exception as e:
                print(f"[GIMBAL] Unexpected error in RX loop: {e}")
                consecutive_errors += 1
                if consecutive_errors > 5:
                    break
                time.sleep(0.1)
                continue
    
    def _attempt_reconnect(self):
        """Attempt to reconnect the gimbal connection with improved stability"""
        print("[GIMBAL] Attempting to reconnect...")
        try:
            # Safely close existing socket
            if self.sock:
                try:
                    self.sock.close()
                except Exception:
                    pass
                self.sock = None
            
            # Wait longer for network to stabilize
            time.sleep(1.0)  # Increased from 0.5 to 1.0 seconds
            
            # Create new socket with improved settings
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.settimeout(1.0)
            
            # Enhanced socket buffer sizes and options
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow address reuse
            
            # Test connection with a simple request first
            self.request_config()
            time.sleep(0.2)  # Allow config response
            
            # Enable stream after config is stable
            self._enable_stream(self._stream_hz)
            
            print("[GIMBAL] Reconnection successful")
        except Exception as e:
            print(f"[GIMBAL] Reconnection failed: {e}")
            # Ensure socket is cleaned up on failure
            if self.sock:
                try:
                    self.sock.close()
                except Exception:
                    pass
                self.sock = None

    @property
    def is_connected(self) -> bool:
        # More lenient connection check - allow up to 3 seconds without updates
        return (self.sock is not None and 
                self.yaw_abs is not None and 
                time.time() - self.last_update < 3.0)
    
    def get_corrected_angles(self, aircraft_heading: float) -> Tuple[Optional[float], Optional[float]]:
        if not self.is_connected:
            return None, None
        
        raw_pitch = self.pitch_norm
        raw_yaw = self.yaw_abs
        
        if self.mount_dir == "UpsideDown":
            corrected_pitch = -raw_pitch
            corrected_yaw_offset = (raw_yaw + 180.0) % 360.0
        elif self.mount_dir == "Normal":
            corrected_pitch = raw_pitch
            corrected_yaw_offset = raw_yaw
        else:
            corrected_pitch = raw_pitch
            corrected_yaw_offset = raw_yaw
        
        final_yaw = (corrected_yaw_offset + aircraft_heading) % 360.0
        return corrected_pitch, final_yaw
