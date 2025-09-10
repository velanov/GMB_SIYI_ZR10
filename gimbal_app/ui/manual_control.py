from ..shared import *

class ManualControlWindow:
    """Manual gimbal control in separate window"""
    
    def __init__(self, parent, gimbal):
        self.gimbal = gimbal
        
        self.window = tk.Toplevel(parent)
        self.window.title("Manual Gimbal Control")
        self.window.geometry("300x400")
        self.window.transient(parent)
        
        parent_x = parent.winfo_x()
        parent_width = parent.winfo_width()
        self.window.geometry(f"300x400+{parent_x + parent_width + 10}+100")
        
        self._create_controls()
    
    def _create_controls(self):
        ttk.Label(self.window, text="Movement Speed").pack(pady=10)
        self.speed_var = tk.IntVar(value=50)
        speed_scale = ttk.Scale(self.window, from_=10, to=100, variable=self.speed_var, 
                               orient="horizontal", length=250)
        speed_scale.pack(pady=5)
        
        speed_label = ttk.Label(self.window, text="50")
        speed_label.pack()
        speed_scale.config(command=lambda v: speed_label.config(text=f"{int(float(v))}"))
        
        yaw_frame = ttk.LabelFrame(self.window, text="Yaw Control", padding=10)
        yaw_frame.pack(pady=15, padx=10, fill="x")
        yaw_btn_frame = ttk.Frame(yaw_frame); yaw_btn_frame.pack()
        self._create_control_button(yaw_btn_frame, "◄ Left", 
                                   lambda: (-self.speed_var.get(), 0)).pack(side="left", padx=10)
        self._create_control_button(yaw_btn_frame, "Right ►", 
                                   lambda: (self.speed_var.get(), 0)).pack(side="left", padx=10)
        
        pitch_frame = ttk.LabelFrame(self.window, text="Pitch Control", padding=10)
        pitch_frame.pack(pady=15, padx=10, fill="x")
        pitch_btn_frame = ttk.Frame(pitch_frame); pitch_btn_frame.pack()
        self._create_control_button(pitch_btn_frame, "▲ Up", 
                                   lambda: (0, self.speed_var.get())).pack(side="top", pady=5)
        self._create_control_button(pitch_btn_frame, "▼ Down", 
                                   lambda: (0, -self.speed_var.get())).pack(side="top", pady=5)
        
        ttk.Separator(self.window, orient="horizontal").pack(fill="x", pady=15)
        ttk.Button(self.window, text="Center Gimbal", 
                  command=self.gimbal.center).pack(pady=10)
        
        self.status_label = ttk.Label(self.window, text="Manual control ready")
        self.status_label.pack(pady=10)
    
    def _create_control_button(self, parent, text, speed_func):
        btn = ttk.Button(parent, text=text, width=12)
        moving = {"active": False}
        
        def start_movement():
            moving["active"] = True
            self.status_label.config(text=f"Moving: {text}")
            def move():
                if moving["active"]:
                    yaw_speed, pitch_speed = speed_func()
                    self.gimbal.jog(yaw_speed, pitch_speed)
                    self.window.after(50, move)
            move()
        
        def stop_movement():
            moving["active"] = False
            self.gimbal.jog(0, 0)
            self.status_label.config(text="Manual control ready")
        
        btn.bind("<Button-1>", lambda e: start_movement())
        btn.bind("<ButtonRelease-1>", lambda e: stop_movement())
        return btn
