from ..shared import *

class SettingsDialog:
    """Modal settings dialog to edit IPs/ports and MAVLink strings."""

    def __init__(self, parent, on_apply):
        self.parent = parent
        self.on_apply = on_apply
        self.win = tk.Toplevel(parent)
        self.win.title("Settings")
        self.win.resizable(False, False)
        self.win.transient(parent)
        self.win.grab_set()

        frm = ttk.Frame(self.win, padding=10)
        frm.pack(fill="both", expand=True)

        # Profile presets
        rowp = ttk.Frame(frm); rowp.pack(fill="x", pady=(0,8))
        ttk.Label(rowp, text="Profile:", width=16).pack(side="left")
        self.profile_var = tk.StringVar(value="Custom")
        cb = ttk.Combobox(rowp, textvariable=self.profile_var, values=["SITL/WSL", "Operation", "Custom"], state="readonly", width=20)
        cb.pack(side="left")
        cb.bind("<<ComboboxSelected>>", self._on_profile)

        # Fields
        self.siyi_ip = self._entry(frm, "Gimbal IP:", Config.SIYI_IP)
        self.siyi_port = self._entry(frm, "Gimbal Port:", str(Config.SIYI_PORT))
        # MAVLink RX/TX
        self.mavlink_addr = self._entry(frm, "MAVLink RX (listen):", Config.MAVLINK_ADDRESS)
        self.mavlink_tx_addr = self._entry(frm, "MAVLink TX (send):", Config.MAVLINK_TX_ADDRESS or "")
        self.sbs_bind = self._entry(frm, "SBS Bind Host:", Config.SBS_BIND)
        self.sbs_port = self._entry(frm, "SBS Port:", str(Config.SBS_PORT))

        # Buttons
        btns = ttk.Frame(frm); btns.pack(fill="x", pady=(12,0))
        ttk.Button(btns, text="Cancel", command=self.win.destroy).pack(side="right")
        ttk.Button(btns, text="Save & Apply", command=self._apply).pack(side="right", padx=6)

        hint = ttk.Label(frm, foreground="#555",
            text="Hint (QGC Forwarding):\n"
                 "- QGC → MAVLink → Forwarding: 127.0.0.1:14540\n"
                 "- RX: udpin:127.0.0.1:14540\n"
                 "- TX: udpout:127.0.0.1:14550 (QGC UDP input)")
        hint.pack(anchor="w", pady=(6,0))

    def _entry(self, parent, label, value):
        row = ttk.Frame(parent); row.pack(fill="x", pady=3)
        ttk.Label(row, text=label, width=18).pack(side="left")
        var = tk.StringVar(value=value)
        ent = ttk.Entry(row, textvariable=var, width=32); ent.pack(side="left")
        return var

    def _on_profile(self, _):
        p = self.profile_var.get()
        if p == "SITL/WSL":
            self.mavlink_addr.set("udpin:127.0.0.1:14540")
            self.mavlink_tx_addr.set("udpout:127.0.0.1:14550")
            self.sbs_bind.set("0.0.0.0")
            self.sbs_port.set("30003")
        elif p == "Operation":
            # Example listen/send addresses adjusted for operation network
            self.mavlink_addr.set("udpin:0.0.0.0:14540")
            self.mavlink_tx_addr.set("")  # Send via same link
            self.sbs_bind.set("0.0.0.0")
            self.sbs_port.set("30003")
        # Custom -> don't touch

    def _apply(self):
        try:
            new_cfg = {
                "SIYI_IP": self.siyi_ip.get().strip(),
                "SIYI_PORT": int(self.siyi_port.get().strip()),
                "MAVLINK_ADDRESS": self.mavlink_addr.get().strip(),
                "MAVLINK_TX_ADDRESS": self.mavlink_tx_addr.get().strip(),
                "SBS_BIND": self.sbs_bind.get().strip(),
                "SBS_PORT": int(self.sbs_port.get().strip()),
            }
        except Exception:
            messagebox.showerror("Error", "Please enter valid port/values.")
            return

        for k, v in new_cfg.items():
            setattr(Config, k, v)
        SettingsStore.save_from()

        try:
            self.on_apply(new_cfg)
        except Exception as e:
            messagebox.showerror("Error", f"Error applying settings: {e}")
            return

        self.win.destroy()
