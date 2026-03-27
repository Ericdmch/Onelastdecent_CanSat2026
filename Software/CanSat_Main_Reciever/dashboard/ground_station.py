#!/usr/bin/env python3

#cd /Users/ericchen/Documents/GitHub/Onelastdecent_CanSat2026/Software/CanSat_Main_Reciever/dashboard
#pip install -r requirements.txt
#python3 ground_station.py

"""
CanSat 2026 Ground Station Dashboard
=====================================
Real-time telemetry display with graphing and CSV recording.

Packet format from receiver (USB serial):
  $CANSAT,<ms>,<temp>,<press>,<hum>,<gas>,<lat_e7>,<lon_e7>,<alt_mm>,<roll>,<pitch>,<yaw>
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import serial
import serial.tools.list_ports
import threading
import csv
import os
from datetime import datetime
from collections import deque

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

# ── Data config ──────────────────────────────────────────────────────────────

FIELDS = [
    "timestamp_ms", "temperature", "pressure", "humidity", "gas_resistance",
    "lat_e7", "lon_e7", "alt_mm", "roll", "pitch", "yaw",
]

GRAPH_GROUPS = [
    {
        "title": "Temperature (C)",
        "fields": ["temperature"],
        "colors": ["#e74c3c"],
        "ylabel": "C",
        "ylim": (-20, 60),
    },
    {
        "title": "Pressure (hPa)",
        "fields": ["pressure"],
        "colors": ["#3498db"],
        "ylabel": "hPa",
        "ylim": (950, 1050),
    },
    {
        "title": "Humidity (%)",
        "fields": ["humidity"],
        "colors": ["#2ecc71"],
        "ylabel": "%",
        "ylim": (0, 100),
    },
    {
        "title": "Altitude (m)",
        "fields": ["alt_mm"],
        "colors": ["#9b59b6"],
        "ylabel": "m",
        "scale": [("alt_mm", 0.001)],  # mm -> m
        "ylim": (-50, 1000),
    },
    {
        "title": "Orientation (deg)",
        "fields": ["roll", "pitch", "yaw"],
        "colors": ["#e67e22", "#1abc9c", "#f1c40f"],
        "ylabel": "deg",
        "ylim": (-180, 180),
    },
    {
        "title": "Gas Resistance (kOhm)",
        "fields": ["gas_resistance"],
        "colors": ["#e84393"],
        "ylabel": "kOhm",
        "scale": [("gas_resistance", 0.001)],  # ohm -> kOhm
        "ylim": (0, 500),
    },
]

TEMP_OFFSET_C = -6.41  # PCB self-heating correction

MAX_POINTS = 300  # rolling window for graphs
REFRESH_MS = 200  # graph refresh interval


# ── Packet parsing ───────────────────────────────────────────────────────────

def parse_packet(line: str) -> dict | None:
    """Parse a $CANSAT CSV line into a dict. Returns None on failure."""
    line = line.strip()
    if not line.startswith("$CANSAT,"):
        return None
    parts = line[len("$CANSAT,"):].split(",")
    if len(parts) != len(FIELDS):
        return None
    try:
        values = {}
        for i, name in enumerate(FIELDS):
            values[name] = float(parts[i])
        return values
    except ValueError:
        return None


# ── Main application ─────────────────────────────────────────────────────────

class GroundStationApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("CanSat 2026 Ground Station")
        self.root.geometry("1400x900")
        self.root.minsize(1000, 700)

        # State
        self.serial_port: serial.Serial | None = None
        self.serial_thread: threading.Thread | None = None
        self.running = False
        self.recording = False
        self.recorded_rows: list[dict] = []
        self.packet_count = 0
        self.last_packet: dict | None = None

        # Rolling data for graphs
        self.time_data = deque(maxlen=MAX_POINTS)
        self.series: dict[str, deque] = {f: deque(maxlen=MAX_POINTS) for f in FIELDS}

        self._build_ui()
        self._refresh_ports()

    # ── UI construction ──────────────────────────────────────────────────

    def _build_ui(self):
        # Top bar: connection + controls
        top = ttk.Frame(self.root, padding=5)
        top.pack(fill=tk.X)

        ttk.Label(top, text="Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=25, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=(2, 5))

        ttk.Button(top, text="Refresh", command=self._refresh_ports).pack(side=tk.LEFT, padx=2)

        ttk.Label(top, text="Baud:").pack(side=tk.LEFT, padx=(10, 0))
        self.baud_var = tk.StringVar(value="115200")
        ttk.Combobox(top, textvariable=self.baud_var, values=["9600", "115200"], width=8, state="readonly").pack(side=tk.LEFT, padx=2)

        self.connect_btn = ttk.Button(top, text="Connect", command=self._toggle_connection)
        self.connect_btn.pack(side=tk.LEFT, padx=10)

        # Separator
        ttk.Separator(top, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=10)

        self.record_btn = ttk.Button(top, text="Start Recording", command=self._toggle_recording, state=tk.DISABLED)
        self.record_btn.pack(side=tk.LEFT, padx=5)

        self.save_btn = ttk.Button(top, text="Save CSV", command=self._save_csv, state=tk.DISABLED)
        self.save_btn.pack(side=tk.LEFT, padx=5)

        ttk.Button(top, text="Clear Graphs", command=self._clear_data).pack(side=tk.LEFT, padx=5)

        # Status bar (right side of top bar)
        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(top, textvariable=self.status_var, foreground="gray").pack(side=tk.RIGHT, padx=5)

        self.pkt_count_var = tk.StringVar(value="Packets: 0")
        ttk.Label(top, textvariable=self.pkt_count_var).pack(side=tk.RIGHT, padx=10)

        self.rec_label_var = tk.StringVar(value="")
        self.rec_label = ttk.Label(top, textvariable=self.rec_label_var, foreground="red")
        self.rec_label.pack(side=tk.RIGHT, padx=5)

        # Main area: left = graphs, right = live values
        main_pane = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_pane.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Left: graphs
        graph_frame = ttk.Frame(main_pane)
        main_pane.add(graph_frame, weight=4)

        nrows = 3
        ncols = 2
        self.fig = Figure(figsize=(10, 7), dpi=100)
        self.fig.set_facecolor("#2b2b2b")
        self.axes = []
        self.lines = {}

        for idx, group in enumerate(GRAPH_GROUPS):
            ax = self.fig.add_subplot(nrows, ncols, idx + 1)
            ax.set_facecolor("#1e1e1e")
            ax.set_title(group["title"], color="white", fontsize=10, pad=4)
            ax.set_ylabel(group["ylabel"], color="white", fontsize=8)
            ax.tick_params(colors="white", labelsize=7)
            for spine in ax.spines.values():
                spine.set_color("#555")
            ax.grid(True, color="#333", linewidth=0.5)
            if "ylim" in group:
                ax.set_ylim(*group["ylim"])

            group_lines = []
            for fi, field in enumerate(group["fields"]):
                (line,) = ax.plot([], [], color=group["colors"][fi], linewidth=1.2, label=field)
                group_lines.append((field, line))
            if len(group["fields"]) > 1:
                ax.legend(fontsize=7, loc="upper left", facecolor="#2b2b2b", edgecolor="#555", labelcolor="white")
            self.lines[idx] = group_lines
            self.axes.append(ax)

        self.fig.tight_layout(pad=2.0)
        self.canvas = FigureCanvasTkAgg(self.fig, master=graph_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Right: live values panel
        values_frame = ttk.LabelFrame(main_pane, text="Live Telemetry", padding=10)
        main_pane.add(values_frame, weight=1)

        self.value_labels: dict[str, ttk.Label] = {}
        display_fields = [
            ("Timestamp", "timestamp_ms", "ms"),
            ("Temperature", "temperature", "C"),
            ("Pressure", "pressure", "hPa"),
            ("Humidity", "humidity", "%"),
            ("Gas Resistance", "gas_resistance", "ohm"),
            ("Latitude", "lat_e7", "e-7 deg"),
            ("Longitude", "lon_e7", "e-7 deg"),
            ("Altitude", "alt_mm", "mm"),
            ("Roll", "roll", "deg"),
            ("Pitch", "pitch", "deg"),
            ("Yaw", "yaw", "deg"),
        ]

        for i, (label_text, field, unit) in enumerate(display_fields):
            row_frame = ttk.Frame(values_frame)
            row_frame.pack(fill=tk.X, pady=2)
            ttk.Label(row_frame, text=f"{label_text}:", width=16, anchor="w").pack(side=tk.LEFT)
            val_label = ttk.Label(row_frame, text="---", width=14, anchor="e", font=("Courier", 12))
            val_label.pack(side=tk.LEFT, padx=(0, 4))
            ttk.Label(row_frame, text=unit, foreground="gray").pack(side=tk.LEFT)
            self.value_labels[field] = val_label

        # Raw packet display at bottom of values panel
        ttk.Separator(values_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=8)
        ttk.Label(values_frame, text="Last Raw Packet:").pack(anchor="w")
        self.raw_var = tk.StringVar(value="---")
        raw_label = ttk.Label(values_frame, textvariable=self.raw_var, wraplength=280, foreground="gray",
                              font=("Courier", 9))
        raw_label.pack(anchor="w", pady=2)

        # Console log at the very bottom
        console_frame = ttk.LabelFrame(self.root, text="Serial Log", padding=2)
        console_frame.pack(fill=tk.X, padx=5, pady=(0, 5))

        self.console = tk.Text(console_frame, height=4, bg="#1e1e1e", fg="#aaa", font=("Courier", 9),
                               state=tk.DISABLED, wrap=tk.WORD)
        self.console.pack(fill=tk.X)

        # Start graph refresh loop
        self._schedule_graph_update()

    # ── Port management ──────────────────────────────────────────────────

    def _refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        port_names = [p.device for p in ports]
        self.port_combo["values"] = port_names
        if port_names:
            self.port_combo.current(0)

    def _toggle_connection(self):
        if self.running:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get()
        baud = int(self.baud_var.get())
        if not port:
            messagebox.showwarning("No Port", "Select a serial port first.")
            return
        try:
            self.serial_port = serial.Serial(port, baud, timeout=1)
        except serial.SerialException as e:
            messagebox.showerror("Connection Error", str(e))
            return

        self.running = True
        self.connect_btn.config(text="Disconnect")
        self.record_btn.config(state=tk.NORMAL)
        self.status_var.set(f"Connected: {port} @ {baud}")
        self._log(f"Connected to {port} at {baud} baud")

        self.serial_thread = threading.Thread(target=self._serial_reader, daemon=True)
        self.serial_thread.start()

    def _disconnect(self):
        self.running = False
        if self.recording:
            self._toggle_recording()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.serial_port = None
        self.connect_btn.config(text="Connect")
        self.record_btn.config(state=tk.DISABLED)
        self.status_var.set("Disconnected")
        self._log("Disconnected")

    # ── Serial reader thread ─────────────────────────────────────────────

    def _serial_reader(self):
        while self.running and self.serial_port and self.serial_port.is_open:
            try:
                raw = self.serial_port.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                # Log non-packet lines (debug output from receiver)
                if not line.startswith("$CANSAT"):
                    self.root.after(0, self._log, line)
                    continue

                data = parse_packet(line)
                if data is None:
                    self.root.after(0, self._log, f"Bad packet: {line}")
                    continue

                self.root.after(0, self._on_packet, data, line)

            except serial.SerialException:
                self.root.after(0, self._disconnect)
                break
            except Exception as e:
                self.root.after(0, self._log, f"Error: {e}")

    # ── Packet handling (runs on main thread) ────────────────────────────

    def _on_packet(self, data: dict, raw_line: str):
        self.packet_count += 1
        self.last_packet = data
        self.pkt_count_var.set(f"Packets: {self.packet_count}")
        self.raw_var.set(raw_line)

        # Apply temperature offset correction
        if data["temperature"] > -999:
            data["temperature"] += TEMP_OFFSET_C

        # Append to rolling buffers
        t = data["timestamp_ms"] / 1000.0  # seconds
        self.time_data.append(t)
        for field in FIELDS:
            self.series[field].append(data[field])

        # Update live value labels
        for field, label in self.value_labels.items():
            val = data.get(field)
            if val is not None:
                if field == "timestamp_ms":
                    label.config(text=f"{int(val)}")
                elif field in ("lat_e7", "lon_e7"):
                    label.config(text=f"{int(val)}")
                elif field == "alt_mm":
                    label.config(text=f"{val:.0f}")
                elif field == "gas_resistance":
                    label.config(text=f"{val:.0f}")
                else:
                    label.config(text=f"{val:.2f}")

        # Recording
        if self.recording:
            now = datetime.now()
            row = dict(data)
            row["host_time"] = now.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            row["elapsed_s"] = round((now - self.record_start_time).total_seconds(), 3)
            self.recorded_rows.append(row)
            self.rec_label_var.set(f"REC [{len(self.recorded_rows)}]")

    # ── Graph update ─────────────────────────────────────────────────────

    def _schedule_graph_update(self):
        self._update_graphs()
        self.root.after(REFRESH_MS, self._schedule_graph_update)

    def _update_graphs(self):
        if not self.time_data:
            return

        t = list(self.time_data)

        for idx, group in enumerate(GRAPH_GROUPS):
            ax = self.axes[idx]
            scale_map = {}
            if "scale" in group:
                scale_map = {s[0]: s[1] for s in group["scale"]}

            for field, line in self.lines[idx]:
                y = list(self.series[field])
                if field in scale_map:
                    y = [v * scale_map[field] for v in y]
                line.set_data(t[:len(y)], y)

            if t:
                ax.set_xlim(t[0], t[-1] if t[-1] > t[0] else t[0] + 1)

        self.canvas.draw_idle()

    # ── Recording ────────────────────────────────────────────────────────

    def _toggle_recording(self):
        if self.recording:
            self.recording = False
            self.record_btn.config(text="Start Recording")
            self.save_btn.config(state=tk.NORMAL if self.recorded_rows else tk.DISABLED)
            self.rec_label_var.set(f"Stopped [{len(self.recorded_rows)} rows]")
            self._log(f"Recording stopped: {len(self.recorded_rows)} rows captured")
        else:
            self.recorded_rows.clear()
            self.record_start_time = datetime.now()
            self.recording = True
            self.record_btn.config(text="Stop Recording")
            self.save_btn.config(state=tk.DISABLED)
            self.rec_label_var.set("REC [0]")
            self._log("Recording started")

    def _save_csv(self):
        if not self.recorded_rows:
            messagebox.showinfo("No Data", "No recorded data to save.")
            return

        default_name = f"cansat_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        filepath = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            initialfile=default_name,
        )
        if not filepath:
            return

        try:
            with open(filepath, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=["host_time", "elapsed_s"] + FIELDS)
                writer.writeheader()
                writer.writerows(self.recorded_rows)
            self._log(f"Saved {len(self.recorded_rows)} rows to {os.path.basename(filepath)}")
            messagebox.showinfo("Saved", f"Data saved to:\n{filepath}")
        except OSError as e:
            messagebox.showerror("Save Error", str(e))

    # ── Utilities ────────────────────────────────────────────────────────

    def _clear_data(self):
        self.time_data.clear()
        for d in self.series.values():
            d.clear()
        self.packet_count = 0
        self.pkt_count_var.set("Packets: 0")
        self._log("Graphs cleared")

    def _log(self, msg: str):
        self.console.config(state=tk.NORMAL)
        self.console.insert(tk.END, f"[{datetime.now().strftime('%H:%M:%S')}] {msg}\n")
        self.console.see(tk.END)
        # Keep log from growing too large
        lines = int(self.console.index("end-1c").split(".")[0])
        if lines > 200:
            self.console.delete("1.0", "100.0")
        self.console.config(state=tk.DISABLED)

    def on_close(self):
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.root.destroy()


# ── Entry point ──────────────────────────────────────────────────────────────

def main():
    root = tk.Tk()
    app = GroundStationApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
