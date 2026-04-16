#!/usr/bin/env python3

#cd /Users/ericchen/Documents/GitHub/Onelastdecent_CanSat2026/Software/CanSat_Main_Reciever/dashboard
#pip install -r requirements.txt
#python3 ground_station.py

"""
CanSat 2026 Ground Station Dashboard
=====================================
Real-time telemetry display with graphing and CSV recording.

Packet format (USB serial):
  $CANSAT,<ms>,<temp>,<press>,<hum>,<gas>,<lat_e7>,<lon_e7>,<alt_mm>,<roll>,<pitch>,<yaw>
  $FREQ,<freq_mhz>,<channel>
"""

import math
import platform
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import serial
import serial.tools.list_ports
import threading
import csv
import os
import time
from datetime import datetime
from collections import deque

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

try:
    import tkintermapview
    _MAP_AVAILABLE = True
except ImportError:
    _MAP_AVAILABLE = False

# ── Platform font helpers ─────────────────────────────────────────────────────
_IS_MAC = platform.system() == "Darwin"
_SANS   = "Helvetica Neue" if _IS_MAC else "Segoe UI"
_MONO   = "Menlo"          if _IS_MAC else "Consolas"

FONT_LABEL    = (_SANS, 10)
FONT_LABEL_SM = (_SANS, 9)
FONT_LABEL_B  = (_SANS, 10, "bold")
FONT_VALUE    = (_MONO, 14, "bold")
FONT_VALUE_SM = (_MONO, 11, "bold")
FONT_SECTION  = (_SANS, 9,  "bold")
FONT_TITLE    = (_SANS, 13, "bold")
FONT_FREQ     = (_MONO, 18, "bold")
FONT_CONSOLE  = (_MONO, 9)

# ── Colour palette ────────────────────────────────────────────────────────────
BG      = "#080808"   # root window background
CARD    = "#0e0e0e"   # panel / card background
CARD2   = "#141414"   # inset / toolbar background
BORDER  = "#242424"   # borders, dividers
TEXT    = "#f0f0f0"   # primary text
SUBTEXT = "#666666"   # muted labels
GREEN   = "#28a745"   # good / connected
RED     = "#e02020"   # primary accent / error
ORANGE  = "#c41a1a"   # secondary red accent
BLUE    = "#4488ee"   # environmental
PURPLE  = "#9966ff"   # attitude
YELLOW  = "#cc9900"   # yaw
PINK    = "#ee3366"   # gas resistance
TEAL    = "#22aacc"   # pitch

GRAPH_BG   = "#080808"
GRAPH_CARD = "#0e0e0e"

# ── Data config ────────────────────────────────────────────────────────────────
FIELDS = [
    "timestamp_ms", "temperature", "pressure", "humidity", "gas_resistance",
    "lat_e7", "lon_e7", "alt_mm", "roll", "pitch", "yaw",
    "fc_bytes", "stx_v1", "stx_v2",
]

GRAPH_GROUPS = [
    {"title": "Temperature",    "fields": ["temperature"],          "colors": [RED],                    "ylabel": "°C",  "ylim": (-20, 60)},
    {"title": "Pressure",       "fields": ["pressure"],             "colors": [BLUE],                   "ylabel": "hPa", "ylim": (950, 1050)},
    {"title": "Humidity",       "fields": ["humidity"],             "colors": [GREEN],                  "ylabel": "%",   "ylim": (0, 100)},
    {"title": "Altitude",       "fields": ["alt_mm"],               "colors": [PURPLE],                 "ylabel": "m",   "scale": [("alt_mm", 0.001)],          "ylim": (-50, 1000)},
    {"title": "Orientation",    "fields": ["roll", "pitch", "yaw"],"colors": [ORANGE, TEAL, YELLOW],   "ylabel": "deg", "ylim": (-180, 180)},
    {"title": "Gas Resistance", "fields": ["gas_resistance"],       "colors": [PINK],                   "ylabel": "kΩ",  "scale": [("gas_resistance", 0.001)],  "ylim": (0, 500)},
]

TEMP_OFFSET_C = -6.41
MAX_POINTS    = 300
REFRESH_MS    = 200


# ── Packet parsing ─────────────────────────────────────────────────────────────

def parse_packet(line: str) -> dict | None:
    """Parse a $CANSAT CSV line into a dict. Returns None on failure."""
    line = line.strip()
    if not line.startswith("$CANSAT,"):
        return None
    parts = line[len("$CANSAT,"):].split(",")
    if len(parts) != len(FIELDS):
        return None
    try:
        return {name: float(parts[i]) for i, name in enumerate(FIELDS)}
    except ValueError:
        return None


def parse_gspos_packet(line: str) -> dict | None:
    """Parse $GSPOS,<lat_e7>,<lon_e7>,<alt_mm>,<hdop_100>,<fix> from ground-station GPS."""
    line = line.strip()
    if not line.startswith("$GSPOS,"):
        return None
    parts = line[len("$GSPOS,"):].split(",")
    if len(parts) != 5:
        return None
    try:
        return {
            "lat_e7":   int(parts[0]),
            "lon_e7":   int(parts[1]),
            "alt_mm":   int(parts[2]),
            "hdop_100": int(parts[3]),
            "fix":      int(parts[4]),
        }
    except ValueError:
        return None


def parse_freq_packet(line: str):
    """Parse $FREQ,<mhz>,<CH65> -> (freq_mhz: float, channel: str) or None."""
    line = line.strip()
    if not line.startswith("$FREQ,"):
        return None
    parts = line[len("$FREQ,"):].split(",")
    if len(parts) >= 2:
        try:
            return float(parts[0]), parts[1]
        except ValueError:
            pass
    return None


# ── Main application ───────────────────────────────────────────────────────────

class GroundStationApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("CanSat 2026 — Ground Station")
        self.root.geometry("1440x900")
        self.root.minsize(1100, 720)
        self.root.configure(bg=BG)

        # State
        self.serial_port: serial.Serial | None = None
        self.serial_thread: threading.Thread | None = None
        self.running   = False
        self.recording = False
        self.recorded_rows: list[dict] = []
        self.packet_count = 0
        self.last_packet: dict | None = None
        self.last_packet_time: float | None = None
        self._overlay_blink_on = False

        # Ground-station GPS state
        self.gs_lat: float | None = None
        self.gs_lon: float | None = None
        self.gs_alt_m: float | None = None
        self.gs_fix: int = 0
        self.gs_map_marker = None

        # Rolling data for graphs
        self.time_data = deque(maxlen=MAX_POINTS)
        self.series: dict[str, deque] = {f: deque(maxlen=MAX_POINTS) for f in FIELDS}

        self._apply_style()
        self._build_ui()
        self._refresh_ports()
        self._schedule_signal_check()

    # ── Theming ────────────────────────────────────────────────────────────

    def _apply_style(self):
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        style.configure(".",                background=CARD,  foreground=TEXT,   borderwidth=0)
        style.configure("TFrame",           background=CARD)
        style.configure("TLabel",           background=CARD,  foreground=TEXT)
        style.configure("TLabelframe",      background=CARD,  foreground=TEXT)
        style.configure("TLabelframe.Label",background=CARD,  foreground=SUBTEXT, font=FONT_SECTION)
        style.configure("TSeparator",       background=BORDER)
        style.configure("TScrollbar",       background=CARD2, troughcolor=CARD, arrowcolor=SUBTEXT, borderwidth=0)

        # Default button
        style.configure("TButton",
                        background=CARD2, foreground=TEXT,
                        relief="flat", borderwidth=1, bordercolor=BORDER,
                        focusthickness=0, padding=(10, 5), font=FONT_LABEL)
        style.map("TButton",
                  background=[("active", BORDER), ("pressed", BORDER)],
                  foreground=[("active", TEXT)])

        # Connect (green)
        style.configure("Accent.TButton",
                        background=GREEN, foreground=BG,
                        relief="flat", borderwidth=0,
                        focusthickness=0, padding=(10, 5), font=FONT_LABEL_B)
        style.map("Accent.TButton",
                  background=[("active", "#2ea043"), ("pressed", "#238636")],
                  foreground=[("active", BG)])

        # Disconnect (red)
        style.configure("Danger.TButton",
                        background="#1a0000", foreground=RED,
                        relief="flat", borderwidth=1, bordercolor="#550000",
                        focusthickness=0, padding=(10, 5), font=FONT_LABEL_B)
        style.map("Danger.TButton",
                  background=[("active", "#2a0000")])

        # Record (red)
        style.configure("Record.TButton",
                        background="#1a0000", foreground=RED,
                        relief="flat", borderwidth=1, bordercolor="#550000",
                        focusthickness=0, padding=(10, 5), font=FONT_LABEL_B)
        style.map("Record.TButton",
                  background=[("active", "#2a0000")])

        # Notebook tabs
        style.configure("TNotebook",
                        background=BG, borderwidth=0, tabmargins=0)
        style.configure("TNotebook.Tab",
                        background=CARD2, foreground=SUBTEXT,
                        padding=(14, 6), font=FONT_LABEL_B, borderwidth=0)
        style.map("TNotebook.Tab",
                  background=[("selected", CARD), ("active", BORDER)],
                  foreground=[("selected", TEXT), ("active", TEXT)])

        # Combobox
        style.configure("TCombobox",
                        fieldbackground=CARD2, background=CARD2,
                        foreground=TEXT, arrowcolor=SUBTEXT,
                        selectbackground=BLUE, selectforeground=BG,
                        padding=(4, 4))
        style.map("TCombobox",
                  fieldbackground=[("readonly", CARD2)],
                  selectbackground=[("readonly", CARD2)],
                  selectforeground=[("readonly", TEXT)])

    # ── UI construction ────────────────────────────────────────────────────

    def _build_ui(self):
        self._build_toolbar()
        self._build_main_area()
        self._build_console()
        self._schedule_graph_update()

    # ── Toolbar ────────────────────────────────────────────────────────────

    def _build_toolbar(self):
        toolbar = tk.Frame(self.root, bg=CARD2, height=56)
        toolbar.pack(fill=tk.X, side=tk.TOP)
        toolbar.pack_propagate(False)
        tk.Frame(self.root, bg=BORDER, height=1).pack(fill=tk.X)

        # ── Left: Logo + title ─────────────────────────────────────────────
        logo_frame = tk.Frame(toolbar, bg=CARD2)
        logo_frame.pack(side=tk.LEFT, padx=(16, 12), pady=8)
        tk.Label(logo_frame, text="▲", bg=CARD2, fg=ORANGE,
                 font=FONT_TITLE).pack(side=tk.LEFT)
        tk.Label(logo_frame, text=" CANSAT 2026", bg=CARD2, fg=TEXT,
                 font=FONT_TITLE).pack(side=tk.LEFT)

        self._vsep(toolbar)

        # ── Port + baud + connect ──────────────────────────────────────────
        conn_frame = tk.Frame(toolbar, bg=CARD2)
        conn_frame.pack(side=tk.LEFT, padx=4, pady=8)

        tk.Label(conn_frame, text="PORT", bg=CARD2, fg=SUBTEXT,
                 font=FONT_LABEL_SM).pack(side=tk.LEFT, padx=(0, 4))
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var,
                                       width=18, state="readonly")
        self.port_combo.pack(side=tk.LEFT)

        ttk.Button(conn_frame, text="↺", command=self._refresh_ports,
                   width=3).pack(side=tk.LEFT, padx=(4, 12))

        tk.Label(conn_frame, text="BAUD", bg=CARD2, fg=SUBTEXT,
                 font=FONT_LABEL_SM).pack(side=tk.LEFT, padx=(0, 4))
        self.baud_var = tk.StringVar(value="115200")
        ttk.Combobox(conn_frame, textvariable=self.baud_var,
                     values=["9600", "115200"], width=8,
                     state="readonly").pack(side=tk.LEFT)

        self.connect_btn = ttk.Button(conn_frame, text="Connect",
                                      command=self._toggle_connection,
                                      style="Accent.TButton")
        self.connect_btn.pack(side=tk.LEFT, padx=(12, 0))

        self._vsep(toolbar)

        # ── Data controls ──────────────────────────────────────────────────
        data_frame = tk.Frame(toolbar, bg=CARD2)
        data_frame.pack(side=tk.LEFT, padx=4, pady=8)

        self.record_btn = ttk.Button(data_frame, text="⏺  Record",
                                     command=self._toggle_recording,
                                     style="Record.TButton", state=tk.DISABLED)
        self.record_btn.pack(side=tk.LEFT, padx=(0, 4))

        self.save_btn = ttk.Button(data_frame, text="Save CSV",
                                   command=self._save_csv, state=tk.DISABLED)
        self.save_btn.pack(side=tk.LEFT, padx=(0, 4))

        ttk.Button(data_frame, text="Clear",
                   command=self._clear_data).pack(side=tk.LEFT)

        # ── Right: Radio freq badge + status ──────────────────────────────
        # Status cluster (rightmost)
        status_frame = tk.Frame(toolbar, bg=CARD2)
        status_frame.pack(side=tk.RIGHT, padx=(0, 16), pady=8)

        self.pkt_count_var = tk.StringVar(value="0 pkts")
        tk.Label(status_frame, textvariable=self.pkt_count_var,
                 bg=CARD2, fg=SUBTEXT, font=FONT_LABEL_SM).pack(side=tk.RIGHT, padx=(12, 0))

        self.rec_label_var = tk.StringVar(value="")
        tk.Label(status_frame, textvariable=self.rec_label_var,
                 bg=CARD2, fg=RED, font=FONT_LABEL_B).pack(side=tk.RIGHT, padx=8)

        self.status_dot = tk.Label(status_frame, text="●", bg=CARD2, fg=RED,
                                   font=(_SANS, 14))
        self.status_dot.pack(side=tk.RIGHT, padx=(0, 4))
        self.status_var = tk.StringVar(value="Disconnected")
        tk.Label(status_frame, textvariable=self.status_var,
                 bg=CARD2, fg=SUBTEXT, font=FONT_LABEL_SM).pack(side=tk.RIGHT, padx=4)

        # Radio frequency badge
        self._vsep(toolbar, right=True)

        freq_badge = tk.Frame(toolbar, bg=CARD, padx=12, pady=4)
        freq_badge.pack(side=tk.RIGHT, pady=6)
        # small "RF" tag
        tk.Label(freq_badge, text="RF", bg=CARD, fg=SUBTEXT,
                 font=FONT_SECTION).pack(side=tk.LEFT, padx=(0, 8))
        # large frequency readout
        self.freq_var = tk.StringVar(value="--- MHz")
        tk.Label(freq_badge, textvariable=self.freq_var, bg=CARD, fg=ORANGE,
                 font=FONT_FREQ).pack(side=tk.LEFT)
        # channel label
        self.chan_var = tk.StringVar(value="")
        tk.Label(freq_badge, textvariable=self.chan_var, bg=CARD, fg=SUBTEXT,
                 font=FONT_LABEL_SM).pack(side=tk.LEFT, padx=(8, 0))

        self._vsep(toolbar, right=True)

    def _vsep(self, parent: tk.Frame, right: bool = False):
        side = tk.RIGHT if right else tk.LEFT
        tk.Frame(parent, bg=BORDER, width=1).pack(side=side, fill=tk.Y, padx=6, pady=10)

    # ── Main content ───────────────────────────────────────────────────────

    def _build_main_area(self):
        content = tk.Frame(self.root, bg=BG)
        content.pack(fill=tk.BOTH, expand=True, padx=8, pady=(6, 0))

        # Notebook (left, expands) — Graphs + Map tabs
        self.notebook = ttk.Notebook(content)
        self.notebook.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # ── Graphs tab ────────────────────────────────────────────────────
        self.graph_outer = tk.Frame(self.notebook, bg=CARD)
        self.notebook.add(self.graph_outer, text="  Graphs  ")

        self.fig = Figure(figsize=(10, 7), dpi=100)
        self.fig.set_facecolor(GRAPH_BG)
        self.axes = []
        self.lines = {}
        self._build_graphs()

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.graph_outer)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self._build_signal_overlay()

        # ── Map tab ───────────────────────────────────────────────────────
        self.map_frame = tk.Frame(self.notebook, bg=CARD)
        self.notebook.add(self.map_frame, text="  Map  ")
        self._build_map()

        # Divider
        tk.Frame(content, bg=BORDER, width=1).pack(side=tk.LEFT, fill=tk.Y, padx=(4, 0))

        # Telemetry panel (right, fixed width)
        right_panel = tk.Frame(content, bg=CARD, width=420)
        right_panel.pack(side=tk.RIGHT, fill=tk.Y)
        right_panel.pack_propagate(False)
        self._build_telemetry_panel(right_panel)

    def _build_signal_overlay(self):
        """Full-size overlay drawn on top of the graph area for warnings."""
        self.overlay = tk.Frame(self.graph_outer, bg="#0d0000")
        # Two lines of text: big icon/status + detail
        self.overlay_icon  = tk.Label(self.overlay, text="⚠", bg="#0d0000", fg=RED,
                                      font=(_SANS, 72, "bold"))
        self.overlay_icon.pack(expand=True, pady=(60, 0))
        self.overlay_title = tk.Label(self.overlay, text="NOT CONNECTED",
                                      bg="#0d0000", fg=RED,
                                      font=(_SANS, 36, "bold"))
        self.overlay_title.pack()
        self.overlay_sub   = tk.Label(self.overlay, text="Select a port and click Connect",
                                      bg="#0d0000", fg="#882222",
                                      font=(_SANS, 14))
        self.overlay_sub.pack(pady=(8, 0))
        # Show it immediately (disconnected at startup)
        self.overlay.place(relx=0, rely=0, relwidth=1, relheight=1)

    def _build_map(self):
        self.gps_positions: list[tuple[float, float]] = []
        self.map_marker = None
        self.map_path   = None
        self._map_mode  = "tile"

        # ── Toolbar ───────────────────────────────────────────────────────
        map_tb = tk.Frame(self.map_frame, bg=CARD2, height=36)
        map_tb.pack(fill=tk.X)
        map_tb.pack_propagate(False)
        tk.Frame(self.map_frame, bg=BORDER, height=1).pack(fill=tk.X)

        self._tile_btn   = ttk.Button(map_tb, text="Tile Map",
                                      command=lambda: self._set_map_mode("tile"))
        self._simple_btn = ttk.Button(map_tb, text="Simple Plot (Offline)",
                                      command=lambda: self._set_map_mode("simple"))
        self._tile_btn.pack(side=tk.LEFT, padx=(8, 2), pady=4)
        self._simple_btn.pack(side=tk.LEFT, padx=2, pady=4)

        if _MAP_AVAILABLE:
            ttk.Button(map_tb, text="Pre-download Tiles",
                       command=self._predownload_tiles).pack(side=tk.LEFT, padx=(16, 2), pady=4)

        # ── Tile map sub-frame ────────────────────────────────────────────
        self.tile_map_frame = tk.Frame(self.map_frame, bg=CARD)

        if _MAP_AVAILABLE:
            self.map_widget = tkintermapview.TkinterMapView(
                self.tile_map_frame, corner_radius=0)
            self.map_widget.pack(fill=tk.BOTH, expand=True)
            self.map_widget.set_zoom(2)
        else:
            tk.Label(self.tile_map_frame,
                     text="tkintermapview not installed\npip install tkintermapview",
                     bg=CARD, fg=SUBTEXT, font=FONT_LABEL).pack(expand=True)
            self.map_widget = None

        # ── Simple plot sub-frame ─────────────────────────────────────────
        self.simple_map_frame = tk.Frame(self.map_frame, bg=CARD)
        self._build_simple_plot()

        self._set_map_mode("tile")

    def _build_simple_plot(self):
        self.simple_fig = Figure(dpi=100)
        self.simple_fig.set_facecolor(GRAPH_BG)
        ax = self.simple_fig.add_subplot(111)
        self.simple_ax = ax
        ax.set_facecolor(GRAPH_CARD)
        ax.set_title("GPS Track", color=TEXT, fontsize=10, pad=6)
        ax.set_xlabel("Longitude (°)", color=SUBTEXT, fontsize=8)
        ax.set_ylabel("Latitude (°)",  color=SUBTEXT, fontsize=8)
        ax.tick_params(colors=SUBTEXT, labelsize=7)
        for spine in ax.spines.values():
            spine.set_color(BORDER)
        ax.grid(True, color=BORDER, linewidth=0.5, alpha=0.8)

        self.simple_path_line, = ax.plot([], [], color=ORANGE, linewidth=1.5)
        self.simple_latest_dot, = ax.plot([], [], "o", color=RED, markersize=8,
                                          zorder=5, label="CanSat")
        self.simple_gs_dot, = ax.plot([], [], "^", color=BLUE, markersize=10,
                                      zorder=6, label="Ground Station")
        ax.legend(fontsize=7, loc="upper left",
                  facecolor=CARD, edgecolor=BORDER,
                  labelcolor=TEXT, framealpha=0.9)

        self.simple_fig.tight_layout(pad=2.5)
        self.simple_canvas = FigureCanvasTkAgg(self.simple_fig,
                                               master=self.simple_map_frame)
        self.simple_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _set_map_mode(self, mode: str):
        self._map_mode = mode
        if mode == "tile":
            self.simple_map_frame.pack_forget()
            self.tile_map_frame.pack(fill=tk.BOTH, expand=True)
        else:
            self.tile_map_frame.pack_forget()
            self.simple_map_frame.pack(fill=tk.BOTH, expand=True)
            self._update_simple_plot()

    def _update_simple_plot(self):
        all_lats: list[float] = []
        all_lons: list[float] = []

        if self.gps_positions:
            lats = [p[0] for p in self.gps_positions]
            lons = [p[1] for p in self.gps_positions]
            self.simple_path_line.set_data(lons, lats)
            self.simple_latest_dot.set_data([lons[-1]], [lats[-1]])
            all_lats.extend(lats)
            all_lons.extend(lons)

        if self.gs_lat is not None and self.gs_lon is not None:
            self.simple_gs_dot.set_data([self.gs_lon], [self.gs_lat])
            all_lats.append(self.gs_lat)
            all_lons.append(self.gs_lon)

        if not all_lats:
            return
        pad_lat = max((max(all_lats) - min(all_lats)) * 0.15, 0.0002)
        pad_lon = max((max(all_lons) - min(all_lons)) * 0.15, 0.0002)
        self.simple_ax.set_xlim(min(all_lons) - pad_lon, max(all_lons) + pad_lon)
        self.simple_ax.set_ylim(min(all_lats) - pad_lat, max(all_lats) + pad_lat)
        self.simple_canvas.draw_idle()

    def _update_map(self, data: dict):
        lat = data["lat_e7"] / 1e7
        lon = data["lon_e7"] / 1e7
        if lat == 0.0 and lon == 0.0:
            return

        self.gps_positions.append((lat, lon))

        # Tile map
        if _MAP_AVAILABLE and self.map_widget is not None:
            if self.map_marker is None:
                self.map_marker = self.map_widget.set_marker(
                    lat, lon, text="CanSat",
                    marker_color_circle=ORANGE, marker_color_outside=RED)
                self.map_widget.set_position(lat, lon)
                self.map_widget.set_zoom(15)
            else:
                self.map_marker.set_position(lat, lon)
            if len(self.gps_positions) >= 2:
                if self.map_path is not None:
                    self.map_path.delete()
                self.map_path = self.map_widget.set_path(
                    self.gps_positions, color=ORANGE, width=2)

        # Simple plot (only redraw if currently visible)
        if self._map_mode == "simple":
            self._update_simple_plot()

    def _update_gs_map(self, lat: float, lon: float):
        """Update the ground-station marker on both map views."""
        if _MAP_AVAILABLE and self.map_widget is not None:
            if self.gs_map_marker is None:
                self.gs_map_marker = self.map_widget.set_marker(
                    lat, lon, text="Ground Station",
                    marker_color_circle=BLUE, marker_color_outside="#2244cc")
            else:
                self.gs_map_marker.set_position(lat, lon)

        self.simple_gs_dot.set_data([lon], [lat])
        if self._map_mode == "simple":
            self._update_simple_plot()

    def _clear_map(self):
        self.gps_positions.clear()
        if self.map_marker is not None:
            self.map_marker.delete()
            self.map_marker = None
        if self.map_path is not None:
            self.map_path.delete()
            self.map_path = None
        if self.gs_map_marker is not None:
            self.gs_map_marker.delete()
            self.gs_map_marker = None
        self.simple_path_line.set_data([], [])
        self.simple_latest_dot.set_data([], [])
        self.simple_gs_dot.set_data([], [])
        self.simple_ax.set_xlim(0, 1)
        self.simple_ax.set_ylim(0, 1)
        self.simple_canvas.draw_idle()

    # ── Offline tile pre-download ──────────────────────────────────────────

    def _predownload_tiles(self):
        dlg = tk.Toplevel(self.root)
        dlg.title("Pre-download Offline Tiles")
        dlg.configure(bg=CARD)
        dlg.geometry("380x290")
        dlg.resizable(False, False)
        dlg.grab_set()

        lat_c = self.gps_positions[-1][0] if self.gps_positions else 49.19
        lon_c = self.gps_positions[-1][1] if self.gps_positions else -122.76
        r = 0.10  # ~10 km radius

        tk.Label(dlg, text="Pre-download tiles for offline use",
                 bg=CARD, fg=TEXT, font=FONT_LABEL_B).pack(pady=(14, 8))

        fields_frame = tk.Frame(dlg, bg=CARD)
        fields_frame.pack(padx=20, fill=tk.X)

        entries: dict[str, tk.StringVar] = {}
        for label, key, default in [
            ("Top-left Lat",      "tl_lat", f"{lat_c + r:.5f}"),
            ("Top-left Lon",      "tl_lon", f"{lon_c - r:.5f}"),
            ("Bottom-right Lat",  "br_lat", f"{lat_c - r:.5f}"),
            ("Bottom-right Lon",  "br_lon", f"{lon_c + r:.5f}"),
            ("Max zoom (1–19)",   "zoom",   "17"),
        ]:
            row = tk.Frame(fields_frame, bg=CARD)
            row.pack(fill=tk.X, pady=3)
            tk.Label(row, text=label, bg=CARD, fg=SUBTEXT,
                     font=FONT_LABEL_SM, width=18, anchor="w").pack(side=tk.LEFT)
            var = tk.StringVar(value=default)
            tk.Entry(row, textvariable=var, bg=CARD2, fg=TEXT,
                     insertbackground=TEXT, relief="flat",
                     font=FONT_LABEL_SM).pack(side=tk.LEFT, fill=tk.X, expand=True)
            entries[key] = var

        def _start():
            try:
                tl   = (float(entries["tl_lat"].get()), float(entries["tl_lon"].get()))
                br   = (float(entries["br_lat"].get()), float(entries["br_lon"].get()))
                zoom = max(1, min(19, int(entries["zoom"].get())))
            except ValueError:
                messagebox.showerror("Invalid input", "Check that all values are numbers.", parent=dlg)
                return
            dlg.destroy()
            self._log(f"Tile download started: {tl} → {br}, zoom 1–{zoom}")
            threading.Thread(target=self._run_tile_download,
                             args=(tl, br, zoom), daemon=True).start()

        ttk.Button(dlg, text="Download", command=_start,
                   style="Accent.TButton").pack(pady=14)

    def _run_tile_download(self, top_left, bottom_right, max_zoom):
        try:
            from tkintermapview import OfflineLoader
            loader = OfflineLoader()
            loader.save_offline_tiles(
                [top_left, bottom_right], zoom_range=(1, max_zoom))
            self.root.after(0, self._log, "Tile download complete — map works offline for this area.")
        except Exception as e:
            self.root.after(0, self._log, f"Tile download failed: {e}")

    def _build_graphs(self):
        for idx, group in enumerate(GRAPH_GROUPS):
            ax = self.fig.add_subplot(3, 2, idx + 1)
            ax.set_facecolor(GRAPH_CARD)
            ax.set_title(group["title"], color=TEXT, fontsize=9, pad=4)
            ax.set_ylabel(group["ylabel"], color=SUBTEXT, fontsize=8)
            ax.tick_params(colors=SUBTEXT, labelsize=7)
            for spine in ax.spines.values():
                spine.set_color(BORDER)
            ax.grid(True, color=BORDER, linewidth=0.5, alpha=0.8)
            if "ylim" in group:
                ax.set_ylim(*group["ylim"])

            group_lines = []
            for fi, field in enumerate(group["fields"]):
                (line,) = ax.plot([], [], color=group["colors"][fi],
                                  linewidth=1.5, label=field)
                group_lines.append((field, line))
            if len(group["fields"]) > 1:
                ax.legend(fontsize=7, loc="upper left",
                          facecolor=CARD, edgecolor=BORDER,
                          labelcolor=TEXT, framealpha=0.9)
            self.lines[idx] = group_lines
            self.axes.append(ax)

        self.fig.tight_layout(pad=2.5)

    # ── Telemetry panel ────────────────────────────────────────────────────

    def _build_telemetry_panel(self, parent: tk.Frame):
        self.value_labels: dict[str, tk.Label] = {}

        # Header row
        hdr = tk.Frame(parent, bg=CARD)
        hdr.pack(fill=tk.X, padx=12, pady=(10, 6))
        tk.Label(hdr, text="LIVE TELEMETRY", bg=CARD, fg=TEXT,
                 font=FONT_SECTION).pack(side=tk.LEFT)
        self.last_pkt_var = tk.StringVar(value="")
        tk.Label(hdr, textvariable=self.last_pkt_var, bg=CARD,
                 fg=SUBTEXT, font=FONT_LABEL_SM).pack(side=tk.RIGHT)

        tk.Frame(parent, bg=BORDER, height=1).pack(fill=tk.X)

        # ── Uptime ────────────────────────────────────────────────────────
        uptime_row = tk.Frame(parent, bg=CARD)
        uptime_row.pack(fill=tk.X, padx=12, pady=(8, 4))
        tk.Label(uptime_row, text="Uptime", bg=CARD, fg=SUBTEXT,
                 font=FONT_LABEL_SM, width=10, anchor="w").pack(side=tk.LEFT)
        self.uptime_var = tk.StringVar(value="--:--")
        tk.Label(uptime_row, textvariable=self.uptime_var, bg=CARD, fg=TEXT,
                 font=FONT_VALUE_SM, anchor="e").pack(side=tk.RIGHT)

        tk.Frame(parent, bg=BORDER, height=1).pack(fill=tk.X, padx=12, pady=(6, 2))

        # ── Environmental ─────────────────────────────────────────────────
        self._section_header(parent, "Environmental", BLUE)
        for label, field, unit in [
            ("Temperature", "temperature",   "°C"),
            ("Pressure",    "pressure",      "hPa"),
            ("Humidity",    "humidity",      "%"),
            ("Gas Resist.", "gas_resistance","kΩ"),
        ]:
            self._value_row(parent, label, field, unit)

        tk.Frame(parent, bg=BORDER, height=1).pack(fill=tk.X, padx=12, pady=(6, 2))

        # ── Navigation ────────────────────────────────────────────────────
        self._section_header(parent, "Navigation", GREEN)
        for label, field, unit in [
            ("Latitude",  "lat_e7", "×10⁻⁷°"),
            ("Longitude", "lon_e7", "×10⁻⁷°"),
            ("Altitude",  "alt_mm", "m"),
        ]:
            self._value_row(parent, label, field, unit)

        tk.Frame(parent, bg=BORDER, height=1).pack(fill=tk.X, padx=12, pady=(6, 2))

        # ── Attitude ──────────────────────────────────────────────────────
        self._section_header(parent, "Attitude", PURPLE)
        for label, field, unit in [
            ("Roll",  "roll",  "deg"),
            ("Pitch", "pitch", "deg"),
            ("Yaw",   "yaw",   "deg"),
        ]:
            self._value_row(parent, label, field, unit)

        tk.Frame(parent, bg=BORDER, height=1).pack(fill=tk.X, padx=12, pady=(6, 2))

        # ── Ground Station GPS ────────────────────────────────────────────
        self._section_header(parent, "Ground Station GPS", TEAL)
        for label, field, unit in [
            ("GS Lat",   "gs_lat",     "°"),
            ("GS Lon",   "gs_lon",     "°"),
            ("GS Alt",   "gs_alt",     "m"),
            ("GPS Fix",  "gs_fix",     ""),
            ("Distance", "gs_dist",    "m"),
            ("Bearing",  "gs_bearing", "°"),
        ]:
            self._value_row(parent, label, field, unit)

        tk.Frame(parent, bg=BORDER, height=1).pack(fill=tk.X, padx=12, pady=(6, 2))

        # ── Last raw packet ───────────────────────────────────────────────
        raw_outer = tk.Frame(parent, bg=CARD)
        raw_outer.pack(fill=tk.X, padx=12, pady=(4, 8))
        tk.Label(raw_outer, text="LAST PACKET", bg=CARD, fg=SUBTEXT,
                 font=FONT_SECTION).pack(anchor="w")
        self.raw_var = tk.StringVar(value="---")
        tk.Label(raw_outer, textvariable=self.raw_var, bg=CARD, fg=SUBTEXT,
                 font=FONT_CONSOLE, wraplength=385, anchor="w",
                 justify="left").pack(anchor="w", pady=(2, 0))

    def _section_header(self, parent: tk.Frame, title: str, color: str):
        f = tk.Frame(parent, bg=CARD)
        f.pack(fill=tk.X, padx=8, pady=(4, 2))
        tk.Frame(f, bg=color, width=3).pack(side=tk.LEFT, fill=tk.Y)
        tk.Label(f, text=f"  {title.upper()}", bg=CARD, fg=color,
                 font=FONT_SECTION).pack(side=tk.LEFT)

    def _value_row(self, parent: tk.Frame, label: str, field: str, unit: str):
        f = tk.Frame(parent, bg=CARD)
        f.pack(fill=tk.X, padx=14, pady=2)
        tk.Label(f, text=label, bg=CARD, fg=SUBTEXT,
                 font=FONT_LABEL, width=12, anchor="w").pack(side=tk.LEFT)
        val_lbl = tk.Label(f, text="---", bg=CARD, fg=TEXT,
                           font=FONT_VALUE, width=12, anchor="e")
        val_lbl.pack(side=tk.LEFT)
        tk.Label(f, text=f" {unit}", bg=CARD, fg=SUBTEXT,
                 font=FONT_LABEL_SM, width=8, anchor="w").pack(side=tk.LEFT)
        self.value_labels[field] = val_lbl

    # ── Console bar ────────────────────────────────────────────────────────

    def _build_console(self):
        tk.Frame(self.root, bg=BORDER, height=1).pack(fill=tk.X)
        console_outer = tk.Frame(self.root, bg=CARD2)
        console_outer.pack(fill=tk.X, side=tk.BOTTOM)

        hdr = tk.Frame(console_outer, bg=CARD2)
        hdr.pack(fill=tk.X, padx=10, pady=(4, 0))
        tk.Label(hdr, text="SERIAL LOG", bg=CARD2, fg=SUBTEXT,
                 font=FONT_SECTION).pack(side=tk.LEFT)
        ttk.Button(hdr, text="Clear",
                   command=self._clear_log).pack(side=tk.RIGHT)

        self.console = tk.Text(
            console_outer, height=4,
            bg=CARD2, fg=SUBTEXT,
            font=FONT_CONSOLE, state=tk.DISABLED, wrap=tk.WORD,
            insertbackground=TEXT, selectbackground=BORDER,
            relief="flat", borderwidth=0,
        )
        self.console.pack(fill=tk.X, padx=10, pady=(2, 6))

    # ── Port management ────────────────────────────────────────────────────

    def _refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        names = [p.device for p in ports]
        prev = self.port_var.get()
        self.port_combo["values"] = names
        if names:
            if prev in names:
                self.port_var.set(prev)
            else:
                self.port_combo.current(0)
        self._log(f"Ports refreshed: {', '.join(names) if names else 'none found'}")

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
        self.connect_btn.config(text="Disconnect", style="Danger.TButton")
        self.record_btn.config(state=tk.NORMAL)
        self.status_var.set(f"{port}  @{baud}")
        self.status_dot.config(fg=GREEN)
        self._log(f"Connected to {port} at {baud} baud")

        self.serial_thread = threading.Thread(target=self._serial_reader, daemon=True)
        self.serial_thread.start()

    def _disconnect(self):
        self.running = False
        self.last_packet_time = None
        if self.recording:
            self._toggle_recording()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.serial_port = None
        self.connect_btn.config(text="Connect", style="Accent.TButton")
        self.record_btn.config(state=tk.DISABLED)
        self.status_var.set("Disconnected")
        self.status_dot.config(fg=RED)
        self._log("Disconnected")

    # ── Serial reader thread ───────────────────────────────────────────────

    def _serial_reader(self):
        while self.running and self.serial_port and self.serial_port.is_open:
            try:
                raw = self.serial_port.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                if line.startswith("$CANSAT"):
                    data = parse_packet(line)
                    if data is None:
                        self.root.after(0, self._log, f"Bad packet: {line}")
                    else:
                        self.root.after(0, self._on_packet, data, line)
                elif line.startswith("$GSPOS"):
                    gs = parse_gspos_packet(line)
                    if gs:
                        self.root.after(0, self._on_gspos_packet, gs)
                elif line.startswith("$FREQ"):
                    info = parse_freq_packet(line)
                    if info:
                        self.root.after(0, self._on_freq_packet, *info)
                    self.root.after(0, self._log, line)
                else:
                    self.root.after(0, self._log, line)

            except serial.SerialException:
                self.root.after(0, self._disconnect)
                break
            except Exception as e:
                self.root.after(0, self._log, f"Error: {e}")

    # ── Packet handlers (main thread) ──────────────────────────────────────

    def _on_freq_packet(self, freq_mhz: float, channel: str):
        self.freq_var.set(f"{freq_mhz:.3f} MHz")
        self.chan_var.set(channel)

    def _on_gspos_packet(self, data: dict):
        lat = data["lat_e7"] / 1e7
        lon = data["lon_e7"] / 1e7
        alt = data["alt_mm"] / 1000.0
        fix = data["fix"]

        self.gs_lat   = lat
        self.gs_lon   = lon
        self.gs_alt_m = alt
        self.gs_fix   = fix

        if "gs_lat" in self.value_labels:
            self.value_labels["gs_lat"].config(text=f"{lat:.5f}")
            self.value_labels["gs_lon"].config(text=f"{lon:.5f}")
            self.value_labels["gs_alt"].config(text=f"{alt:.1f}")
            fix_str = ["No Fix", "GPS", "DGPS", "PPS", "RTK", "Float RTK",
                       "Est.", "Manual", "Sim"][fix] if fix < 9 else str(fix)
            self.value_labels["gs_fix"].config(text=fix_str)

        self._update_relative()
        self._update_gs_map(lat, lon)

    def _update_relative(self):
        """Recalculate distance and bearing from GS to CanSat."""
        if self.gs_lat is None or self.last_packet is None:
            return
        cs_lat = self.last_packet.get("lat_e7", 0) / 1e7
        cs_lon = self.last_packet.get("lon_e7", 0) / 1e7
        if cs_lat == 0.0 and cs_lon == 0.0:
            return
        dist = self._haversine(self.gs_lat, self.gs_lon, cs_lat, cs_lon)
        bear = self._bearing(self.gs_lat, self.gs_lon, cs_lat, cs_lon)
        if "gs_dist" in self.value_labels:
            self.value_labels["gs_dist"].config(text=f"{dist:.0f}")
            self.value_labels["gs_bearing"].config(text=f"{bear:.1f}")

    @staticmethod
    def _haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Great-circle distance in metres."""
        R = 6_371_000.0
        p1, p2 = math.radians(lat1), math.radians(lat2)
        dp = math.radians(lat2 - lat1)
        dl = math.radians(lon2 - lon1)
        a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    @staticmethod
    def _bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """True bearing (°, 0–360) from point 1 to point 2."""
        p1, p2 = math.radians(lat1), math.radians(lat2)
        dl = math.radians(lon2 - lon1)
        x = math.sin(dl) * math.cos(p2)
        y = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
        return (math.degrees(math.atan2(x, y)) + 360) % 360

    def _on_packet(self, data: dict, raw_line: str):
        self.packet_count += 1
        self.last_packet = data
        self.last_packet_time = time.monotonic()
        self.pkt_count_var.set(f"{self.packet_count} pkts")
        self.raw_var.set(raw_line)
        self.last_pkt_var.set(datetime.now().strftime("%H:%M:%S"))

        if data["temperature"] > -999:
            data["temperature"] += TEMP_OFFSET_C

        t = data["timestamp_ms"] / 1000.0
        self.time_data.append(t)
        for field in FIELDS:
            self.series[field].append(data[field])

        self.uptime_var.set(self._fmt_uptime(data["timestamp_ms"]))

        for field, lbl in self.value_labels.items():
            val = data.get(field)
            if val is None:
                continue
            if field in ("lat_e7", "lon_e7"):
                lbl.config(text=f"{int(val):+d}")
            elif field == "alt_mm":
                lbl.config(text=f"{val * 0.001:.1f}")
            elif field == "gas_resistance":
                lbl.config(text=f"{val * 0.001:.1f}")
            else:
                lbl.config(text=f"{val:.2f}")

        self._update_map(data)
        self._update_relative()

        if self.recording:
            now = datetime.now()
            row = dict(data)
            row["host_time"] = now.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            row["elapsed_s"] = round((now - self.record_start_time).total_seconds(), 3)
            self.recorded_rows.append(row)
            self.rec_label_var.set(f"● REC  {len(self.recorded_rows)}")

    @staticmethod
    def _fmt_uptime(ms: float) -> str:
        s = int(ms / 1000)
        h, rem = divmod(s, 3600)
        m, sec = divmod(rem, 60)
        if h:
            return f"{h:02d}:{m:02d}:{sec:02d}"
        return f"{m:02d}:{sec:02d}"

    # ── Signal status overlay ──────────────────────────────────────────────

    NO_DATA_TIMEOUT = 5  # seconds before "NO SIGNAL" warning

    def _schedule_signal_check(self):
        self._check_signal_status()
        self.root.after(500, self._schedule_signal_check)

    def _check_signal_status(self):
        if not self.running:
            self._show_overlay("NOT CONNECTED", "Select a port and click Connect",
                               icon="⚡", blink=False)
            return

        if self.last_packet_time is None:
            self._show_overlay("WAITING FOR DATA", "Connected — no packets received yet",
                               icon="📡", blink=True)
            return

        elapsed = time.monotonic() - self.last_packet_time
        if elapsed > self.NO_DATA_TIMEOUT:
            self._show_overlay(
                "NO SIGNAL",
                f"Last packet  {elapsed:.0f}s ago",
                icon="✕", blink=True,
            )
        else:
            self._hide_overlay()

    def _show_overlay(self, title: str, subtitle: str, icon: str = "⚠", blink: bool = False):
        self._overlay_blink_on = not self._overlay_blink_on
        bg = "#1a0000" if (blink and self._overlay_blink_on) else "#0d0000"
        fg_main = "#ff2222" if (blink and self._overlay_blink_on) else RED

        self.overlay.config(bg=bg)
        self.overlay_icon.config(text=icon,  bg=bg, fg=fg_main)
        self.overlay_title.config(text=title, bg=bg, fg=fg_main)
        self.overlay_sub.config(text=subtitle, bg=bg, fg="#882222" if blink and self._overlay_blink_on else "#661111")
        self.overlay.lift()
        self.overlay.place(relx=0, rely=0, relwidth=1, relheight=1)

    def _hide_overlay(self):
        self.overlay.place_forget()

    # ── Graph update ───────────────────────────────────────────────────────

    def _schedule_graph_update(self):
        self._update_graphs()
        self.root.after(REFRESH_MS, self._schedule_graph_update)

    def _update_graphs(self):
        if not self.time_data:
            return
        t = list(self.time_data)
        for idx, group in enumerate(GRAPH_GROUPS):
            ax = self.axes[idx]
            scale_map = {s[0]: s[1] for s in group.get("scale", [])}
            for field, line in self.lines[idx]:
                y = list(self.series[field])
                if field in scale_map:
                    y = [v * scale_map[field] for v in y]
                line.set_data(t[:len(y)], y)
            if t:
                ax.set_xlim(t[0], t[-1] if t[-1] > t[0] else t[0] + 1)
        self.canvas.draw_idle()

    # ── Recording ──────────────────────────────────────────────────────────

    def _toggle_recording(self):
        if self.recording:
            self.recording = False
            self.record_btn.config(text="⏺  Record", style="Record.TButton")
            self.save_btn.config(state=tk.NORMAL if self.recorded_rows else tk.DISABLED)
            self.rec_label_var.set(f"✓ {len(self.recorded_rows)} rows")
            self._log(f"Recording stopped — {len(self.recorded_rows)} rows captured")
        else:
            self.recorded_rows.clear()
            self.record_start_time = datetime.now()
            self.recording = True
            self.record_btn.config(text="⏹  Stop", style="Danger.TButton")
            self.save_btn.config(state=tk.DISABLED)
            self.rec_label_var.set("● REC  0")
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
            self._log(f"Saved {len(self.recorded_rows)} rows → {os.path.basename(filepath)}")
            messagebox.showinfo("Saved", f"Data saved to:\n{filepath}")
        except OSError as e:
            messagebox.showerror("Save Error", str(e))

    # ── Utilities ──────────────────────────────────────────────────────────

    def _clear_data(self):
        self.time_data.clear()
        for d in self.series.values():
            d.clear()
        self.packet_count = 0
        self.pkt_count_var.set("0 pkts")
        self._clear_map()
        self._log("Graphs cleared")

    def _clear_log(self):
        self.console.config(state=tk.NORMAL)
        self.console.delete("1.0", tk.END)
        self.console.config(state=tk.DISABLED)

    def _log(self, msg: str):
        self.console.config(state=tk.NORMAL)
        self.console.insert(tk.END, f"[{datetime.now().strftime('%H:%M:%S')}]  {msg}\n")
        self.console.see(tk.END)
        lines = int(self.console.index("end-1c").split(".")[0])
        if lines > 200:
            self.console.delete("1.0", "50.0")
        self.console.config(state=tk.DISABLED)

    def on_close(self):
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.root.destroy()


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    root = tk.Tk()
    app = GroundStationApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
