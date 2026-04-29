#!/usr/bin/env python3
"""
LUCARIO Iceberg Threat Assessment Tool
Gyrodos Robotics

Calculates threat levels to oil platforms and subsea assets
based on iceberg position, keel depth, and trajectory.
"""

import math
import tkinter as tk
from tkinter import ttk, messagebox
from dataclasses import dataclass
from typing import Optional
import tkinter.font as tkfont


# ═══════════════════════════════════════════════════════════════
#  CONSTANTS
# ═══════════════════════════════════════════════════════════════

NM_PER_DEG_LAT = 60.0          # 1° latitude = 60 nautical miles
EARTH_RADIUS_NM = 3440.065     # Earth radius in nautical miles
TRAJECTORY_LENGTH_NM = 50      # How far ahead to draw the iceberg path

# Threat thresholds (nautical miles)
SURFACE_GREEN  = 10.0
SURFACE_YELLOW =  5.0

SUBSEA_RADIUS  = 25.0          # nm — subsea threat zone radius

# Keel depth thresholds (% of local ocean depth)
KEEL_SAFE      = 1.10          # ≥110% → always green
KEEL_RED_HIGH  = 1.10          # 90–110% → red
KEEL_RED_LOW   = 0.90
KEEL_YEL_HIGH  = 0.90          # 70–90% → yellow
KEEL_YEL_LOW   = 0.70
                                # <70% → green


# ═══════════════════════════════════════════════════════════════
#  DATA CLASSES
# ═══════════════════════════════════════════════════════════════

@dataclass
class Platform:
    name: str
    lat: float
    lon: float
    depth_m: float              # negative = below sea level


@dataclass
class Iceberg:
    lat: float
    lon: float
    keel_m: float               # positive = depth below surface
    heading_deg: float          # 0 = North, 90 = East


@dataclass
class ThreatResult:
    platform: Platform
    surface_dist_nm: float
    closest_dist_nm: float      # closest point on trajectory
    surface_level: str          # GREEN / YELLOW / RED
    subsea_level: str
    subsea_reason: str
    trajectory_intersects: bool # does path pass within 25nm?


# ═══════════════════════════════════════════════════════════════
#  GEO MATH
# ═══════════════════════════════════════════════════════════════

def haversine_nm(lat1, lon1, lat2, lon2) -> float:
    """Great-circle distance in nautical miles."""
    r = EARTH_RADIUS_NM
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi  = math.radians(lat2 - lat1)
    dlam  = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlam/2)**2
    return 2 * r * math.asin(math.sqrt(a))


def destination(lat, lon, heading_deg, dist_nm) -> tuple[float, float]:
    """Return (lat, lon) after travelling dist_nm on heading_deg."""
    r   = EARTH_RADIUS_NM
    d   = dist_nm / r
    hdg = math.radians(heading_deg)
    phi1 = math.radians(lat)
    lam1 = math.radians(lon)
    phi2 = math.asin(math.sin(phi1)*math.cos(d) +
                     math.cos(phi1)*math.sin(d)*math.cos(hdg))
    lam2 = lam1 + math.atan2(math.sin(hdg)*math.sin(d)*math.cos(phi1),
                              math.cos(d) - math.sin(phi1)*math.sin(phi2))
    return math.degrees(phi2), math.degrees(lam2)


def point_to_segment_dist_nm(px, py, ax, ay, bx, by) -> float:
    """
    Minimum distance from point P to segment AB, all in flat (lon, lat) space.
    Works well for small areas (Grand Banks region).
    Returns distance in nautical miles.
    """
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        # Segment is a point
        return haversine_nm(py, px, ay, ax)
    t = max(0.0, min(1.0, ((px-ax)*dx + (py-ay)*dy) / (dx*dx + dy*dy)))
    cx, cy = ax + t*dx, ay + t*dy
    return haversine_nm(py, px, cy, cx)


# ═══════════════════════════════════════════════════════════════
#  THREAT LOGIC
# ═══════════════════════════════════════════════════════════════

def assess_surface_threat(dist_nm: float) -> str:
    if dist_nm > SURFACE_GREEN:
        return "GREEN"
    elif dist_nm > SURFACE_YELLOW:
        return "YELLOW"
    else:
        return "RED"


def assess_subsea_threat(iceberg: Iceberg, platform: Platform,
                         closest_nm: float) -> tuple[str, str]:
    """Returns (level, reason)."""
    # Outside 25nm → always green
    if closest_nm > SUBSEA_RADIUS:
        return "GREEN", "Iceberg trajectory outside 25 nm subsea zone"

    depth = abs(platform.depth_m)
    keel  = iceberg.keel_m
    ratio = keel / depth if depth > 0 else 0.0

    if ratio >= KEEL_SAFE:
        return "GREEN", f"Keel {keel:.0f}m ≥ 110% of seabed depth {depth:.0f}m — no contact risk"
    elif KEEL_RED_LOW <= ratio < KEEL_RED_HIGH:
        return "RED",   f"Keel {keel:.0f}m is {ratio*100:.0f}% of seabed depth {depth:.0f}m — CONTACT RISK"
    elif KEEL_YEL_LOW <= ratio < KEEL_YEL_HIGH:
        return "YELLOW", f"Keel {keel:.0f}m is {ratio*100:.0f}% of seabed depth {depth:.0f}m — margin limited"
    else:
        return "GREEN", f"Keel {keel:.0f}m < 70% of seabed depth {depth:.0f}m — safe clearance"


def evaluate(iceberg: Iceberg, platforms: list[Platform]) -> list[ThreatResult]:
    results = []

    # Iceberg trajectory end point
    end_lat, end_lon = destination(
        iceberg.lat, iceberg.lon, iceberg.heading_deg, TRAJECTORY_LENGTH_NM)

    for plat in platforms:
        # Current distance (for display)
        surf_dist = haversine_nm(iceberg.lat, iceberg.lon, plat.lat, plat.lon)

        # Closest approach along trajectory (surface threat)
        surf_closest = point_to_segment_dist_nm(
            plat.lon, plat.lat,
            iceberg.lon, iceberg.lat,
            end_lon,    end_lat)
        surf_level = assess_surface_threat(surf_closest)

        # Closest approach along trajectory (subsea threat)
        closest = point_to_segment_dist_nm(
            plat.lon, plat.lat,
            iceberg.lon, iceberg.lat,
            end_lon,    end_lat)

        intersects = closest <= SUBSEA_RADIUS
        subsea_level, subsea_reason = assess_subsea_threat(iceberg, plat, closest)

        results.append(ThreatResult(
            platform=plat,
            surface_dist_nm=surf_dist,
            closest_dist_nm=closest,
            surface_level=surf_level,
            subsea_level=subsea_level,
            subsea_reason=subsea_reason,
            trajectory_intersects=intersects,
        ))

    return results


# ═══════════════════════════════════════════════════════════════
#  COLOUR HELPERS
# ═══════════════════════════════════════════════════════════════

LEVEL_COLOUR = {
    "GREEN":  "#16a34a",
    "YELLOW": "#ca8a04",
    "RED":    "#dc2626",
}

LEVEL_BG = {
    "GREEN":  "#052e16",
    "YELLOW": "#422006",
    "RED":    "#450a0a",
}


# ═══════════════════════════════════════════════════════════════
#  MAP CANVAS
# ═══════════════════════════════════════════════════════════════

class MapCanvas(tk.Canvas):
    """Simple 2-D map of the Grand Banks region."""

    # Viewport bounds (lon, lat)
    LON_MIN, LON_MAX = -52.0, -46.0
    LAT_MIN, LAT_MAX =  43.0,  49.0

    def __init__(self, parent, **kw):
        super().__init__(parent, bg="#0a1628", highlightthickness=0, **kw)
        self._platforms: list[Platform] = []
        self._iceberg: Optional[Iceberg] = None
        self._results: list[ThreatResult] = []
        self.bind("<Configure>", lambda _: self.redraw())

    def load(self, platforms, iceberg, results):
        self._platforms = platforms
        self._iceberg   = iceberg
        self._results   = results
        self.redraw()

    def _proj(self, lat, lon) -> tuple[float, float]:
        W, H = self.winfo_width(), self.winfo_height()
        x = (lon - self.LON_MIN) / (self.LON_MAX - self.LON_MIN) * W
        y = (1 - (lat - self.LAT_MIN) / (self.LAT_MAX - self.LAT_MIN)) * H
        return x, y

    def _nm_to_px(self, nm: float) -> float:
        W = self.winfo_width()
        return nm / ((self.LON_MAX - self.LON_MIN) * 60) * W

    def redraw(self):
        self.delete("all")
        W, H = self.winfo_width(), self.winfo_height()
        if W < 2 or H < 2:
            return

        # Grid
        for lat in range(43, 50):
            x0, y0 = self._proj(lat, self.LON_MIN)
            x1, y1 = self._proj(lat, self.LON_MAX)
            self.create_line(x0, y0, x1, y1, fill="#1e3a5f", width=1)
            self.create_text(4, y0, text=f"{lat}°N",
                             fill="#334155", font=("Courier New", 7), anchor="w")

        for lon in range(-52, -45):
            x0, y0 = self._proj(self.LAT_MIN, lon)
            x1, y1 = self._proj(self.LAT_MAX, lon)
            self.create_line(x0, y0, x1, y1, fill="#1e3a5f", width=1)
            self.create_text(x0, H-4, text=f"{lon}°",
                             fill="#334155", font=("Courier New", 7), anchor="s")

        if not self._iceberg:
            return

        ice = self._iceberg



        # Iceberg trajectory
        end_lat, end_lon = destination(
            ice.lat, ice.lon, ice.heading_deg, TRAJECTORY_LENGTH_NM)
        ix0, iy0 = self._proj(ice.lat, ice.lon)
        ix1, iy1 = self._proj(end_lat, end_lon)
        self.create_line(ix0, iy0, ix1, iy1,
                         fill="#93c5fd", width=2, dash=(6, 3),
                         arrow=tk.LAST, arrowshape=(10, 12, 4))

        # Surface threat rings around iceberg
        for nm, col in [(SURFACE_GREEN, "#16a34a"), (SURFACE_YELLOW, "#ca8a04")]:
            r = self._nm_to_px(nm)
            self.create_oval(ix0-r, iy0-r, ix0+r, iy0+r,
                             outline=col, width=1, dash=(2, 4))

        # Platforms — square = surface threat, circle = subsea threat
        result_map = {r.platform.name: r for r in self._results}
        for plat in self._platforms:
            px, py = self._proj(plat.lat, plat.lon)
            res = result_map.get(plat.name)

            surf_col = LEVEL_COLOUR[res.surface_level] if res else LEVEL_COLOUR["GREEN"]
            subsea_col = LEVEL_COLOUR[res.subsea_level] if res else LEVEL_COLOUR["GREEN"]

            # Subsea zone circle — colour = subsea threat
            r = self._nm_to_px(SUBSEA_RADIUS)
            self.create_oval(px - r, py - r, px + r, py + r,
                             outline=subsea_col, width=2, dash=(4, 4))

            # Platform square — colour = surface threat
            self.create_rectangle(px - 7, py - 7, px + 7, py + 7,
                                  fill=surf_col, outline="white", width=1)

            self.create_text(px, py - 14, text=plat.name,
                             fill="white", font=("Courier New", 8, "bold"))

        # Iceberg marker
        self.create_polygon(
            ix0, iy0-12, ix0+8, iy0+8, ix0-8, iy0+8,
            fill="#bfdbfe", outline="white", width=2)
        self.create_text(ix0, iy0+18, text=f"⬡ {ice.keel_m:.0f}m keel",
                         fill="#bfdbfe", font=("Courier New", 8))


# ═══════════════════════════════════════════════════════════════
#  MAIN GUI
# ═══════════════════════════════════════════════════════════════

class App(tk.Tk):

    DEFAULTS = [
        Platform("Hibernia",  43.7504, -48.7819, -78),
        Platform("Sea Rose",  46.7895, -48.1417, -107),
        Platform("Terra Nova", 46.4,   -48.4,    -91),
        Platform("Hebron",    46.544,  -48.498,  -93),
    ]

    def __init__(self):
        super().__init__()
        self.title("Iceberg Threat Assessment — Gyrodos Robotics")
        self.configure(bg="#111827")
        self.minsize(1100, 700)
        self._apply_style()
        self._build()

    def _apply_style(self):
        s = ttk.Style(self)
        s.theme_use("clam")
        s.configure(".",
            background="#111827", foreground="white",
            fieldbackground="#1f2937", font=("Courier New", 10))
        s.configure("TLabel",  background="#111827", foreground="white")
        s.configure("TFrame",  background="#111827")
        s.configure("TLabelframe", background="#1f2937",
                    foreground="#fbbf24", bordercolor="#4b5563")
        s.configure("TLabelframe.Label", background="#1f2937",
                    foreground="#fbbf24", font=("Courier New", 10, "bold"))
        s.configure("TEntry",  fieldbackground="#374151",
                    foreground="white", insertcolor="white")
        s.configure("TButton", background="#3b82f6", foreground="white",
                    font=("Courier New", 10, "bold"), padding=6)
        s.map("TButton", background=[("active", "#2563eb")])
        s.configure("TNotebook", background="#111827", borderwidth=0)
        s.configure("TNotebook.Tab", background="#1f2937",
                    foreground="#9ca3af", padding=[10, 4],
                    font=("Courier New", 10))
        s.map("TNotebook.Tab",
              background=[("selected", "#3b82f6")],
              foreground=[("selected", "white")])

    def _build(self):
        # ── Top header ──
        hdr = tk.Frame(self, bg="#0f172a", pady=8)
        hdr.pack(fill="x")
        tk.Label(hdr, text="⬡  ICEBERG THREAT ASSESSMENT",
                 bg="#0f172a", fg="#60a5fa",
                 font=("Courier New", 15, "bold")).pack(side="left", padx=16)
        tk.Label(hdr, text="Gyrodos Robotics — Ocean Conservation Suite",
                 bg="#0f172a", fg="#475569",
                 font=("Courier New", 10)).pack(side="right", padx=16)

        # ── Main layout ──
        main = tk.Frame(self, bg="#111827")
        main.pack(fill="both", expand=True, padx=10, pady=8)

        # Left panel
        left = tk.Frame(main, bg="#111827", width=360)
        left.pack(side="left", fill="y", padx=(0, 8))
        left.pack_propagate(False)

        self._build_platforms(left)
        self._build_iceberg(left)

        run_btn = ttk.Button(left, text="▶  CALCULATE THREAT",
                             command=self._run)
        run_btn.pack(fill="x", pady=8, padx=4)

        # Right panel — notebook
        right = tk.Frame(main, bg="#111827")
        right.pack(side="left", fill="both", expand=True)

        nb = ttk.Notebook(right)
        nb.pack(fill="both", expand=True)

        # Map tab
        map_frame = ttk.Frame(nb)
        nb.add(map_frame, text="  🗺  MAP  ")
        self.map_canvas = MapCanvas(map_frame)
        self.map_canvas.pack(fill="both", expand=True, padx=4, pady=4)

        # Results tab
        res_frame = ttk.Frame(nb)
        nb.add(res_frame, text="  📋  RESULTS  ")
        self._build_results_panel(res_frame)

        # Draw default platforms
        self.map_canvas.load(self.DEFAULTS, None, [])

    # ── Platform inputs ──────────────────────────────────────────

    def _build_platforms(self, parent):
        lf = ttk.LabelFrame(parent, text="  OIL PLATFORMS", padding=8)
        lf.pack(fill="x", pady=(0, 6), padx=4)

        headers = ["Name", "Lat", "Lon", "Depth(m)"]
        for c, h in enumerate(headers):
            tk.Label(lf, text=h, fg="#9ca3af", bg="#1f2937",
                     font=("Courier New", 8)).grid(row=0, column=c, padx=3)

        self._plat_rows: list[dict] = []
        for i, p in enumerate(self.DEFAULTS):
            row = {}
            for c, (key, val) in enumerate([
                ("name",  p.name),
                ("lat",   p.lat),
                ("lon",   p.lon),
                ("depth", p.depth_m),
            ]):
                e = ttk.Entry(lf, width=9)
                e.insert(0, str(val))
                e.grid(row=i+1, column=c, padx=2, pady=2)
                row[key] = e
            self._plat_rows.append(row)

        # Add / Remove buttons
        btn_row = tk.Frame(lf, bg="#1f2937")
        btn_row.grid(row=len(self.DEFAULTS)+1, column=0,
                     columnspan=4, pady=(6, 0), sticky="ew")
        ttk.Button(btn_row, text="+ Add",
                   command=self._add_platform).pack(side="left", padx=2)
        ttk.Button(btn_row, text="− Remove",
                   command=self._remove_platform).pack(side="left", padx=2)

    def _add_platform(self):
        lf = self._plat_rows[0]["name"].master   # get the LabelFrame
        i = len(self._plat_rows) + 1
        row = {}
        defaults = ["New", "46.0", "-48.0", "-100"]
        for c, (key, val) in enumerate(zip(
                ["name","lat","lon","depth"], defaults)):
            e = ttk.Entry(lf, width=9)
            e.insert(0, val)
            e.grid(row=i, column=c, padx=2, pady=2)
            row[key] = e
        self._plat_rows.append(row)

    def _remove_platform(self):
        if len(self._plat_rows) > 1:
            row = self._plat_rows.pop()
            for e in row.values():
                e.destroy()

    # ── Iceberg inputs ───────────────────────────────────────────

    def _build_iceberg(self, parent):
        lf = ttk.LabelFrame(parent, text="  ICEBERG", padding=8)
        lf.pack(fill="x", pady=(0, 6), padx=4)

        fields = [
            ("Latitude",        "lat",     "46.0"),
            ("Longitude",       "lon",     "-49.5"),
            ("Keel Depth (m)",  "keel",    "85"),
            ("Heading (°)",     "heading", "135"),
        ]
        self._ice_vars: dict[str, ttk.Entry] = {}
        for r, (label, key, default) in enumerate(fields):
            tk.Label(lf, text=label, bg="#1f2937", fg="#d1d5db",
                     font=("Courier New", 9), width=16,
                     anchor="w").grid(row=r, column=0, sticky="w", pady=2)
            e = ttk.Entry(lf, width=14)
            e.insert(0, default)
            e.grid(row=r, column=1, padx=6, pady=2)
            self._ice_vars[key] = e

    # ── Results panel ────────────────────────────────────────────

    def _build_results_panel(self, parent):
        self._res_frame = tk.Frame(parent, bg="#111827")
        self._res_frame.pack(fill="both", expand=True, padx=4, pady=4)
        tk.Label(self._res_frame,
                 text="Run a calculation to see results.",
                 bg="#111827", fg="#4b5563",
                 font=("Courier New", 11)).pack(pady=40)

    def _populate_results(self, results: list[ThreatResult], ice: Iceberg):
        for w in self._res_frame.winfo_children():
            w.destroy()

        # Summary header
        worst_surf = max(results, key=lambda r: ["GREEN","YELLOW","RED"].index(r.surface_level))
        worst_sub  = max(results, key=lambda r: ["GREEN","YELLOW","RED"].index(r.subsea_level))
        overall_idx = max(
            ["GREEN","YELLOW","RED"].index(worst_surf.surface_level),
            ["GREEN","YELLOW","RED"].index(worst_sub.subsea_level))
        overall = ["GREEN","YELLOW","RED"][overall_idx]

        hdr = tk.Frame(self._res_frame,
                       bg=LEVEL_BG[overall], pady=8)
        hdr.pack(fill="x", pady=(0, 8))
        tk.Label(hdr,
                 text=f"OVERALL THREAT:  {overall}",
                 bg=LEVEL_BG[overall], fg=LEVEL_COLOUR[overall],
                 font=("Courier New", 14, "bold")).pack()
        tk.Label(hdr,
                 text=f"Iceberg @ {ice.lat:.4f}°N  {ice.lon:.4f}°E  "
                      f"| Keel {ice.keel_m:.0f}m  | Hdg {ice.heading_deg:.0f}°",
                 bg=LEVEL_BG[overall], fg="#9ca3af",
                 font=("Courier New", 9)).pack()

        # Scrollable results
        canvas = tk.Canvas(self._res_frame, bg="#111827",
                           highlightthickness=0)
        sb = ttk.Scrollbar(self._res_frame, orient="vertical",
                           command=canvas.yview)
        canvas.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)

        inner = tk.Frame(canvas, bg="#111827")
        canvas.create_window((0, 0), window=inner, anchor="nw")
        inner.bind("<Configure>",
                   lambda e: canvas.configure(
                       scrollregion=canvas.bbox("all")))

        for res in results:
            self._result_card(inner, res)

    def _result_card(self, parent, res: ThreatResult):
        levels = ["GREEN", "YELLOW", "RED"]
        worst_idx = max(levels.index(res.surface_level),
                        levels.index(res.subsea_level))
        worst = levels[worst_idx]

        card = tk.Frame(parent, bg="#1f2937",
                        highlightbackground=LEVEL_COLOUR[worst],
                        highlightthickness=2)
        card.pack(fill="x", pady=4, padx=2)

        # Title row
        title = tk.Frame(card, bg=LEVEL_BG[worst])
        title.pack(fill="x")
        tk.Label(title, text=f"  {res.platform.name}",
                 bg=LEVEL_BG[worst], fg="white",
                 font=("Courier New", 11, "bold")).pack(side="left", pady=4)
        tk.Label(title,
                 text=f"{res.platform.lat:.4f}°N  {res.platform.lon:.4f}°E  "
                      f"depth {abs(res.platform.depth_m):.0f}m  ",
                 bg=LEVEL_BG[worst], fg="#9ca3af",
                 font=("Courier New", 8)).pack(side="right", pady=4)

        # Metrics grid
        grid = tk.Frame(card, bg="#1f2937")
        grid.pack(fill="x", padx=8, pady=6)

        def metric(row, col, label, value, level=None):
            tk.Label(grid, text=label, bg="#1f2937", fg="#6b7280",
                     font=("Courier New", 8)).grid(
                         row=row*2, column=col, sticky="w", padx=6)
            col_val = LEVEL_COLOUR[level] if level else "white"
            tk.Label(grid, text=value, bg="#1f2937", fg=col_val,
                     font=("Courier New", 11, "bold")).grid(
                         row=row*2+1, column=col, sticky="w", padx=6)

        metric(0, 0, "SURFACE DISTANCE (CLOSEST)",
               f"{res.closest_dist_nm:.2f} nm", res.surface_level)
        metric(0, 1, "SURFACE THREAT", res.surface_level, res.surface_level)
        metric(0, 2, "CLOSEST APPROACH",
               f"{res.closest_dist_nm:.2f} nm")
        metric(1, 0, "SUBSEA THREAT", res.subsea_level, res.subsea_level)
        metric(1, 1, "TRAJ. IN ZONE",
               "YES" if res.trajectory_intersects else "NO",
               "RED" if res.trajectory_intersects else "GREEN")

        # Reason
        tk.Label(card, text=f"  ℹ  {res.subsea_reason}",
                 bg="#1f2937", fg="#9ca3af",
                 font=("Courier New", 8),
                 anchor="w").pack(fill="x", padx=8, pady=(0, 6))

    # ── Run ──────────────────────────────────────────────────────

    def _run(self):
        # Parse platforms
        platforms = []
        for row in self._plat_rows:
            try:
                platforms.append(Platform(
                    name  = row["name"].get().strip(),
                    lat   = float(row["lat"].get()),
                    lon   = float(row["lon"].get()),
                    depth_m = float(row["depth"].get()),
                ))
            except ValueError:
                messagebox.showerror("Input Error",
                    f"Invalid platform data in row: {row['name'].get()}")
                return

        # Parse iceberg
        try:
            ice = Iceberg(
                lat         = float(self._ice_vars["lat"].get()),
                lon         = float(self._ice_vars["lon"].get()),
                keel_m      = float(self._ice_vars["keel"].get()),
                heading_deg = float(self._ice_vars["heading"].get()),
            )
        except ValueError:
            messagebox.showerror("Input Error", "Invalid iceberg data.")
            return

        results = evaluate(ice, platforms)
        self.map_canvas.load(platforms, ice, results)
        self._populate_results(results, ice)


# ═══════════════════════════════════════════════════════════════
#  ENTRY
# ═══════════════════════════════════════════════════════════════

if __name__ == "__main__":
    app = App()
    app.mainloop()
