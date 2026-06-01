"""
ABU Robocon 2026 — PreprocessConfig Interactive Tuner
======================================================
Visualizes gamefield.pcd with the new simplified preprocessing parameters.
Sliders let you tune all PreprocessConfig values in real-time.

Views:
  Left  : Top-down  (X forward vs Y lateral)
  Right : Side view (X forward vs Z up)

Point colours:
  Gray  — outside range filter
  Green — passes range filter (above ground after RANSAC)

Usage:
  python3 tune_config.py
  python3 tune_config.py /path/to/gamefield.pcd
"""

import sys
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import matplotlib.widgets as mwidgets

PCD_PATH = "/home/utmrbc/Demo4/src/r2/gamefield.pcd"

DEFAULTS = dict(
    VOXEL_SIZE        = 0.03,
    MAX_RANGE         = 12.0,
    Z_MIN             = -1.5,
    Z_MAX             = 2.0,
    RANSAC_DIST_THRESH= 0.03,
    GROUND_Z_MARGIN   = 0.05,
    SOR_K_NEIGHBORS   = 20,
    SOR_STD_RATIO     = 1.5,
)

RANGES = dict(
    VOXEL_SIZE        = (0.01, 0.15, 0.005),
    MAX_RANGE         = (2.0,  15.0, 0.5),
    Z_MIN             = (-2.0,  0.0, 0.05),
    Z_MAX             = (0.5,   3.0, 0.05),
    RANSAC_DIST_THRESH= (0.01, 0.15, 0.005),
    GROUND_Z_MARGIN   = (0.01, 0.20, 0.005),
    SOR_K_NEIGHBORS   = (5,    50,   1),
    SOR_STD_RATIO     = (0.5,  3.0,  0.1),
)

SUBSAMPLE = 40_000


def load_pcd(path: str) -> np.ndarray:
    raw = o3d.io.read_point_cloud(path)
    if not raw.has_points():
        raise RuntimeError(f"Empty or missing PCD: {path}")
    cad = np.asarray(raw.points)
    ros = np.zeros_like(cad)
    ros[:, 0] = cad[:, 0] * 0.001
    ros[:, 1] = cad[:, 2] * 0.001
    ros[:, 2] = (2470.0 - cad[:, 1]) * 0.001
    print(f"Loaded {len(ros):,} pts  |  "
          f"X {ros[:,0].min():.2f}–{ros[:,0].max():.2f}  "
          f"Y {ros[:,1].min():.2f}–{ros[:,1].max():.2f}  "
          f"Z {ros[:,2].min():.2f}–{ros[:,2].max():.2f}")
    return ros


def subsample(pts: np.ndarray, n: int) -> np.ndarray:
    if len(pts) <= n:
        return pts
    idx = np.random.default_rng(0).choice(len(pts), n, replace=False)
    return pts[idx]


def apply_filters(pts: np.ndarray, cfg: dict):
    """Apply range filter only (RANSAC not applied to static PCD for tuner)."""
    xy_dist = np.linalg.norm(pts[:, :2], axis=1)
    mask = (
        (xy_dist <= cfg["MAX_RANGE"]) &
        (pts[:, 2] >= cfg["Z_MIN"]) &
        (pts[:, 2] <= cfg["Z_MAX"])
    )
    return mask


class Tuner:
    def __init__(self, pts_full: np.ndarray):
        self.pts = subsample(pts_full, SUBSAMPLE)
        self.cfg = dict(DEFAULTS)

        self.fig = plt.figure("PreprocessConfig Tuner", figsize=(16, 9))
        self.fig.patch.set_facecolor("#1e1e1e")

        self.ax_top  = self.fig.add_axes([0.03, 0.45, 0.44, 0.50])
        self.ax_side = self.fig.add_axes([0.53, 0.45, 0.44, 0.50])

        for ax in (self.ax_top, self.ax_side):
            ax.set_facecolor("#2b2b2b")
            ax.tick_params(colors="white")
            for spine in ax.spines.values():
                spine.set_edgecolor("#555")

        self.ax_top.set_xlabel("X  forward (m)", color="white")
        self.ax_top.set_ylabel("Y  lateral (m)", color="white")
        self.ax_top.set_title("Top-down  (X–Y)", color="white")
        self.ax_side.set_xlabel("X  forward (m)", color="white")
        self.ax_side.set_ylabel("Z  up (m)",      color="white")
        self.ax_side.set_title("Side view  (X–Z)", color="white")

        self.sc_top_out  = self.ax_top.scatter([], [], s=0.4, c="#555555", rasterized=True)
        self.sc_top_in   = self.ax_top.scatter([], [], s=0.6, c="#33ff77", rasterized=True)
        self.sc_side_out = self.ax_side.scatter([], [], s=0.4, c="#555555", rasterized=True)
        self.sc_side_in  = self.ax_side.scatter([], [], s=0.6, c="#33ff77", rasterized=True)

        # Range circle on top-down view
        theta = np.linspace(0, 2 * np.pi, 200)
        self.range_circle, = self.ax_top.plot([], [], color="#ffaa00", lw=1.2)
        self.z_line_lo,    = self.ax_side.plot([], [], color="#ffaa00", lw=1.0, linestyle="--")
        self.z_line_hi,    = self.ax_side.plot([], [], color="#ffaa00", lw=1.0, linestyle="--")
        self._theta = theta

        # Sliders
        slider_keys = list(DEFAULTS.keys())
        n = len(slider_keys)
        cols, rows = 2, (n + 1) // 2
        self.sliders = {}

        for i, key in enumerate(slider_keys):
            col = i % cols
            row = i // cols
            left   = 0.05 + col * 0.50
            bottom = 0.36 - row * 0.042
            ax_sl  = self.fig.add_axes([left, bottom, 0.38, 0.025], facecolor="#3a3a3a")
            lo, hi, step = RANGES[key]
            sl = mwidgets.Slider(
                ax_sl, key.replace("_", " "),
                lo, hi,
                valinit=DEFAULTS[key],
                valstep=step,
                color="#4a9eff",
            )
            sl.label.set_color("white")
            sl.valtext.set_color("#ffdd88")
            sl.on_changed(lambda val, k=key: self._on_change(k, val))
            self.sliders[key] = sl

        self.stats_ax = self.fig.add_axes([0.03, 0.01, 0.94, 0.03])
        self.stats_ax.axis("off")
        self.stats_txt = self.stats_ax.text(
            0.5, 0.5, "", ha="center", va="center",
            color="#ffdd88", fontsize=9, transform=self.stats_ax.transAxes
        )

        self.fig.canvas.mpl_connect("close_event", self._on_close)
        self._refresh()
        plt.show()

    def _on_change(self, key, val):
        self.cfg[key] = val
        self._refresh()

    def _refresh(self):
        mask = apply_filters(self.pts, self.cfg)
        kept = self.pts[mask]
        out  = self.pts[~mask]

        self.sc_top_out.set_offsets(out[:, :2]   if len(out)  else np.empty((0, 2)))
        self.sc_top_in .set_offsets(kept[:, :2]  if len(kept) else np.empty((0, 2)))
        self.sc_side_out.set_offsets(out[:, [0, 2]]  if len(out)  else np.empty((0, 2)))
        self.sc_side_in .set_offsets(kept[:, [0, 2]] if len(kept) else np.empty((0, 2)))

        r = self.cfg["MAX_RANGE"]
        self.range_circle.set_data(r * np.cos(self._theta), r * np.sin(self._theta))

        xlim = self.ax_side.get_xlim()
        self.z_line_lo.set_data(xlim, [self.cfg["Z_MIN"]] * 2)
        self.z_line_hi.set_data(xlim, [self.cfg["Z_MAX"]] * 2)

        for ax in (self.ax_top, self.ax_side):
            ax.relim()
            ax.autoscale_view()

        pct = 100 * mask.sum() / max(len(mask), 1)
        self.stats_txt.set_text(
            f"Showing {len(self.pts):,} pts (subsampled)  |  "
            f"Kept: {mask.sum():,}  ({pct:.1f}%)  |  "
            f"MAX_RANGE={self.cfg['MAX_RANGE']:.1f}m   "
            f"Z [{self.cfg['Z_MIN']:.2f}, {self.cfg['Z_MAX']:.2f}]m   "
            f"VOXEL={self.cfg['VOXEL_SIZE']*100:.0f}cm"
        )
        self.fig.canvas.draw_idle()

    def _on_close(self, _):
        self._print_config()

    def _print_config(self):
        c = self.cfg
        print("\n" + "=" * 60)
        print("  Final PreprocessConfig — paste into lidar_preprocessing.py")
        print("=" * 60)
        print(f"    VOXEL_SIZE               = {c['VOXEL_SIZE']:.3f}")
        print(f"    MAX_RANGE                = {c['MAX_RANGE']:.1f}")
        print(f"    Z_MIN                    = {c['Z_MIN']:.3f}")
        print(f"    Z_MAX                    = {c['Z_MAX']:.3f}")
        print(f"    RANSAC_DISTANCE_THRESHOLD= {c['RANSAC_DIST_THRESH']:.3f}")
        print(f"    GROUND_Z_MARGIN          = {c['GROUND_Z_MARGIN']:.3f}")
        print(f"    SOR_K_NEIGHBORS          = {int(c['SOR_K_NEIGHBORS'])}")
        print(f"    SOR_STD_RATIO            = {c['SOR_STD_RATIO']:.1f}")
        print("=" * 60 + "\n")


if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else PCD_PATH
    pts = load_pcd(path)
    Tuner(pts)
