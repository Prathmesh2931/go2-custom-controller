#!/usr/bin/env python3
"""
go2_gait_dashboard.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Go2 Gait Live Dashboard
  • Subscribes to /joint_states  → shows real-time joint angles per leg
  • Subscribes to a phase topic  → shows current phase + tick counter
  • Draws live swing trajectory  → X/Z plot of the active swing foot
  • Phase timeline bar           → which phase is active and for how long
  • Tuning sliders               → change SWING_H, STEP_LEN, PUSH_DIST live
                                   and publishes them on /gait_params
  • Log pane                     → paste your rosbag/terminal logs here
                                   and click "Analyse" for auto tuning hints

RUN:
  # With a live ROS2 system:
  python3 go2_gait_dashboard.py

  # Without ROS2 (simulation / demo mode):
  python3 go2_gait_dashboard.py --demo

DEPENDENCIES:
  pip install matplotlib numpy
  (ROS2 rclpy + sensor_msgs only needed in live mode)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import sys
import math
import time
import threading
import argparse
import collections
import textwrap

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
from matplotlib.widgets import Slider, Button, TextBox
from matplotlib.lines import Line2D

# ─────────────────────────────────────────────────────────────────────────────
#  Robot constants  (mirror your C++ header)
# ─────────────────────────────────────────────────────────────────────────────
L0 = 0.0955
L1 = 0.213
L2 = 0.213

LEG_NAMES  = ["LF", "RF", "LH", "RH"]
LEG_COLORS = ["#3B8BD4", "#E24B4A", "#1D9E75", "#EF9F27"]
PHASE_NAMES  = ["SHIFT_WEIGHT", "SWING", "PLACE", "RECENTER"]
PHASE_COLORS = ["#7F77DD", "#3B8BD4", "#1D9E75", "#EF9F27"]

# ─────────────────────────────────────────────────────────────────────────────
#  Inverse kinematics  (Python mirror of your C++ ik())
# ─────────────────────────────────────────────────────────────────────────────
def ik(x, y, z):
    hip   = math.atan2(y, -z) - math.atan2(L0, math.sqrt(max(0.0, y*y + z*z - L0*L0)))
    z2d   = math.sqrt(max(0.0, y*y + z*z - L0*L0))
    D_raw = (x*x + z2d*z2d - L1*L1 - L2*L2) / (2.0*L1*L2)
    D     = max(-1.0, min(1.0, D_raw))
    calf  = -math.acos(D)
    thigh = math.atan2(x, z2d) - math.atan2(L2*math.sin(calf), L1 + L2*math.cos(calf))
    return hip, thigh, calf, D_raw   # D_raw exposed for singularity monitoring

def fk_foot_xz(thigh, calf, z2d_ref=None):
    """Forward kinematics: returns (x, z_depth) in sagittal plane."""
    # used only for trajectory visualisation
    return None, None


# ─────────────────────────────────────────────────────────────────────────────
#  Trajectory generator  (matches phase_gen.cpp)
# ─────────────────────────────────────────────────────────────────────────────
def swing_traj(t, step_len, swing_h, stance_h):
    """t in [0,1].  Returns (x, z) in body frame."""
    x = step_len * (1.0 - math.cos(math.pi * t)) / 2.0
    z = -stance_h + swing_h * math.sin(math.pi * t)
    return x, z


def full_trajectory_points(step_len, swing_h, stance_h, n=200):
    ts = np.linspace(0, 1, n)
    xs = [swing_traj(t, step_len, swing_h, stance_h)[0] for t in ts]
    zs = [swing_traj(t, step_len, swing_h, stance_h)[1] for t in ts]
    return np.array(xs), np.array(zs)


def singularity_map(step_len, swing_h, stance_h, n=200):
    """Returns D_raw values along the swing arc – reveals singularity risk."""
    ts = np.linspace(0, 1, n)
    D_vals = []
    for t in ts:
        x, z = swing_traj(t, step_len, swing_h, stance_h)
        _, _, _, D_raw = ik(x, L0, z)
        D_vals.append(D_raw)
    return np.array(ts), np.array(D_vals)


# ─────────────────────────────────────────────────────────────────────────────
#  Simulated ROS data  (used in --demo mode)
# ─────────────────────────────────────────────────────────────────────────────
class DemoSimulator:
    """
    Runs the same state machine as phase_gen.cpp in Python so the dashboard
    works without a real robot.
    """
    PHASE_TICKS = [25, 60, 15, 25]   # SHIFT, SWING, PLACE, RECENTER

    def __init__(self):
        self.phase      = 0   # 0-3
        self.tick       = 0
        self.swing_leg  = 0
        self.t_start    = time.time()

        # Tunable params (overwritten by sliders)
        self.swing_h    = 0.09
        self.step_len   = 0.10
        self.push_dist  = 0.08
        self.stance_h   = 0.32
        self.shift_z    = 0.02

        # State outputs
        self.joint_angles = {leg: [0.0, 0.7, -1.4] for leg in LEG_NAMES}
        self.foot_xz      = {leg: (0.0, -self.stance_h) for leg in LEG_NAMES}
        self.phase_name   = "SHIFT_WEIGHT"
        self.phase_prog   = 0.0
        self.d_raw_swing  = 0.0
        self.swing_foot_trail_x = collections.deque(maxlen=200)
        self.swing_foot_trail_z = collections.deque(maxlen=200)

    def step(self):
        Y = [L0, -L0, L0, -L0]
        t_norm = self.tick / self.PHASE_TICKS[self.phase] if self.PHASE_TICKS[self.phase] > 0 else 1.0

        self.phase_name = PHASE_NAMES[self.phase]
        self.phase_prog = t_norm

        for i, leg in enumerate(LEG_NAMES):
            if self.phase == 0:   # SHIFT_WEIGHT
                z = -self.stance_h if i == self.swing_leg else -self.stance_h - self.shift_z * t_norm
                hip, thigh, calf, _ = ik(0.0, Y[i], z)
            elif self.phase == 1:  # SWING
                if i == self.swing_leg:
                    x, z = swing_traj(t_norm, self.step_len, self.swing_h, self.stance_h)
                    hip, thigh, calf, D_raw = ik(x, Y[i], z)
                    self.d_raw_swing = D_raw
                    self.swing_foot_trail_x.append(x)
                    self.swing_foot_trail_z.append(z)
                    self.foot_xz[leg] = (x, z)
                else:
                    x = -self.push_dist * t_norm
                    z = -self.stance_h - self.shift_z
                    hip, thigh, calf, _ = ik(x, Y[i], z)
                    self.foot_xz[leg] = (x, z)
            elif self.phase == 2:  # PLACE
                x = self.step_len if i == self.swing_leg else -self.push_dist
                z = -self.stance_h if i == self.swing_leg else -self.stance_h - self.shift_z
                hip, thigh, calf, _ = ik(x, Y[i], z)
                self.foot_xz[leg] = (x, z)
            else:                  # RECENTER
                x_start = self.step_len if i == self.swing_leg else -self.push_dist
                x = x_start * (1.0 - t_norm)
                z = -self.stance_h - self.shift_z * (1.0 - t_norm)
                hip, thigh, calf, _ = ik(x, Y[i], z)
                self.foot_xz[leg] = (x, z)

            self.joint_angles[leg] = [
                math.degrees(hip),
                math.degrees(thigh),
                math.degrees(calf)
            ]

        self.tick += 1
        if self.tick >= self.PHASE_TICKS[self.phase]:
            self.tick = 0
            self.phase = (self.phase + 1) % 4
            if self.phase == 0:
                self.swing_foot_trail_x.clear()
                self.swing_foot_trail_z.clear()
                self.swing_leg = (self.swing_leg + 1) % 4

    def run(self):
        while True:
            self.step()
            time.sleep(0.02)


# ─────────────────────────────────────────────────────────────────────────────
#  Log analyser
# ─────────────────────────────────────────────────────────────────────────────
def analyse_logs(log_text, swing_h, step_len, push_dist, stance_h):
    """
    Parse pasted terminal/rosbag log text and return a list of tuning hints.
    Looks for:
      - "IK near singularity" warnings
      - phase transition timings
      - repeated RECENTER messages (might mean it's looping too fast)
    """
    hints = []
    lines = log_text.strip().split("\n")

    singularity_count = sum(1 for l in lines if "singularity" in l.lower())
    if singularity_count > 0:
        hints.append(
            f"⚠  {singularity_count} singularity warning(s) found.\n"
            f"   → Reduce SWING_H (currently {swing_h:.3f} m) by ~0.01 m.\n"
            f"   → Or increase STANCE_H (currently {stance_h:.3f} m) by ~0.01 m."
        )

    swing_lines = [l for l in lines if "→ SWING" in l]
    place_lines = [l for l in lines if "→ PLACE" in l]
    if len(swing_lines) > 2 and len(place_lines) > 2:
        hints.append(
            f"✓  Phase transitions detected: {len(swing_lines)} SWING entries.\n"
            f"   Cycle looks healthy."
        )

    if any("warn" in l.lower() for l in lines) and singularity_count == 0:
        hints.append(
            "ℹ  Warnings present (non-singularity).\n"
            "   Check if joint limits or torque saturation is mentioned."
        )

    if not hints:
        hints.append("✓  No obvious issues found in the log.\n"
                     "   If the robot still looks wrong, share the full log.")

    # Suggest D-raw threshold check
    d_vals_at_peak = []
    for t in [0.3, 0.5, 0.7]:
        x, z = swing_traj(t, step_len, swing_h, stance_h)
        _, _, _, D_raw = ik(x, L0, z)
        d_vals_at_peak.append(abs(D_raw))
    max_d = max(d_vals_at_peak)
    if max_d > 0.90:
        hints.append(
            f"⚠  Computed D_raw peak = {max_d:.3f} with current params.\n"
            f"   Singularity risk is HIGH.  Reduce SWING_H or STEP_LEN."
        )
    else:
        hints.append(
            f"✓  D_raw peak = {max_d:.3f} – safely away from singularity (< 0.90)."
        )

    return hints


# ─────────────────────────────────────────────────────────────────────────────
#  Dashboard
# ─────────────────────────────────────────────────────────────────────────────
class GaitDashboard:
    HISTORY = 300   # samples to keep in time-series plots

    def __init__(self, demo_mode=False):
        self.demo_mode = demo_mode
        self.sim = DemoSimulator()

        # ── Tunable params (shared between sliders and sim) ────────────────
        self.swing_h   = 0.09
        self.step_len  = 0.10
        self.push_dist = 0.08
        self.stance_h  = 0.32

        # ── History buffers for time-series ───────────────────────────────
        self.ts_time    = collections.deque(maxlen=self.HISTORY)
        self.ts_angles  = {leg: {j: collections.deque(maxlen=self.HISTORY)
                                 for j in ["hip","thigh","calf"]}
                           for leg in LEG_NAMES}
        self.ts_phase   = collections.deque(maxlen=self.HISTORY)
        self.ts_d_raw   = collections.deque(maxlen=self.HISTORY)
        self.t0         = time.time()

        self._build_figure()

        if demo_mode:
            sim_thread = threading.Thread(target=self.sim.run, daemon=True)
            sim_thread.start()
        else:
            self._init_ros()

    # ── Figure layout ──────────────────────────────────────────────────────
    def _build_figure(self):
        matplotlib.rcParams.update({
            "figure.facecolor": "#0f0f0e",
            "axes.facecolor":   "#1a1a18",
            "axes.edgecolor":   "#3a3a38",
            "axes.labelcolor":  "#c2c0b6",
            "xtick.color":      "#888780",
            "ytick.color":      "#888780",
            "text.color":       "#c2c0b6",
            "grid.color":       "#2e2e2c",
            "grid.linewidth":   0.5,
            "lines.linewidth":  1.5,
            "font.family":      "monospace",
            "font.size":        9,
        })

        self.fig = plt.figure(figsize=(18, 11))
        self.fig.patch.set_facecolor("#0f0f0e")
        self.fig.canvas.manager.set_window_title("Go2 Gait Dashboard")

        gs = gridspec.GridSpec(
            4, 4,
            left=0.06, right=0.97, top=0.93, bottom=0.22,
            hspace=0.55, wspace=0.45
        )

        # Row 0: trajectory + phase bar + D_raw
        self.ax_traj   = self.fig.add_subplot(gs[0:2, 0:2])
        self.ax_phase  = self.fig.add_subplot(gs[0,   2:4])
        self.ax_d      = self.fig.add_subplot(gs[1,   2:4])

        # Row 2-3: one axis per leg  (joint angles over time)
        self.ax_legs = []
        for i, leg in enumerate(LEG_NAMES):
            ax = self.fig.add_subplot(gs[2 + i//2, 2*(i%2) : 2*(i%2)+2])
            ax.set_title(f"{leg}  joint angles (°)", fontsize=8, color=LEG_COLORS[i], pad=3)
            ax.set_xlim(0, self.HISTORY)
            ax.set_ylim(-120, 120)
            ax.grid(True)
            ax.axhline(0, color="#3a3a38", linewidth=0.5)
            self.ax_legs.append(ax)

        # ── Trajectory axis setup ──────────────────────────────────────────
        self.ax_traj.set_title("Swing foot trajectory (body frame)", fontsize=9, pad=4)
        self.ax_traj.set_xlabel("x  forward (m)")
        self.ax_traj.set_ylabel("z  depth (m)")
        self.ax_traj.set_xlim(-0.04, 0.22)
        self.ax_traj.set_ylim(-0.40, -0.18)
        self.ax_traj.grid(True)
        self.ax_traj.invert_yaxis()
        # ground line
        self.ax_traj.axhline(-self.stance_h, color="#1D9E75", linewidth=1,
                              linestyle="--", label="ground (stance_h)")
        # planned trajectory
        xs, zs = full_trajectory_points(self.step_len, self.swing_h, self.stance_h)
        self.line_planned, = self.ax_traj.plot(xs, zs, color="#3a3a38",
                                               linewidth=2, label="planned arc")
        # live trail
        self.line_trail,   = self.ax_traj.plot([], [], color=LEG_COLORS[0],
                                               linewidth=2.5, label="live foot path")
        self.dot_foot,     = self.ax_traj.plot([], [], "o", color=LEG_COLORS[0],
                                               markersize=8)
        # peak annotation
        peak_x, peak_z = swing_traj(0.5, self.step_len, self.swing_h, self.stance_h)
        self.annot_peak = self.ax_traj.annotate(
            f"peak\n{self.swing_h*100:.0f} cm",
            xy=(peak_x, peak_z), xytext=(peak_x + 0.02, peak_z + 0.02),
            arrowprops=dict(arrowstyle="->", color="#EF9F27"),
            color="#EF9F27", fontsize=8
        )
        self.ax_traj.legend(fontsize=7, loc="lower right",
                            facecolor="#1a1a18", edgecolor="#3a3a38")

        # ── Phase bar ─────────────────────────────────────────────────────
        self.ax_phase.set_title("Phase timeline", fontsize=9, pad=4)
        self.ax_phase.set_xlim(0, self.HISTORY)
        self.ax_phase.set_ylim(-0.5, 3.5)
        self.ax_phase.set_yticks([0,1,2,3])
        self.ax_phase.set_yticklabels(PHASE_NAMES, fontsize=7)
        self.ax_phase.grid(True, axis="x")
        self.scat_phase = self.ax_phase.scatter([], [], c=[], s=6,
                                                cmap="tab10", vmin=0, vmax=3)
        # current phase label
        self.phase_text = self.ax_phase.text(
            0.98, 0.92, "SHIFT_WEIGHT", transform=self.ax_phase.transAxes,
            ha="right", va="top", fontsize=10, color="#7F77DD", fontweight="bold"
        )

        # ── D_raw (singularity monitor) ────────────────────────────────────
        self.ax_d.set_title("IK D_raw  (swing leg)  — danger > 0.95", fontsize=9, pad=4)
        self.ax_d.set_xlim(0, self.HISTORY)
        self.ax_d.set_ylim(-1.05, 1.05)
        self.ax_d.axhline(0.95,  color="#E24B4A", linewidth=1, linestyle="--")
        self.ax_d.axhline(-0.95, color="#E24B4A", linewidth=1, linestyle="--")
        self.ax_d.axhline(0, color="#3a3a38", linewidth=0.5)
        self.ax_d.grid(True)
        self.line_d,     = self.ax_d.plot([], [], color="#EF9F27", linewidth=1.5)
        self.line_d_fill = None   # filled area added dynamically

        # ── Leg angle lines ────────────────────────────────────────────────
        joint_colors = ["#c2c0b6", "#7F77DD", "#E24B4A"]  # hip, thigh, calf
        self.leg_lines = {}
        for i, leg in enumerate(LEG_NAMES):
            self.leg_lines[leg] = {}
            for j, jname in enumerate(["hip", "thigh", "calf"]):
                ln, = self.ax_legs[i].plot([], [], color=joint_colors[j],
                                           label=jname, linewidth=1.2)
                self.leg_lines[leg][jname] = ln
            self.ax_legs[i].legend(fontsize=7, loc="upper right",
                                   facecolor="#1a1a18", edgecolor="#3a3a38")

        # swing leg indicator per leg axis
        self.swing_patches = []
        for ax in self.ax_legs:
            p = ax.axvspan(0, 0, alpha=0.12, color="#3B8BD4")
            self.swing_patches.append(p)

        # ── Title ──────────────────────────────────────────────────────────
        mode_str = "DEMO MODE" if self.demo_mode else "LIVE  /joint_states"
        self.fig.suptitle(
            f"Go2 Gait Dashboard  ·  {mode_str}",
            fontsize=11, color="#c2c0b6", y=0.975
        )

        # ── Sliders ────────────────────────────────────────────────────────
        slider_ax_specs = [
            [0.06, 0.155, 0.18, 0.018],
            [0.06, 0.125, 0.18, 0.018],
            [0.06, 0.095, 0.18, 0.018],
            [0.06, 0.065, 0.18, 0.018],
        ]
        slider_defs = [
            ("SWING_H  (m)",   0.02, 0.16, self.swing_h),
            ("STEP_LEN (m)",   0.02, 0.20, self.step_len),
            ("PUSH_DIST(m)",   0.01, 0.15, self.push_dist),
            ("STANCE_H (m)",   0.26, 0.38, self.stance_h),
        ]
        self.sliders = {}
        for spec, (label, vmin, vmax, val) in zip(slider_ax_specs, slider_defs):
            ax_sl = self.fig.add_axes(spec, facecolor="#2e2e2c")
            sl = Slider(ax_sl, label, vmin, vmax, valinit=val,
                        color="#3B8BD4", track_color="#2e2e2c")
            sl.label.set_color("#c2c0b6")
            sl.label.set_fontsize(8)
            sl.valtext.set_color("#EF9F27")
            self.sliders[label] = sl

        self.sliders["SWING_H  (m)"].on_changed(self._on_slider)
        self.sliders["STEP_LEN (m)"].on_changed(self._on_slider)
        self.sliders["PUSH_DIST(m)"].on_changed(self._on_slider)
        self.sliders["STANCE_H (m)"].on_changed(self._on_slider)

        # ── Log analyser pane ──────────────────────────────────────────────
        ax_log_label = self.fig.add_axes([0.30, 0.01, 0.25, 0.185], facecolor="#1a1a18")
        ax_log_label.axis("off")
        ax_log_label.text(0.0, 1.0, "Paste terminal log below  →  click Analyse",
                          transform=ax_log_label.transAxes,
                          fontsize=8, color="#888780", va="top")

        ax_textbox = self.fig.add_axes([0.30, 0.04, 0.25, 0.14], facecolor="#2e2e2c")
        self.textbox = TextBox(ax_textbox, "", initial="Paste log here …",
                               color="#2e2e2c", hovercolor="#3a3a38",
                               label_pad=0)
        self.textbox.label.set_color("#888780")

        ax_btn = self.fig.add_axes([0.56, 0.085, 0.07, 0.04], facecolor="#2e2e2c")
        self.btn_analyse = Button(ax_btn, "Analyse ↗",
                                  color="#2e2e2c", hovercolor="#3B8BD4")
        self.btn_analyse.label.set_color("#c2c0b6")
        self.btn_analyse.label.set_fontsize(8)
        self.btn_analyse.on_clicked(self._on_analyse)

        # hints output
        self.ax_hints = self.fig.add_axes([0.64, 0.01, 0.34, 0.195], facecolor="#1a1a18")
        self.ax_hints.axis("off")
        self.hints_text = self.ax_hints.text(
            0.02, 0.98, "Hints will appear here after Analyse.",
            transform=self.ax_hints.transAxes,
            fontsize=8, color="#888780", va="top", wrap=True,
            family="monospace"
        )

        # ── Phase legend ───────────────────────────────────────────────────
        legend_handles = [
            mpatches.Patch(color=c, label=n)
            for c, n in zip(PHASE_COLORS, PHASE_NAMES)
        ]
        self.fig.legend(handles=legend_handles, loc="lower left",
                        bbox_to_anchor=(0.06, 0.01), ncol=4, fontsize=7,
                        facecolor="#1a1a18", edgecolor="#3a3a38",
                        labelcolor="#c2c0b6")

        plt.ion()
        plt.show()

    # ── Slider callback ────────────────────────────────────────────────────
    def _on_slider(self, _val):
        self.swing_h   = self.sliders["SWING_H  (m)"].val
        self.step_len  = self.sliders["STEP_LEN (m)"].val
        self.push_dist = self.sliders["PUSH_DIST(m)"].val
        self.stance_h  = self.sliders["STANCE_H (m)"].val

        # Push into sim
        self.sim.swing_h   = self.swing_h
        self.sim.step_len  = self.step_len
        self.sim.push_dist = self.push_dist
        self.sim.stance_h  = self.stance_h

        # Redraw planned arc
        xs, zs = full_trajectory_points(self.step_len, self.swing_h, self.stance_h)
        self.line_planned.set_data(xs, zs)

        # Update peak annotation
        peak_x, peak_z = swing_traj(0.5, self.step_len, self.swing_h, self.stance_h)
        self.annot_peak.xy = (peak_x, peak_z)
        self.annot_peak.xyann = (peak_x + 0.02, peak_z + 0.02)
        self.annot_peak.set_text(f"peak\n{self.swing_h*100:.0f} cm")

        # Update ground line
        for line in self.ax_traj.lines:
            if "ground" in line.get_label():
                line.set_ydata([-self.stance_h, -self.stance_h])

        # Update y limits
        ymax = -self.stance_h + self.swing_h + 0.04
        self.ax_traj.set_ylim(-self.stance_h - 0.06, ymax)
        self.ax_traj.invert_yaxis()

        # Publish to ROS topic if live
        if not self.demo_mode:
            self._publish_params()

    # ── Log analyse callback ───────────────────────────────────────────────
    def _on_analyse(self, _event):
        log = self.textbox.text
        hints = analyse_logs(log, self.swing_h, self.step_len,
                             self.push_dist, self.stance_h)
        hint_str = "\n\n".join(hints)
        # wrap lines
        wrapped = "\n".join(
            textwrap.fill(h, width=52) for h in hints
        )
        self.hints_text.set_text(wrapped)
        self.hints_text.set_color("#c2c0b6")
        self.fig.canvas.draw_idle()

    # ── ROS init ───────────────────────────────────────────────────────────
    def _init_ros(self):
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import JointState
            from std_msgs.msg import Float64MultiArray

            rclpy.init()
            self._ros_node = Node("go2_dashboard")
            self._ros_sub  = self._ros_node.create_subscription(
                JointState, "/joint_states", self._ros_js_callback, 10
            )
            self._ros_pub  = self._ros_node.create_publisher(
                Float64MultiArray, "/gait_params", 10
            )
            self._ros_thread = threading.Thread(
                target=rclpy.spin, args=(self._ros_node,), daemon=True
            )
            self._ros_thread.start()
            print("[dashboard] ROS2 node started, listening on /joint_states")
        except ImportError:
            print("[dashboard] rclpy not found – falling back to demo mode")
            self.demo_mode = True
            self.sim_thread = threading.Thread(target=self.sim.run, daemon=True)
            self.sim_thread.start()

    def _ros_js_callback(self, msg):
        """Map /joint_states name list to our leg/joint structure."""
        name_map = {
            "lf_hip_joint":       ("LF", "hip"),
            "lf_upper_leg_joint": ("LF", "thigh"),
            "lf_lower_leg_joint": ("LF", "calf"),
            "rf_hip_joint":       ("RF", "hip"),
            "rf_upper_leg_joint": ("RF", "thigh"),
            "rf_lower_leg_joint": ("RF", "calf"),
            "lh_hip_joint":       ("LH", "hip"),
            "lh_upper_leg_joint": ("LH", "thigh"),
            "lh_lower_leg_joint": ("LH", "calf"),
            "rh_hip_joint":       ("RH", "hip"),
            "rh_upper_leg_joint": ("RH", "thigh"),
            "rh_lower_leg_joint": ("RH", "calf"),
        }
        for name, pos in zip(msg.name, msg.position):
            if name in name_map:
                leg, jnt = name_map[name]
                self.sim.joint_angles[leg][["hip","thigh","calf"].index(jnt)] = math.degrees(pos)

    def _publish_params(self):
        try:
            from std_msgs.msg import Float64MultiArray
            m = Float64MultiArray()
            m.data = [self.swing_h, self.step_len, self.push_dist, self.stance_h]
            self._ros_pub.publish(m)
        except Exception:
            pass

    # ── Main update loop ───────────────────────────────────────────────────
    def update(self):
        now = time.time() - self.t0
        self.ts_time.append(now)

        angles = self.sim.joint_angles
        phase_idx = self.sim.phase
        d_raw     = self.sim.d_raw_swing
        swing_leg = self.sim.swing_leg

        # ── Record history ─────────────────────────────────────────────────
        for leg in LEG_NAMES:
            for j, jname in enumerate(["hip","thigh","calf"]):
                self.ts_angles[leg][jname].append(angles[leg][j])
        self.ts_phase.append(phase_idx)
        self.ts_d_raw.append(d_raw)

        n = len(self.ts_time)
        xs = list(range(n))

        # ── Update leg angle plots ─────────────────────────────────────────
        for i, leg in enumerate(LEG_NAMES):
            for jname in ["hip","thigh","calf"]:
                self.leg_lines[leg][jname].set_data(xs, list(self.ts_angles[leg][jname]))
            # highlight swing leg
            if i == swing_leg and phase_idx in (1, 2):
                start = max(0, n - 60)
                # Replace axvspan with a fresh one each time (most reliable)
                self.swing_patches[i].remove()
                self.swing_patches[i] = self.ax_legs[i].axvspan(
                    start, n, alpha=0.15, color="#3B8BD4")

            else:
                self.swing_patches[i].set_alpha(0.0)

        # ── Update trajectory trail ────────────────────────────────────────
        trail_x = list(self.sim.swing_foot_trail_x)
        trail_z = list(self.sim.swing_foot_trail_z)
        if trail_x:
            self.line_trail.set_data(trail_x, trail_z)
            self.dot_foot.set_data([trail_x[-1]], [trail_z[-1]])
            # Color by current swing leg
            self.line_trail.set_color(LEG_COLORS[swing_leg])
            self.dot_foot.set_color(LEG_COLORS[swing_leg])
        else:
            self.line_trail.set_data([], [])
            self.dot_foot.set_data([], [])

        # ── Phase scatter ──────────────────────────────────────────────────
        scatter_data = np.column_stack([xs, list(self.ts_phase)]) if n > 1 else np.empty((0,2))
        if len(scatter_data):
            self.scat_phase.set_offsets(scatter_data)
            self.scat_phase.set_array(np.array(list(self.ts_phase), dtype=float))
        phase_name = PHASE_NAMES[phase_idx]
        self.phase_text.set_text(f"► {phase_name}  leg {swing_leg}={LEG_NAMES[swing_leg]}")
        self.phase_text.set_color(PHASE_COLORS[phase_idx])

        # ── D_raw line ─────────────────────────────────────────────────────
        self.line_d.set_data(xs, list(self.ts_d_raw))
        # colour the line red when near singularity
        danger = any(abs(v) > 0.90 for v in list(self.ts_d_raw)[-10:])
        self.line_d.set_color("#E24B4A" if danger else "#EF9F27")

        # ── Rescale time-axis on all plots ─────────────────────────────────
        if n > 1:
            for ax in self.ax_legs + [self.ax_phase, self.ax_d]:
                ax.set_xlim(max(0, n - self.HISTORY), n)

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def run(self):
        print("Dashboard running.  Close the window or Ctrl-C to exit.")
        try:
            while plt.fignum_exists(self.fig.number):
                self.update()
                time.sleep(0.04)   # ~25 fps
        except KeyboardInterrupt:
            print("Exiting.")


# ─────────────────────────────────────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Go2 Gait Dashboard")
    parser.add_argument("--demo", action="store_true",
                        help="Run in simulation mode without a real robot")
    args = parser.parse_args()

    dash = GaitDashboard(demo_mode=args.demo or ("--demo" in sys.argv))
    dash.run()