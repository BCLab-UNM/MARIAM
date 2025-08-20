#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.patches import Polygon

# --------------------------
# Internal helpers
# --------------------------

def _unit_from_theta(theta):
    th = np.asarray(theta).reshape(-1)
    return np.column_stack((np.cos(th), np.sin(th)))

def _get_arrays(params):
    """Extract arrays from params and ensure proper shapes."""
    t   = np.asarray(params["t"]).reshape(-1)
    c   = np.asarray(params["center"])
    b1  = np.asarray(params["base1"])
    b2  = np.asarray(params["base2"])
    th1 = np.asarray(params["theta1"]).reshape(-1)
    th2 = np.asarray(params["theta2"]).reshape(-1)

    # Optional (for animation; not used in summary)
    p1 = np.asarray(params.get("pivot1", b1))
    p2 = np.asarray(params.get("pivot2", b2))
    contacts = params.get("contacts", None)
    if contacts is not None:
        g1, g2 = contacts
        g1 = np.asarray(g1); g2 = np.asarray(g2)
    else:
        g1, g2 = b1.copy(), b2.copy()

    phi = np.asarray(params["phi"]).reshape(-1)
    bp = params["box"]
    L = float(getattr(bp, "L", 0.1))
    W = float(getattr(bp, "W", 0.1))

    return t, c, b1, b2, p1, p2, g1, g2, th1, th2, phi, L, W

def _compute_limits(arrays, pad_ratio=0.08):
    pts = np.vstack([a[:, :2] for a in arrays if a is not None])
    xmin, ymin = pts.min(axis=0)
    xmax, ymax = pts.max(axis=0)
    dx, dy = xmax - xmin, ymax - ymin
    pad = pad_ratio * max(dx, dy, 1e-6)
    return (xmin - pad, xmax + pad, ymin - pad, ymax + pad)

def _box_corners(center_xy, phi, L, W):
    cx, cy = center_xy
    c, s = np.cos(phi), np.sin(phi)
    R = np.array([[c, -s], [s, c]])
    half = np.array([[ L/2,  W/2],
                     [-L/2,  W/2],
                     [-L/2, -W/2],
                     [ L/2, -W/2]])
    return (R @ half.T).T + np.array([cx, cy])

def _downsample(arr, stride):
    if stride <= 1: return arr
    return arr[::stride].copy()

# --------------------------
# Public plotting utilities
# --------------------------

def plot_summary(params, show=True, save_prefix=None):
    """
    Static summary plot that ONLY shows trajectories:
      - Centerline
      - Base 1 path
      - Base 2 path
    No arms, no pivots, no boxes.
    """
    t, c, b1, b2, *_ = _get_arrays(params)

    fig, ax = plt.subplots(figsize=(7.5, 6.5))
    ax.set_aspect('equal', adjustable='box')

    # Paths only
    ax.plot(c[:, 0],  c[:, 1],  color='0.1',      lw=1.25, label='Centerline')
    ax.plot(b1[:, 0], b1[:, 1], color='tab:blue',  lw=1.6, label='Base 1')
    ax.plot(b2[:, 0], b2[:, 1], color='tab:orange',lw=1.6, label='Base 2')

    # Start/goal markers (centerline)
    ax.plot([c[0, 0]],  [c[0, 1]],  'ko', ms=5, label='Start')
    ax.plot([c[-1, 0]], [c[-1, 1]], 'ks', ms=5, label='Goal')

    xmin, xmax, ymin, ymax = _compute_limits([c, b1, b2])
    ax.set_xlim(xmin, xmax); ax.set_ylim(ymin, ymax)
    ax.grid(True, alpha=0.2)
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.set_title('Cooperative Carry — Trajectories')

    # Legend (dedup)
    handles, labels = ax.get_legend_handles_labels()
    uniq = dict(zip(labels, handles))
    ax.legend(uniq.values(), uniq.keys(), loc='best', frameon=True)

    if save_prefix:
        fig.savefig(f"{save_prefix}_summary.png", dpi=180, bbox_inches='tight')

    if show:
        plt.show()
    else:
        plt.close(fig)


def plot_trajectories(
    desired_ross_poses, desired_monica_poses,
    ross_poses, monica_poses):
    # ross trajectory
    plt.plot(
        desired_ross_poses[0],
        desired_ross_poses[1],
        '-r'
    )
    plt.plot(
        desired_monica_poses[0],
        desired_monica_poses[1],
        '-b'
    )
    plt.plot(
        ross_poses[0],
        ross_poses[1],
        '--r'
    )
    plt.plot(
        monica_poses[0],
        monica_poses[1],
        '--b'
    )
    plt.grid(True)
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.title(f'Trajectory over time')
    plt.savefig(fname=f'ros2_coop_traj')


def animate_carry(params,
                  fps=30,
                  head_len=0.18,          # short/light so it won't be confused with arms
                  show=True,
                  show_arms=True,         # arms are black
                  show_box=True,
                  show_pivots=True,       # draw pivot markers/paths in animation
                  draw_links=False,       # base→pivot links off by default
                  trail_len_s=0.5,
                  frame_step=1,           # 1 = realtime smooth; increase to lighten load
                  path_stride=6,          # decimate static paths for speed
                  save_gif=None           # None by default (no slow encoding)
                  ):
    """
    Real-time friendly animation:
    - No GIF saved unless save_gif is set (encoding is slow).
    - Blitting + lightweight artists; headings are light gray and short.
    - Arms (pivot->box contacts) are thick black lines.
    """
    t, c, b1, b2, p1, p2, g1, g2, th1, th2, phi, L, W = _get_arrays(params)
    N_full = len(t)
    if N_full < 2:
        raise ValueError("Not enough samples in params['t'] to animate.")

    # Frame decimation (keep default = 1 for smooth real-time; increase to 2+ if needed)
    k_idx = np.arange(0, N_full, max(1, int(frame_step)), dtype=int)
    if k_idx[-1] != N_full - 1:
        k_idx = np.append(k_idx, N_full - 1)
    tA   = t[k_idx]
    cA   = c[k_idx]
    b1A  = b1[k_idx]; b2A = b2[k_idx]
    p1A  = p1[k_idx]; p2A = p2[k_idx]
    g1A  = g1[k_idx]; g2A = g2[k_idx]
    th1A = th1[k_idx]; th2A = th2[k_idx]
    phiA = phi[k_idx]
    N = len(k_idx)

    # Heading unit vectors for tiny pose ticks
    e1 = _unit_from_theta(th1A)
    e2 = _unit_from_theta(th2A)

    fig, ax = plt.subplots(figsize=(7.5, 6.5))
    ax.set_aspect('equal', adjustable='box')

    # Static paths (light, decimated)
    cS  = _downsample(c, path_stride)
    b1S = _downsample(b1, path_stride)
    b2S = _downsample(b2, path_stride)
    if show_pivots:
        p1S = _downsample(p1, path_stride)
        p2S = _downsample(p2, path_stride)
    else:
        p1S = p2S = None

    ax.plot(cS[:, 0],  cS[:, 1],  color='0.4',  lw=1.0, alpha=0.35)
    ax.plot(b1S[:, 0], b1S[:, 1], color='tab:blue',  lw=1.0, alpha=0.45)
    ax.plot(b2S[:, 0], b2S[:, 1], color='tab:orange', lw=1.0, alpha=0.45)
    if show_pivots:
        ax.plot(p1S[:, 0], p1S[:, 1], color='tab:blue',  lw=0.9, ls='--', alpha=0.5)
        ax.plot(p2S[:, 0], p2S[:, 1], color='tab:orange', lw=0.9, ls='--', alpha=0.5)

    # Limits & axes
    arrays_for_limits = [c, b1, b2]
    if show_pivots:
        arrays_for_limits += [p1, p2]
    xmin, xmax, ymin, ymax = _compute_limits(arrays_for_limits)
    ax.set_xlim(xmin, xmax); ax.set_ylim(ymin, ymax)
    ax.grid(True, alpha=0.2)
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.set_title('Cooperative Carry — Real-time Animation')

    # Moving markers (animated for blit)
    (base1_pt,) = ax.plot([], [], 'o', color='tab:blue',  ms=5, animated=True)
    (base2_pt,) = ax.plot([], [], 'o', color='tab:orange', ms=5, animated=True)

    # Short, subtle heading ticks (light gray) – avoids confusion with arms
    (b1_head,) = ax.plot([], [], '-', color='0.6',  lw=1.6, animated=True, solid_capstyle='round')
    (b2_head,) = ax.plot([], [], '-', color='0.6',  lw=1.6, animated=True, solid_capstyle='round')

    # Pivots & optional base→pivot links
    if show_pivots:
        (pivot1_pt,) = ax.plot([], [], 's', color='tab:blue',  ms=4, animated=True)
        (pivot2_pt,) = ax.plot([], [], 's', color='tab:orange', ms=4, animated=True)
        if draw_links:
            (link1,) = ax.plot([], [], '-', color='0.5',  lw=1.2, alpha=0.8, animated=True)
            (link2,) = ax.plot([], [], '-', color='0.5',  lw=1.2, alpha=0.8, animated=True)
        else:
            link1 = link2 = None
    else:
        pivot1_pt = pivot2_pt = link1 = link2 = None

    # Arms (pivot -> grasp points on box faces) — BLACK and prominent
    if show_arms:
        (arm1_line,) = ax.plot([], [], '-', color='k',  lw=3.0, animated=True, solid_capstyle='round')
        (arm2_line,) = ax.plot([], [], '-', color='k',  lw=3.0, animated=True, solid_capstyle='round')
    else:
        arm1_line = arm2_line = None

    # Box patch (neutral gray outline)
    if show_box:
        corners0 = _box_corners(cA[0], phiA[0], L, W)
        box_patch = Polygon(corners0, closed=True, fill=False, ec='0.25', lw=2.0, animated=True)
        ax.add_patch(box_patch)
    else:
        box_patch = None

    # Trails (bases & pivots)
    trail_frames = max(1, int(round(trail_len_s * fps)))  # wall-clock mapping
    (trail1,) = ax.plot([], [], '-', color='tab:blue',  lw=1.8, alpha=0.9, animated=True)
    (trail2,) = ax.plot([], [], '-', color='tab:orange', lw=1.8, alpha=0.9, animated=True)
    if show_pivots:
        (ptrail1,) = ax.plot([], [], '--', color='0.2', lw=1.2, alpha=0.9, animated=True)
        (ptrail2,) = ax.plot([], [], '--', color='0.2', lw=1.2, alpha=0.9, animated=True)
    else:
        ptrail1 = ptrail2 = None

    artists_all = [base1_pt, base2_pt, b1_head, b2_head, trail1, trail2]
    if show_pivots:
        artists_all += [pivot1_pt, pivot2_pt]
        if ptrail1 is not None: artists_all += [ptrail1, ptrail2]
        if link1 is not None:   artists_all += [link1, link2]
    if show_arms:
        artists_all += [arm1_line, arm2_line]
    if show_box:
        artists_all += [box_patch]

    # ---------- animation callbacks ----------

    def init():
        for ln in artists_all:
            if isinstance(ln, Polygon):
                ln.set_xy(np.empty((0, 2)))
            else:
                ln.set_data([], [])
        return artists_all

    def animate(i):
        # Use decimated index directly
        k = i  # k indexes arrays with suffix A

        # current positions
        x1, y1 = b1A[k, 0], b1A[k, 1]
        x2, y2 = b2A[k, 0], b2A[k, 1]
        base1_pt.set_data([x1], [y1])
        base2_pt.set_data([x2], [y2])

        # short heading ticks (subtle pose cue)
        hx1, hy1 = x1 + head_len * e1[k, 0], y1 + head_len * e1[k, 1]
        hx2, hy2 = x2 + head_len * e2[k, 0], y2 + head_len * e2[k, 1]
        b1_head.set_data([x1, hx1], [y1, hy1])
        b2_head.set_data([x2, hx2], [y2, hy2])

        # trails (show last window)
        k0 = max(0, k - trail_frames)
        trail1.set_data(b1A[k0:k+1, 0], b1A[k0:k+1, 1])
        trail2.set_data(b2A[k0:k+1, 0], b2A[k0:k+1, 1])

        if show_pivots:
            xp1, yp1 = p1A[k, 0], p1A[k, 1]
            xp2, yp2 = p2A[k, 0], p2A[k, 1]
            pivot1_pt.set_data([xp1], [yp1])
            pivot2_pt.set_data([xp2], [yp2])
            if ptrail1 is not None:
                ptrail1.set_data(p1A[k0:k+1, 0], p1A[k0:k+1, 1])
                ptrail2.set_data(p2A[k0:k+1, 0], p2A[k0:k+1, 1])
            if link1 is not None:
                link1.set_data([x1, xp1], [y1, yp1])
                link2.set_data([x2, xp2], [y2, yp2])

        # arms (pivot -> grasp point on box face) — BLACK
        if show_arms:
            arm1_line.set_data([p1A[k,0], g1A[k,0]], [p1A[k,1], g1A[k,1]])
            arm2_line.set_data([p2A[k,0], g2A[k,0]], [p2A[k,1], g2A[k,1]])

        # box outline
        if show_box:
            box_patch.set_xy(_box_corners(cA[k], phiA[k], L, W))

        return artists_all

    # Real-time pacing target
    interval_ms = 1000.0 / float(fps)

    anim = animation.FuncAnimation(
        fig, animate, init_func=init,
        frames=len(tA), interval=interval_ms, blit=True
    )

    # Only save a GIF if explicitly requested (slow)
    if save_gif:
        try:
            anim.save(save_gif, writer="pillow", fps=fps)
        except Exception as e:
            print(f"[WARN] Failed to save GIF to '{save_gif}': {e}")

    if show:
        plt.show()
    else:
        plt.close(fig)

    return anim
