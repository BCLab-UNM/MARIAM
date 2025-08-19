#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simple public API on top of the SCQP planner.

Public calls:
  params = opt_traj_params([xi,yi,thi], [xf,yf,thf], T, **options)
  b1_pose, b2_pose = traj(params, t, T=None)
  p1_pose, p2_pose = traj_pivot(params, t, T=None)   # arm pivot poses

Each pose is (x, y, theta). Internally we keep a 500 Hz trajectory.

Updates:
- Enforce rest-to-rest by default for x,y: v(0)=v(T)=0 and a(0)=a(T)=0 via soft penalties.
- Expose non-zero endpoint velocity/acceleration targets:
    v0_xy, vT_xy, a0_xy, aT_xy   (world-frame X/Y)
- Support arm pivot offset relative to base center:
    pivot_fwd_offset (float or (d1, d2))
  If a scalar is provided, we interpret it as: rear robot gets +d (forward),
  front robot (facing backwards) gets -d. A tuple is respected verbatim.
"""

from dataclasses import dataclass
import numpy as np

# ---------- small utils ----------

def wrap_to_pi(a): return (a + np.pi) % (2*np.pi) - np.pi

def unwrap_relative(wrapped):
    out = np.zeros_like(wrapped); out[0] = wrapped[0]
    for k in range(1, len(wrapped)):
        step = wrap_to_pi(wrapped[k] - wrapped[k-1])
        out[k] = out[k-1] + step
    return out

def e_unit(theta):
    """Returns Nx2 array of unit vectors aligned with angle theta (radians)."""
    th = np.asarray(theta)
    return np.vstack((np.cos(th), np.sin(th))).T

def e_perp(theta):
    """Returns Nx2 unit vectors rotated +90° from e_unit(theta)."""
    th = np.asarray(theta)
    return np.vstack((-np.sin(th), np.cos(th))).T

def rot2d(th):
    c,s = np.cos(th), np.sin(th)
    return np.array([[c,-s],[s,c]])

def smooth_signal(z, win):
    """Symmetric moving-average with edge hold; win must be odd >=3."""
    w = int(win)
    if w < 3: return z.copy()
    if w % 2 == 0: w += 1
    pad = w // 2
    zpad = np.pad(z, (pad, pad), mode='edge')
    kernel = np.ones(w, dtype=float) / w
    return np.convolve(zpad, kernel, mode='valid')

# ---------- SCQP core ----------

@dataclass
class BoxParams:
    L: float = 0.16
    W: float = 0.11
    a: float = 0.2

@dataclass
class SmoothParams:
    # centerline (x,y)
    w_track: float = 120.0
    w_snap:  float = 1.0
    w_bc:    float = 1e6
    # yaw (phi)
    w_phi_d1:   float = 2.0
    w_phi_d2:   float = 50.0
    w_align:    float = 30.0
    s0_align:   float = 0.12
    w_phi_bc:   float = 2e5
    w_dphi_bc:  float = 5e4
    align_eps:  float = 1e-3
    # timing
    T: float = 16.0
    coarse_hz: int = 100
    # relaxation
    relax: float = 0.6
    # velocity/acceleration boundary penalties for x,y
    w_v_bc: float = 5e4   # velocity at t=0,T
    w_a_bc: float = 5e4   # acceleration at t=0,T

# finite-difference helpers
def apply_D1(N, dt, x):
    a=1.0/(2.0*dt); out=np.zeros(N)
    out[0]=(-3*x[0]+4*x[1]-x[2])*a
    out[1:-1]=(x[2:]-x[:-2])*a
    out[-1]=(x[-3]-4*x[-2]+3*x[-1])*a
    return out

def q_from_D1_weights(N, dt, w):
    a=1.0/(2.0*dt)
    d0=np.zeros(N); d1=np.zeros(N-1); d2=np.zeros(N-2)
    c=np.array([-3*a,4*a,-1*a]); idx=np.array([0,1,2]); wk=w[0]
    d0[idx]+=wk*c*c; d1[idx[:2]]+=wk*c[:2]*c[1:]; d2[idx[:1]]+=wk*c[0]*c[2]
    for k in range(1,N-1):
        wk=w[k]; i0=k-1; i2=k+1
        d0[i0]+=wk*a*a; d0[i2]+=wk*a*a; d2[i0]+=wk*(-a*a)
    c=np.array([1*a,-4*a,3*a]); idx=np.array([N-3,N-2,N-1]); wk=w[N-1]
    d0[idx]+=wk*c*c; d1[idx[:2]]+=wk*c[:2]*c[1:]; d2[idx[:1]]+=wk*c[0]*c[2]
    Q=np.diag(d0); Q+=np.diag(d1,1)+np.diag(d1,-1); Q+=np.diag(d2,2)+np.diag(d2,-2)
    return Q

def q_from_D2(N, dt):
    b=1.0/(dt*dt)
    d0=np.zeros(N); d1=np.zeros(N-1); d2=np.zeros(N-2)
    def add(coeff,idx):
        c=coeff*b
        d0[idx]+=c*c
        for j in range(len(idx)-1):
            if idx[j+1]==idx[j]+1: d1[idx[j]]+=c[j]*c[j+1]
        for j in range(len(idx)-2):
            if idx[j+2]==idx[j]+2: d2[idx[j]]+=c[j]*c[j+2]
    add(np.array([1,-2,1]),np.array([0,1,2]))
    for k in range(1,N-1): add(np.array([1,-2,1]),np.array([k-1,k,k+1]))
    add(np.array([1,-2,1]),np.array([N-3,N-2,N-1]))
    Q=np.diag(d0); Q+=np.diag(d1,1)+np.diag(d1,-1); Q+=np.diag(d2,2)+np.diag(d2,-2)
    return Q

def q_from_D4(N):
    s=np.array([1.,-4.,6.,-4.,1.])
    d0=np.zeros(N); d1=np.zeros(N-1); d2=np.zeros(N-2); d3=np.zeros(N-3); d4=np.zeros(N-4)
    for i in range(N-4):
        idx=np.arange(i,i+5)
        d0[idx]+=s*s
        d1[idx[:4]]+=s[:4]*s[1:]
        d2[idx[:3]]+=s[:3]*s[2:]
        d3[idx[:2]]+=s[:2]*s[3:]
        d4[idx[:1]]+=s[:1]*s[4:]
    Q=np.diag(d0)
    Q+=np.diag(d1,1)+np.diag(d1,-1)
    Q+=np.diag(d2,2)+np.diag(d2,-2)
    Q+=np.diag(d3,3)+np.diag(d3,-3)
    Q+=np.diag(d4,4)+np.diag(d4,-4)
    return Q

def apply_D1T_weighted(N, dt, w, z):
    a=1.0/(2.0*dt); b=np.zeros(N)
    wk=w[0]; zk=z[0]; b[0]+=wk*(-3*a)*zk; b[1]+=wk*(4*a)*zk; b[2]+=wk*(-1*a)*zk
    for k in range(1,N-1):
        wk=w[k]; zk=z[k]; b[k-1]+=wk*(-a)*zk; b[k+1]+=wk*(a)*zk
    wk=w[N-1]; zk=z[N-1]
    b[N-3]+=wk*(1*a)*zk; b[N-2]+=wk*(-4*a)*zk; b[N-1]+=wk*(3*a)*zk
    return b

# derivative "rows" for endpoint velocity/acceleration penalties
def _d1_rows(N, dt):
    a = 1.0/(2.0*dt)
    r0 = np.zeros(N); r0[0:3]  = np.array([-3*a, 4*a, -1*a])  # start vel stencil
    rT = np.zeros(N); rT[-3:]  = np.array([ 1*a,-4*a,  3*a])  # end vel stencil
    return r0, rT

def _d2_rows(N, dt):
    b = 1.0/(dt*dt)
    s0 = np.zeros(N); s0[0:3]  = np.array([1,-2,1]) * b       # start acc stencil
    sT = np.zeros(N); sT[-3:]  = np.array([1,-2,1]) * b       # end acc stencil
    return s0, sT

def scqp_center_yaw(x_ref, y_ref, t_s, sm: SmoothParams, iters_scqp=1, phi_end_goal=None,
                    # endpoint targets (world XY)
                    v0_xy=(0.0,0.0), vT_xy=(0.0,0.0),
                    a0_xy=(0.0,0.0), aT_xy=(0.0,0.0)):
    """SCQP with speed-gated yaw alignment and soft endpoint velocity/accel targets for x,y."""
    N=len(t_s); dt=t_s[1]-t_s[0]; I=np.eye(N)
    Qsnap=q_from_D4(N)
    Qd2=q_from_D2(N, dt)
    Qd1=q_from_D1_weights(N, dt, np.ones(N))
    r0, rT = _d1_rows(N, dt)
    s0, sT = _d2_rows(N, dt)

    # ----- initial x,y solve -----
    Ax0=sm.w_track*I + sm.w_snap*Qsnap
    Ay0=Ax0.copy()
    bx0=sm.w_track*x_ref.copy(); by0=sm.w_track*y_ref.copy()

    # position BC (strong)
    Ax0[0,0]+=sm.w_bc;   bx0[0]+=sm.w_bc*x_ref[0]
    Ax0[-1,-1]+=sm.w_bc; bx0[-1]+=sm.w_bc*x_ref[-1]
    Ay0[0,0]+=sm.w_bc;   by0[0]+=sm.w_bc*y_ref[0]
    Ay0[-1,-1]+=sm.w_bc; by0[-1]+=sm.w_bc*y_ref[-1]

    # velocity/acceleration BCs
    Ax0 += sm.w_v_bc * (r0[:,None]@r0[None,:]) + sm.w_v_bc * (rT[:,None]@rT[None,:])
    bx0 += sm.w_v_bc * (r0 * float(v0_xy[0])) + sm.w_v_bc * (rT * float(vT_xy[0]))
    Ax0 += sm.w_a_bc * (s0[:,None]@s0[None,:]) + sm.w_a_bc * (sT[:,None]@sT[None,:])
    bx0 += sm.w_a_bc * (s0 * float(a0_xy[0])) + sm.w_a_bc * (sT * float(aT_xy[0]))

    Ay0 += sm.w_v_bc * (r0[:,None]@r0[None,:]) + sm.w_v_bc * (rT[:,None]@rT[None,:])
    by0 += sm.w_v_bc * (r0 * float(v0_xy[1])) + sm.w_v_bc * (rT * float(vT_xy[1]))
    Ay0 += sm.w_a_bc * (s0[:,None]@s0[None,:]) + sm.w_a_bc * (sT[:,None]@sT[None,:])
    by0 += sm.w_a_bc * (s0 * float(a0_xy[1])) + sm.w_a_bc * (sT * float(aT_xy[1]))

    x=np.linalg.solve(Ax0, bx0); y=np.linalg.solve(Ay0, by0)

    # initial ψ, φ
    dx=apply_D1(N, dt, x); dy=apply_D1(N, dt, y)
    psi=np.arctan2(dy, dx); phi=psi.copy()

    for _ in range(iters_scqp):
        dx_j=apply_D1(N, dt, x); dy_j=apply_D1(N, dt, y)
        s  = np.sqrt(np.maximum(dx_j*dx_j+dy_j*dy_j, sm.align_eps**2))
        psi_j=np.arctan2(dy_j, dx_j)
        Px = -dy_j / np.maximum(s*s, sm.align_eps**2)
        Py =  dx_j / np.maximum(s*s, sm.align_eps**2)
        gate = (s*s)/(s*s + sm.s0_align**2)
        wA   = sm.w_align * gate
        c_j  = (phi - psi_j) + Px*dx_j + Py*dy_j

        # ---- X update ----
        Qx_align = q_from_D1_weights(N, dt, wA*(Px*Px))
        Ax = sm.w_track*I + sm.w_snap*Qsnap + Qx_align
        bx = sm.w_track*x_ref.copy()
        Ax[0,0]+=sm.w_bc; bx[0]+=sm.w_bc*x_ref[0]
        Ax[-1,-1]+=sm.w_bc; bx[-1]+=sm.w_bc*x_ref[-1]
        bx += apply_D1T_weighted(N, dt, wA*Px, c_j - Py*dy_j)

        Ax += sm.w_v_bc * (r0[:,None]@r0[None,:]) + sm.w_v_bc * (rT[:,None]@rT[None,:])
        bx += sm.w_v_bc * (r0 * float(v0_xy[0])) + sm.w_v_bc * (rT * float(vT_xy[0]))
        Ax += sm.w_a_bc * (s0[:,None]@s0[None,:]) + sm.w_a_bc * (sT[:,None]@sT[None,:])
        bx += sm.w_a_bc * (s0 * float(a0_xy[0])) + sm.w_a_bc * (sT * float(aT_xy[0]))

        x = (1.0-sm.relax)*x + sm.relax*np.linalg.solve(Ax, bx)

        # ---- Y update ----
        dx_new=apply_D1(N, dt, x)
        Qy_align = q_from_D1_weights(N, dt, wA*(Py*Py))
        Ay = sm.w_track*I + sm.w_snap*Qsnap + Qy_align
        by = sm.w_track*y_ref.copy()
        Ay[0,0]+=sm.w_bc; by[0]+=sm.w_bc*y_ref[0]
        Ay[-1,-1]+=sm.w_bc; by[-1]+=sm.w_bc*y_ref[-1]
        by += apply_D1T_weighted(N, dt, wA*Py, c_j - Px*dx_new)

        Ay += sm.w_v_bc * (r0[:,None]@r0[None,:]) + sm.w_v_bc * (rT[:,None]@rT[None,:])
        by += sm.w_v_bc * (r0 * float(v0_xy[1])) + sm.w_v_bc * (rT * float(vT_xy[1]))
        Ay += sm.w_a_bc * (s0[:,None]@s0[None,:]) + sm.w_a_bc * (sT[:,None]@sT[None,:])
        by += sm.w_a_bc * (s0 * float(a0_xy[1])) + sm.w_a_bc * (sT * float(aT_xy[1]))

        y = (1.0-sm.relax)*y + sm.relax*np.linalg.solve(Ay, by)

        # ---- φ solve ----
        dy_new=apply_D1(N, dt, y)
        dx_diff=dx_new - dx_j; dy_diff=dy_new - dy_j
        phi_target = psi_j + Px*dx_diff + Py*dy_diff

        Aphi = sm.w_phi_d2*Qd2 + sm.w_phi_d1*Qd1 + np.diag(wA)
        bphi = (wA * phi_target)
        # endpoints: φ guided by path tangent (or user end goal) and zero φ̇ at ends
        Aphi[0,0]   += sm.w_phi_bc;   bphi[0]   += sm.w_phi_bc*psi_j[0]
        end_ref = psi_j[-1] if (phi_end_goal is None) else float(phi_end_goal)
        Aphi[-1,-1] += sm.w_phi_bc;   bphi[-1]  += sm.w_phi_bc*end_ref
        a=1.0/(2.0*dt)
        r0_idx=np.array([0,1,2]); r0v=np.array([-3*a,4*a,-1*a])
        rn_idx=np.array([N-3,N-2,N-1]); rnv=np.array([1*a,-4*a,3*a])
        Aphi[np.ix_(r0_idx,r0_idx)] += sm.w_dphi_bc*(r0v[:,None]@r0v[None,:])
        Aphi[np.ix_(rn_idx,rn_idx)] += sm.w_dphi_bc*(rnv[:,None]@rnv[None,:])

        phi_star=np.linalg.solve(Aphi, bphi)
        phi = wrap_to_pi((1.0-sm.relax)*phi + sm.relax*phi_star)

    return x, y, phi

# ---------- post-process to 500 Hz; bases, headings, pivots ----------

def _canonical_s_shift(N, Lx, Ly):
    """
    Canonical lateral shift: x(u)=Lx*u, y(u)=Ly*S(u) with quintic S(u)
    that has zero vel/acc at both ends. u in [0,1], N samples.
    """
    u = np.linspace(0.0, 1.0, N)
    S = 10*u**3 - 15*u**4 + 6*u**5   # C2 at ends
    x = Lx * u
    y = Ly * S
    return x, y

def _freeze_phi_and_build_heads(x, y, phi,
                                T, bp:BoxParams,
                                freeze_stop_thr=0.005, freeze_margin_s=0.35,
                                phi_blend_s=0.20,
                                heading_smooth_s=0.15,
                                heading_speed_thr=0.01,
                                heading_hyst_ratio=1.5,
                                anti_parallel_end=False, end_blend_s=0.15,
                                # forward offset(s) of arm pivot from base center(s)
                                pivot_fwd_offset=0.0):
    """Post-process to 500 Hz and compute bases + headings + pivots."""
    # 500 Hz grid
    N_o = int(round(500*T)) + 1
    t   = np.linspace(0.0, T, N_o)
    dt  = t[1] - t[0]
    # upsample center and yaw
    t_s = np.linspace(0, T, len(x))
    X   = np.interp(t, t_s, x)
    Y   = np.interp(t, t_s, y)
    PHI = wrap_to_pi(np.interp(t, t_s, np.unwrap(phi)))
    c   = np.column_stack([X,Y])

    # center speed and φ freeze (end spin fix)
    vx=np.gradient(X, dt); vy=np.gradient(Y, dt)
    speed=np.hypot(vx,vy)
    moving = speed > freeze_stop_thr
    if np.any(moving):
        last = int(np.max(np.where(moving)[0]))
        kf   = max(0, last - int(round(freeze_margin_s/dt)))
        phiu = np.unwrap(PHI)
        phi_const = phiu[kf]
        blend = max(1, int(round(phi_blend_s/dt)))
        kb = max(0, kf - blend)
        if kf > kb:
            alphas = np.linspace(0.0, 1.0, kf-kb, endpoint=False)
            phiu[kb:kf] = (1.0 - alphas)*phiu[kb:kf] + alphas*phi_const
        phiu[kf:] = phi_const
        PHI = wrap_to_pi(phiu)

    # derivatives from smoothed signals
    win = max(3, int(round(heading_smooth_s / dt))); win += (win % 2 == 0)
    Xf = smooth_signal(X, win); Yf = smooth_signal(Y, win)
    PHIf = wrap_to_pi(smooth_signal(np.unwrap(PHI), win))
    vx_f = np.gradient(Xf, dt); vy_f = np.gradient(Yf, dt)
    phidot = np.gradient(np.unwrap(PHIf), dt)

    # geometry for bases/contacts (box faces)
    vhat = e_unit(PHI); nhat = e_perp(PHI)
    nhat_vel = e_perp(PHIf)
    r = bp.L/2 + bp.a
    g1 = c - (bp.L/2)*vhat; g2 = c + (bp.L/2)*vhat
    b1 = g1 - bp.a*vhat;    b2 = g2 + bp.a*vhat

    # velocities (not exported directly; kept for internal heading gating)
    cdot  = np.column_stack([vx_f, vy_f])
    b1dot = cdot - (r*phidot)[:,None]*nhat_vel
    b2dot = cdot + (r*phidot)[:,None]*nhat_vel

    # headings with speed gating + hysteresis
    sp1=np.hypot(b1dot[:,0], b1dot[:,1]); sp2=np.hypot(b2dot[:,0], b2dot[:,1])
    th1=np.zeros_like(X); th2=np.zeros_like(X)
    th1[0]=np.arctan2(b1dot[0,1], b1dot[0,0])
    th2[0]=np.arctan2(b2dot[0,1], b2dot[0,0])
    thr_off = float(heading_speed_thr); thr_on = float(heading_hyst_ratio)*thr_off
    upd1 = sp1[0] > thr_on; upd2 = sp2[0] > thr_on
    for k in range(1, len(X)):
        if upd1:
            if sp1[k] < thr_off: upd1=False
        else:
            if sp1[k] > thr_on:  upd1=True
        if upd2:
            if sp2[k] < thr_off: upd2=False
        else:
            if sp2[k] > thr_on:  upd2=True
        th1[k] = np.arctan2(b1dot[k,1], b1dot[k,0]) if upd1 else th1[k-1]
        th2[k] = np.arctan2(b2dot[k,1], b2dot[k,0]) if upd2 else th2[k-1]

    # final heading freeze + optional anti-parallel end
    moving2 = (speed>freeze_stop_thr)|(sp1>freeze_stop_thr)|(sp2>freeze_stop_thr)
    if np.any(moving2):
        last = int(np.max(np.where(moving2)[0]))
        kf   = max(0, last - int(round(freeze_margin_s/dt)))
        if anti_parallel_end:
            th1_end = th1[kf]; th2_end = wrap_to_pi(th1_end - np.pi)
            blend = max(1, int(round(end_blend_s/dt)))
            kb = max(0, kf - blend)
            alphas = np.linspace(0.0, 1.0, kf-kb, endpoint=False)
            if kf > kb:
                th2[kb:kf] = wrap_to_pi((1.0 - alphas)*th2[kb:kf] + alphas*th2_end)
            th1[kf:] = th1_end; th2[kf:] = th2_end
        else:
            th1[kf:] = th1[kf]; th2[kf:] = th2[kf]

    dth_wrapped = wrap_to_pi(th1 - th2)
    dth_cont    = unwrap_relative(dth_wrapped)

    # ---------- pivots (base heading offsets) ----------
    # If scalar: rear robot gets +d (forward); front robot gets -d (faces backwards).
    if isinstance(pivot_fwd_offset, (list, tuple, np.ndarray)):
        d1 = float(pivot_fwd_offset[0])
        d2 = float(pivot_fwd_offset[1])
    else:
        d = float(pivot_fwd_offset)
        d1, d2 = d, -d

    e1 = e_unit(th1)  # N×2
    e2 = e_unit(th2)  # N×2
    p1 = b1 + d1 * e1
    p2 = b2 + d2 * e2

    return t, c, PHI, b1, b2, wrap_to_pi(th1), wrap_to_pi(th2), dth_cont, (g1,g2), p1, p2

# ---------- PUBLIC API ----------

def opt_traj_params(start, goal, T,
                    scqp_iters=1,
                    # end spin / end relation
                    anti_parallel_end=False, end_blend_s=0.15,
                    freeze_stop_thr=0.02, freeze_margin_s=0.35, phi_blend_s=0.20,
                    # heading noise controls
                    heading_smooth_s=0.15, heading_speed_thr=0.03, heading_hyst_ratio=1.5,
                    # solver weights overrides (optional)
                    w_align=None, w_phi_d2=None,
                    # optional: bias φ(T) to user thf (soft)
                    bias_phi_to_thf=True,
                    # velocity/acceleration endpoint targets (world XY)
                    v0_xy=(0.0, 0.0), vT_xy=(0.0, 0.0),
                    a0_xy=(0.0, 0.0), aT_xy=(0.0, 0.0),
                    # arm pivot offset(s) relative to base centers (m)
                    pivot_fwd_offset=0.16):
    """
    Build full trajectory once. Returns a dict `params` with pre-sampled arrays.

    Notes:
      * Positions always match the requested goal exactly.
      * If `bias_phi_to_thf=True`, we softly bias φ(T) toward goal[2].
      * x,y satisfy soft boundary targets for velocity/acceleration at t=0 and t=T.
      * `pivot_fwd_offset` can be a scalar or (d1, d2). For a scalar, we apply +d to the
        rear robot and -d to the front robot (which faces backwards).
    """
    (xi,yi,thi) = map(float, start)
    (xf,yf,thf) = map(float, goal)
    sm = SmoothParams()
    if w_align  is not None: sm.w_align  = float(w_align)
    if w_phi_d2 is not None: sm.w_phi_d2 = float(w_phi_d2)
    sm.T = float(T)

    # displacement in start-yaw local frame
    R_wl = rot2d(-thi)
    d_local = R_wl @ np.array([xf - xi, yf - yi])
    Lx, Ly = d_local[0], d_local[1]

    # coarse grid
    N_s = int(round(sm.coarse_hz*sm.T)) + 1
    t_s = np.linspace(0.0, sm.T, N_s)

    # canonical S-shift in local frame (length/time-parameterized)
    xr_loc, yr_loc = _canonical_s_shift(N_s, Lx, Ly)

    # rotate back to world + translate
    R_lw = rot2d(thi)
    XYw  = (R_lw @ np.vstack([xr_loc, yr_loc])).T + np.array([[xi, yi]])
    x_ref = XYw[:,0]; y_ref = XYw[:,1]

    # SCQP centerline + yaw (softly bias φ(T) to thf if requested)
    phi_end_goal = (thi + (thf - thi)) if bias_phi_to_thf else None
    x_s, y_s, phi_s = scqp_center_yaw(x_ref, y_ref, t_s, sm,
                                      iters_scqp=scqp_iters,
                                      phi_end_goal=phi_end_goal,
                                      v0_xy=v0_xy, vT_xy=vT_xy,
                                      a0_xy=a0_xy, aT_xy=aT_xy)

    # post-process to bases/headings/pivots on 500 Hz grid
    bp = BoxParams()
    (t, c, phi, b1, b2, th1, th2, dth, contacts, p1, p2) = _freeze_phi_and_build_heads(
        x_s, y_s, phi_s, T, bp,
        freeze_stop_thr=freeze_stop_thr, freeze_margin_s=freeze_margin_s,
        phi_blend_s=phi_blend_s,
        heading_smooth_s=heading_smooth_s,
        heading_speed_thr=heading_speed_thr,
        heading_hyst_ratio=heading_hyst_ratio,
        anti_parallel_end=anti_parallel_end, end_blend_s=end_blend_s,
        pivot_fwd_offset=pivot_fwd_offset
    )

    # package
    params = dict(
        T=T, t=t,
        center=c, phi=phi,
        base1=b1, base2=b2,
        theta1=th1, theta2=th2,
        dtheta=dth,
        contacts=contacts,
        pivot1=p1, pivot2=p2,
        box=bp,
        start=(xi,yi,thi), goal=(xf,yf,thf),
        opts=dict(
            anti_parallel_end=anti_parallel_end, end_blend_s=end_blend_s,
            freeze_stop_thr=freeze_stop_thr, freeze_margin_s=freeze_margin_s,
            phi_blend_s=phi_blend_s,
            heading_smooth_s=heading_smooth_s,
            heading_speed_thr=heading_speed_thr,
            heading_hyst_ratio=heading_hyst_ratio,
            scqp_iters=scqp_iters,
            w_align=sm.w_align, w_phi_d2=sm.w_phi_d2,
            v0_xy=v0_xy, vT_xy=vT_xy, a0_xy=a0_xy, aT_xy=aT_xy,
            pivot_fwd_offset=pivot_fwd_offset
        )
    )
    return params

def traj(params, t, T=None):
    """
    Sample the base poses at time t.
    Returns ((x1,y1,theta1), (x2,y2,theta2)).
    """
    Ttot = params["T"] if (T is None) else float(T)
    tt   = params["t"]
    N    = len(tt)
    # clamp & index
    if t <= 0: k = 0
    elif t >= Ttot: k = N-1
    else:
        k = int(round((N-1) * (t / Ttot)))
    b1 = params["base1"][k]; b2 = params["base2"][k]
    th1 = params["theta1"][k]; th2 = params["theta2"][k]
    return (float(b1[0]), float(b1[1]), float(th1)), (float(b2[0]), float(b2[1]), float(th2))

def traj_pivot(params, t, T=None):
    """
    Sample the arm pivot poses at time t (offset along each base's heading).
    Returns ((x1_piv, y1_piv, theta1), (x2_piv, y2_piv, theta2)).
    """
    Ttot = params["T"] if (T is None) else float(T)
    tt   = params["t"]; N = len(tt)
    if t <= 0: k = 0
    elif t >= Ttot: k = N-1
    else: k = int(round((N-1) * (t / Ttot)))
    p1 = params["pivot1"][k]; p2 = params["pivot2"][k]
    th1 = params["theta1"][k]; th2 = params["theta2"][k]
    return (float(p1[0]), float(p1[1]), float(th1)), (float(p2[0]), float(p2[1]), float(th2))
