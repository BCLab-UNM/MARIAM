from coop_traj_api import opt_traj_params, traj
from coop_traj_viz import plot_summary, animate_carry
import time
import numpy as np
import math

#trajectory params
params = opt_traj_params(
    start=[0.0, 0.0, 0.0], 
    goal=[1.0, 0.1, 0.0],
    T=16.0,
    anti_parallel_end=False,
    heading_smooth_s=0.15,
    heading_speed_thr=0.03,
    heading_hyst_ratio=1.5,
    scqp_iters=1,
)

# uncomment the following to generate trajectory, will need ros publisher for pose and twist for each respective base. 
# which columns are angles (adjust if your pose layout differs)
angle_cols = [2]  # e.g., [x, y, yaw]

# ------------ 250 Hz loop with on-the-fly velocities ------------
dt = 1.0 / 250.0
t = 0.0

b1_all, b2_all = [], []
v1_all, v2_all = [], []

last_b1 = None
last_b2 = None

start_wall = time.time()
while t <= params["T"] + 1e-9:
    # sample poses
    b1, b2 = traj(params, t=t, T=params["T"])
    b1 = np.asarray(b1, dtype=float)
    b2 = np.asarray(b2, dtype=float)

    b1_all.append(b1)
    b2_all.append(b2)

    # instantaneous velocities
    if last_b1 is None:
        v1 = np.zeros_like(b1)
        v2 = np.zeros_like(b2)
    else:
        v1 = (b1 - last_b1) / dt
        v2 = (b2 - last_b2) / dt

    v1_all.append(v1)
    v2_all.append(v2)

    # print current sample
    print(f"t={t:.3f}  b1={b1}  v1={v1}   b2={b2}  v2={v2}")

    # shift for next loop
    last_b1, last_b2 = b1, b2

    # (optional) real-time pacing at 250 Hz
    elapsed = time.time() - start_wall
    if elapsed < t:
        time.sleep(t - elapsed)

    t += dt

# arrays
b1_all = np.vstack(b1_all)
b2_all = np.vstack(b2_all)
v1_all = np.vstack(v1_all)
v2_all = np.vstack(v2_all)

print("Samples:", len(b1_all), "dt:", dt)
print("b1 pose/vel shapes:", b1_all.shape, v1_all.shape)
print("b2 pose/vel shapes:", b2_all.shape, v2_all.shape)

# plot summary of trajectories
plot_summary(params, save_prefix="../figures/coop_demo")

animate_carry(params, fps=30, head_len=0.18,
              show_arms=True, show_box=True, show_pivots=True,
              frame_step=25, path_stride=6, save_gif=None)
