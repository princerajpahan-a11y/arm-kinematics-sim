import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from kinematics import joint_positions, line_trajectory, workspace_points

L1, L2 = 1.0, 0.8

path = line_trajectory((1.2, 0.3), (0.3, 1.2), L1, L2, n=80)
pts = workspace_points(L1, L2, steps=40)

wx = []
wy = []
for p in pts:
    wx.append(p[0])
    wy.append(p[1])

fig, ax = plt.subplots(figsize=(6, 6))
ax.scatter(wx, wy, s=1, alpha=0.15)
ax.plot([1.2, 0.3], [0.3, 1.2], "r--", lw=1)
arm, = ax.plot([], [], "o-", lw=4, color="navy")
trace, = ax.plot([], [], "g-", lw=1)
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_aspect("equal", adjustable="box")
ax.grid(True)
ax.set_title("2-link arm following a straight line")

trail_x = []
trail_y = []

def update(i):
    t1, t2 = path[i]
    p3 = joint_positions(t1, t2, L1, L2)
    px = []
    py = []
    for p in p3:
        px.append(p[0])
        py.append(p[1])
    arm.set_data(px, py)
    trail_x.append(px[2])
    trail_y.append(py[2])
    trace.set_data(trail_x, trail_y)
    return arm, trace

anim = FuncAnimation(fig, update, frames=len(path), interval=40, repeat=False)
anim.save("docs/demo.gif", writer="pillow", fps=25)
plt.show()