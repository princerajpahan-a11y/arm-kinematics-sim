import math
import matplotlib.pyplot as plt
from kinematics import forward_kinematics, jacobian_determinant

L1, L2 = 1.0, 0.8
steps = 80

xs = []
ys = []
ds = []
for i in range(steps):
    t1 = -math.pi + 2 * math.pi * i / (steps - 1)
    for j in range(steps):
        t2 = -math.pi + 2 * math.pi * j / (steps - 1)
        x, y = forward_kinematics(t1, t2, L1, L2)
        xs.append(x)
        ys.append(y)
        ds.append(abs(jacobian_determinant(t1, t2, L1, L2)))

fig, ax = plt.subplots(figsize=(6, 5))
sc = ax.scatter(xs, ys, c=ds, s=4, cmap="viridis")
fig.colorbar(sc, label="|det J|")
ax.set_aspect("equal", adjustable="box")
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.grid(True)
ax.set_title("Workspace coloured by |det J|")
fig.savefig("docs/singularity_map.png", dpi=150)
plt.show()