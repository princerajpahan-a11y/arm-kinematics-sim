import math
import matplotlib.pyplot as plt
from kinematics import joint_positions

L1, L2 = 1.0, 0.8

pts = joint_positions(math.radians(60), math.radians(90), L1, L2)

xs = []
ys = []
for p in pts:
    xs.append(p[0])
    ys.append(p[1])

plt.plot(xs, ys, "o-", linewidth=4)
plt.xlim(-2, 2)
plt.ylim(-2, 2)
plt.gca().set_aspect("equal", adjustable="box")
plt.grid(True)
plt.title("2-link arm")
plt.show()