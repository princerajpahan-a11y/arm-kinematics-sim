import math
import matplotlib.pyplot as plt
from kinematics import line_trajectory, jacobian_determinant

L1, L2 = 1.0, 0.8
path = line_trajectory((1.2, 0.3), (0.3, 1.2), L1, L2, n=80)

steps = []
th1 = []
th2 = []
det = []
for k in range(len(path)):
    steps.append(k)
    th1.append(math.degrees(path[k][0]))
    th2.append(math.degrees(path[k][1]))
    det.append(abs(jacobian_determinant(path[k][0], path[k][1], L1, L2)))

fig, axs = plt.subplots(2, 1, figsize=(6, 6), sharex=True)
axs[0].plot(steps, th1, label="theta1")
axs[0].plot(steps, th2, label="theta2")
axs[0].set_ylabel("angle (deg)")
axs[0].legend()
axs[0].grid(True)
axs[1].plot(steps, det, color="red")
axs[1].set_ylabel("|det J|")
axs[1].set_xlabel("step along path")
axs[1].grid(True)
fig.suptitle("Joint angles and |det J| along the path")
fig.savefig("docs/joint_angles.png", dpi=150)
plt.show()