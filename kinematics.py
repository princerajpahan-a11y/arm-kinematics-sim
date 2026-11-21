import math

def forward_kinematics(theta1, theta2, L1, L2):
    x = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2)
    y = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)
    return x, y



def joint_positions(theta1, theta2, L1, L2):
    x1 = L1 * math.cos(theta1)
    y1 = L1 * math.sin(theta1)
    x2, y2 = forward_kinematics(theta1, theta2, L1, L2)
    return [(0.0, 0.0), (x1, y1), (x2, y2)]

def is_reachable(x, y, L1, L2):
    r = math.hypot(x, y)
    return abs(L1 - L2) <= r <= (L1 + L2)

def inverse_kinematics(x, y, L1, L2, elbow="up"):
    if not is_reachable(x, y, L1, L2):
        return None
    c2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    c2 = max(-1.0, min(1.0, c2))
    s2 = math.sqrt(1 - c2**2)
    if elbow == "up":
        s2 = -s2
    theta2 = math.atan2(s2, c2)
    k1 = L1 + L2 * c2
    k2 = L2 * s2
    theta1 = math.atan2(y, x) - math.atan2(k2, k1)
    return theta1, theta2

sol = inverse_kinematics(1.2, 0.3, 1.0, 0.8, "up")
print(math.degrees(sol[0]), math.degrees(sol[1]))
print(forward_kinematics(sol[0], sol[1], 1.0, 0.8))

