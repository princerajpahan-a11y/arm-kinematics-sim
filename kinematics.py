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

