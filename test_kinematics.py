import math
import random
from kinematics import *

L1, L2 = 1.0, 0.8

def test_known_pose():
    x, y = forward_kinematics(0, 0, L1, L2)
    assert abs(x - 1.8) < 1e-9 and abs(y) < 1e-9

def test_round_trip():
    random.seed(1)
    for _ in range(1000):
        t1 = random.uniform(-math.pi, math.pi)
        t2 = random.uniform(0.05, math.pi - 0.05) * random.choice([-1, 1])
        x, y = forward_kinematics(t1, t2, L1, L2)
        for elbow in ("up", "down"):
            sol = inverse_kinematics(x, y, L1, L2, elbow)
            assert sol is not None
            x2, y2 = forward_kinematics(sol[0], sol[1], L1, L2)
            assert abs(x - x2) < 1e-9 and abs(y - y2) < 1e-9

def test_unreachable():
    assert inverse_kinematics(3, 0, L1, L2) is None
    assert inverse_kinematics(0.1, 0, L1, L2) is None

def test_singularity():
    assert abs(jacobian_determinant(0.3, 0.0, L1, L2)) < 1e-9

def test_jacobian_matches_numeric():
    t1, t2, h = 0.4, 0.9, 1e-6
    J = jacobian(t1, t2, L1, L2)
    xa, ya = forward_kinematics(t1 + h, t2, L1, L2)
    xb, yb = forward_kinematics(t1 - h, t2, L1, L2)
    assert abs((xa - xb) / (2 * h) - J[0][0]) < 1e-5
    assert abs((ya - yb) / (2 * h) - J[1][0]) < 1e-5    

if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("PASS", name)