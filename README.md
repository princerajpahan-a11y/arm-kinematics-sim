# 2-Link Arm Kinematics Simulator

A from-scratch Python implementation of forward and inverse kinematics
for a planar 2-link robot arm.

![demo](docs/demo.gif)

## Features so far
- Forward and inverse kinematics (elbow up and down)
- Reachability check and workspace sampling
- Straight-line trajectory animation
- Automated tests (round trip, unreachable points)

## How to run
pip install matplotlib numpy
python animate.py

## Next steps
jacobian and singularity analysis, joint angle plots, joint limits.

