"""
Quick offline consistency test for the analytic Yaskawa 3R solver vs full 6‑axis IK.

It sweeps a few interpolated Cartesian targets (including negative ranges),
computes J1–J3 analytically, then asks planning.kin_serial.kin_engine
for the full 6‑axis solution at the same pose (with a neutral tool orientation).
Prints per‑sample joint deltas so we can spot J2/J3 flips.
"""

import os
import sys
import numpy as np

# Ensure project root on path when running directly
_this_dir = os.path.dirname(os.path.abspath(__file__))
_project_root = os.path.dirname(_this_dir)
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

from yaskawa_3R import solve_position_3r, fk_orientation_quat
from planning.kin_serial_yaskawa import kin_engine

# Neutral tool orientation (XYZ Euler = 0) for kin_engine
TOOL_EULER_DEG = (0.0, 0.0, 0.0)

# Interpolated test targets (mm)
X_POINTS = np.linspace(-400.0, 600.0, 5)
Y_POINTS = np.linspace(-300.0, 300.0, 4)
Z_POINTS = np.linspace(300.0, 900.0, 3)

def main():
    rows = []
    for x in X_POINTS:
        for y in Y_POINTS:
            for z in Z_POINTS:
                # Analytic J1-J3
                j1_a, j2_a, j3_a = solve_position_3r(x, y, z)

                # Full 6-DOF IK (expects XYZABC, deg)
                ik = kin_engine(x, y, z, *TOOL_EULER_DEG)
                if ik is None:
                    rows.append((x, y, z, None, None, None, "IK_FAIL"))
                    continue

                j1_k, j2_k, j3_k, j4_k, j5_k, j6_k = ik
                rows.append((
                    x, y, z,
                    j1_a, j2_a, j3_a,
                    j1_k, j2_k, j3_k, j4_k, j5_k, j6_k
                ))

    # Report
    print("x   y   z   j1a  j2a  j3a  j1k  j2k  j3k  j4k  j5k  j6k   |Δj1| |Δj2| |Δj3|")
    for r in rows:
        if len(r) == 7:  # IK_FAIL
            x, y, z, *_ = r
            print(f"{x:6.1f} {y:6.1f} {z:6.1f}   ----- IK_FAIL -----")
            continue
        x, y, z, j1a, j2a, j3a, j1k, j2k, j3k, j4k, j5k, j6k = r
        dj1 = abs(j1a - j1k)
        dj2 = abs(j2a - j2k)
        dj3 = abs(j3a - j3k)
        print(f"{x:6.1f} {y:6.1f} {z:6.1f}  {j1a:6.2f} {j2a:6.2f} {j3a:6.2f} "
              f"{j1k:6.2f} {j2k:6.2f} {j3k:6.2f} {j4k:6.2f} {j5k:6.2f} {j6k:6.2f} "
              f" {dj1:5.2f} {dj2:5.2f} {dj3:5.2f}")


if __name__ == "__main__":
    main()
