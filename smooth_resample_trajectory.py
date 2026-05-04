import argparse

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp


def main():
    parser = argparse.ArgumentParser(
        description="Smoothly resample a Franka Cartesian CSV instead of repeating rows."
    )
    parser.add_argument("input_csv", nargs="?", default="exp3b_trajectory.csv")
    parser.add_argument("output_csv", nargs="?", default="exp3b_trajectory_10sec_smooth.csv")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--rate", type=float, default=1000.0)
    args = parser.parse_args()

    df = pd.read_csv(args.input_csv)
    required = ["x", "y", "z", "qw", "qx", "qy", "qz"]
    missing = [name for name in required if name not in df.columns]
    if missing:
        raise ValueError(f"Missing required columns: {missing}")

    steps = int(round(args.duration * args.rate))
    old_t = np.linspace(0.0, args.duration, len(df))
    new_t = np.linspace(0.0, args.duration, steps)

    out = pd.DataFrame()
    for axis in ["x", "y", "z"]:
        out[axis] = np.interp(new_t, old_t, df[axis].to_numpy(float))

    # SciPy uses [qx, qy, qz, qw], while the controller CSV uses qw first.
    quats_xyzw = df[["qx", "qy", "qz", "qw"]].to_numpy(float)
    quats_xyzw = quats_xyzw / np.linalg.norm(quats_xyzw, axis=1, keepdims=True)
    rots = R.from_quat(quats_xyzw)
    slerp = Slerp(old_t, rots)
    new_quats = slerp(new_t).as_quat()

    out["qw"] = new_quats[:, 3]
    out["qx"] = new_quats[:, 0]
    out["qy"] = new_quats[:, 1]
    out["qz"] = new_quats[:, 2]

    # Keep any extra columns smooth too. The diagonal controller ignores these,
    # but preserving them keeps the file compatible with older scripts.
    for col in df.columns:
        if col in out.columns:
            continue
        if np.issubdtype(df[col].dtype, np.number):
            out[col] = np.interp(new_t, old_t, df[col].to_numpy(float))

    out = out[df.columns]
    out.to_csv(args.output_csv, index=False)

    print(f"Input rows:  {len(df)}")
    print(f"Output rows: {len(out)}")
    print(f"Saved:       {args.output_csv}")


if __name__ == "__main__":
    main()
