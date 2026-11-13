"""
Fit the HIVE correction's constants to real-world data.

INPUT: Robot Controller log files (logcat / robotControllerLog.txt). Only lines containing
"VisionTuner: " are used; everything else in the log is ignored. They come from:

    RobotLog.d("VisionTuner: " + calibrationSpot + ", " + follower.pose().y() + ", "
               + y + ", " + z + ", " + avgDist);

so each line holds, in order (inches, same frame your code uses):
    spot      - calibrationSpot, label for the taped spot (frames at one spot share a label)
    true_y    - follower.pose().y(): MUST equal the tape-measured y (see note below)
    raw_y     - Limelight y BEFORE your HIVE correction
    raw_z     - Limelight z BEFORE your HIVE correction
    avg_dist  - getBotposeAvgDist() converted to inches

NOTE: follower.pose().y() is only ground truth if you setPose() to the tape-measured pose at each
spot and the robot doesn't move while logging. If odometry has drifted, the fit learns the drift.

MODEL (your sqrt(d^2 - h^2) correction, plus two camera error terms):
    z_fixed = raw_z - z_offset - z_per_dist * avg_dist     <- camera height / pitch errors
    relY    = raw_y - pivot_y
    relZ    = z_fixed - h
    y_est   = pivot_y + sign(relY) * sqrt(relY^2 + relZ^2 - h^2)

The fit finds h, pivot_y, z_offset, z_per_dist that make y_est match true_y best.

USAGE:  python3 fit_hive.py robotControllerLog.txt [more_logs.txt ...]

Get the log with:  adb logcat -d > log.txt   (or download robotControllerLog.txt from the
Robot Controller's Manage page).
"""
import sys

import numpy as np
from scipy.optimize import least_squares

# Your current hand-measured constants: the starting guess and the "before" baseline
START = {"h": 43.95, "pivot_y": 72.0, "z_offset": 0.0, "z_per_dist": 0.0}
NAMES = list(START)


TAG = "VisionTuner: "
FIELDS = ("true_y", "raw_y", "raw_z", "avg_dist")


def load(paths):
    """Pull every VisionTuner line out of the log files; skip lines that don't parse."""
    spots, values, bad = [], [], 0
    for path in paths:
        with open(path, errors="replace") as f:
            for line in f:
                i = line.find(TAG)
                if i < 0:
                    continue
                parts = [p.strip() for p in line[i + len(TAG):].split(",")]
                try:
                    if len(parts) != 5 or not parts[0]:
                        raise ValueError
                    nums = [float(p) for p in parts[1:]]
                    if not all(np.isfinite(nums)):
                        raise ValueError
                except ValueError:
                    bad += 1
                    continue
                spots.append(parts[0])
                values.append(nums)
    if not values:
        sys.exit(f"No usable '{TAG.strip()}' lines found in: {', '.join(paths)}")
    print(f"Loaded {len(values)} frames from {len(set(spots))} spots"
          + (f" (skipped {bad} malformed/NaN lines)" if bad else ""))
    arr = np.array(values)
    cols = {k: arr[:, j] for j, k in enumerate(FIELDS)}
    return np.array(spots), cols


def predict(p, d):
    h, pivot_y, z_offset, z_per_dist = p
    z_fixed = d["raw_z"] - z_offset - z_per_dist * d["avg_dist"]
    rel_y = d["raw_y"] - pivot_y
    rel_z = z_fixed - h
    sq = rel_y**2 + rel_z**2 - h**2
    # Frames with no floor solution get a large error instead of NaN
    return np.where(sq >= 0, pivot_y + np.sign(rel_y) * np.sqrt(np.maximum(sq, 0)), np.nan)


def residuals(p, d):
    r = predict(p, d) - d["true_y"]
    return np.nan_to_num(r, nan=50.0)


def report(label, p, spots, d):
    err = predict(p, d) - d["true_y"]
    print(f"\n{label}: overall RMS error {np.sqrt(np.nanmean(err**2)):.2f} in")
    print(f"  {'spot':<6}{'bias (avg err)':>16}{'noise (std)':>14}{'frames':>8}")
    for s in sorted(set(spots)):
        e = err[spots == s]
        print(f"  {s:<6}{np.nanmean(e):>16.2f}{np.nanstd(e):>14.2f}{len(e):>8}")


def subset(d, mask):
    return {k: v[mask] for k, v in d.items()}


def main(paths):
    spots, d = load(paths)
    p0 = np.array([START[n] for n in NAMES])

    # ---- Hold out ~1/4 of the SPOTS (not frames) to check the fit generalizes ----
    unique = sorted(set(spots))
    if len(unique) < 4:
        sys.exit(f"Only {len(unique)} spots logged - need at least 4 (8-12 recommended) to fit and validate.")
    rng = np.random.default_rng(0)
    test_spots = set(rng.choice(unique, size=max(1, len(unique) // 4), replace=False))
    train = np.array([s not in test_spots for s in spots])

    fit = least_squares(residuals, p0, args=(subset(d, train),))
    p = fit.x

    print("Fitted constants (start -> fitted):")
    for n, a, b in zip(NAMES, p0, p):
        print(f"  {n:<11} {a:8.3f} -> {b:8.3f}")
    print(f"  (z_per_dist {p[3]:.4f} in/in  ~ camera pitch error of {np.degrees(np.arctan(p[3])):.2f} deg)")

    report("BEFORE (hand-measured constants), held-out spots", p0, spots[~train], subset(d, ~train))
    report("AFTER  (fitted constants),        held-out spots", p, spots[~train], subset(d, ~train))
    print("\nHeld-out spots:", ", ".join(sorted(test_spots)))
    print("If AFTER bias is ~0 but noise is still large, the rest is frame-to-frame noise:")
    print("regression can't fix that - smooth the tilt or average in your Kalman filter instead.")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1:])
