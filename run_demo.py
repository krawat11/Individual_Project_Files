"""Run the IMU gait analysis pipeline on a CSV file.

Usage:
    python run_demo.py path/to/data.csv [--start 2.0] [--height 4.0]
"""
import argparse
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

from preprocessing import prefilter
from events import detect_events
from orientation import complementary_filter, madgwick_filter, ekf_filter


SAMPLE_RATE_HZ = 100.0
WINDOW_SECONDS = 4.0


def load_csv(path):
    """Load the CSV file. Returns (time, accel, gyro)."""
    data = np.genfromtxt(path, delimiter=",", skip_header=1)
    t = data[:, 0].astype(float)
    t = t - t[0]
    accel = data[:, 1:4].astype(float)
    gyro  = data[:, 4:7].astype(float)
    return t, accel, gyro


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", type=Path, help="path to the IMU CSV file")
    parser.add_argument("--start", type=float, default=2.0,
                        help="start time of the 4 s display window (seconds)")
    parser.add_argument("--height", type=float, default=4.0,
                        help="IC peak threshold in g")
    args = parser.parse_args()

    if not args.csv_path.exists():
        parser.error(f"file not found: {args.csv_path}")

    out_dir = Path(__file__).parent / "figures"
    out_dir.mkdir(exist_ok=True)

    fs = SAMPLE_RATE_HZ

    # load and filter
    t, accel, gyro = load_csv(args.csv_path)
    print(f"Loaded {len(t)} samples ({t[-1]:.1f} s)")

    accel_f, gyro_f = prefilter(accel, gyro, fs)
    az = accel_f[:, 2]
    print(f"az range: {az.min():.2f} to {az.max():.2f} g")

    # detect events
    ic_idx, to_idx = detect_events(az, gyro_f[:, 1], fs, height_g=args.height)
    n_to = int((to_idx >= 0).sum())
    print(f"Detected {len(ic_idx)} ICs and {n_to} TOs (threshold {args.height} g)")

    # orientation filters
    print("Running orientation filters...")
    cf  = complementary_filter(t, accel_f, gyro_f)
    mwf = madgwick_filter(t, accel_f, gyro_f)
    ekf = ekf_filter(t, accel_f, gyro_f)

    # display window
    i0 = int(args.start * fs)
    i1 = min(int((args.start + WINDOW_SECONDS) * fs), len(t))
    if i1 - i0 < int(0.5 * fs):
        parser.error(f"window too short - recording is only {t[-1]:.1f} s")
    tw = t[i0:i1]

    # ---------- Figure 1: az with detected events ----------
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(tw, az[i0:i1], color="black", lw=1.0, label="filtered az")

    ic_in = [i for i in ic_idx if i0 <= i < i1]
    to_in = [i for i in to_idx if i >= 0 and i0 <= i < i1]
    if ic_in:
        ax.plot(t[ic_in], az[ic_in], "v", color="red", markersize=10,
                mec="black", mew=0.5, label="IC")
    if to_in:
        ax.plot(t[to_in], az[to_in], "^", color="green", markersize=10,
                mec="black", mew=0.5, label="TO")
    ax.axhline(args.height, color="grey", linestyle="--", lw=0.8,
               label=f"threshold ({args.height:g} g)")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("az (g)")
    ax.set_title(f"Vertical acceleration with detected events ({WINDOW_SECONDS:.0f} s window)")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right")
    plt.tight_layout()
    plt.savefig(out_dir / "event_detection.png", dpi=150)
    plt.close()

    # ---------- Figure 2: pitch comparison ----------
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(tw, cf[i0:i1],  color="red",   lw=1.3, label="Complementary")
    ax.plot(tw, mwf[i0:i1], color="green", lw=1.3, label="Madgwick")
    ax.plot(tw, ekf[i0:i1], color="blue",  lw=1.3, label="EKF")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Pitch (deg)")
    ax.set_title(f"Shank pitch from three orientation filters ({WINDOW_SECONDS:.0f} s window)")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right")
    plt.tight_layout()
    plt.savefig(out_dir / "pitch_comparison.png", dpi=150)
    plt.close()

    print(f"Saved figures to {out_dir}/")


if __name__ == "__main__":
    main()
