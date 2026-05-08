"""Detect foot-strike (IC) and toe-off (TO) events from IMU data.

IC is the peak in vertical acceleration when the foot lands.
TO is the peak in pitch gyroscope rate during the push-off phase.
"""
import numpy as np
from scipy.signal import find_peaks


def detect_initial_contacts(az, fs, height_g=4.0, refractory_s=0.30):
    """Find ICs as peaks in vertical acceleration above a threshold.

    Parameters
    ----------
    az            : vertical acceleration in g
    fs            : sample rate in Hz
    height_g      : minimum peak height in g
    refractory_s  : minimum time between successive peaks in seconds
    """
    min_distance = int(refractory_s * fs)
    peaks, _ = find_peaks(az, height=height_g, distance=min_distance)
    return peaks


def detect_toe_offs(gy, ic_idx, fs, search_window=(0.10, 0.40)):
    """Find TO as the peak in pitch gyro after each IC.

    For every IC, search a window 100 to 400 ms later and pick the index
    where gy reaches its maximum. That maximum corresponds to peak swing
    angular velocity, which marks toe-off.
    """
    start = int(search_window[0] * fs)
    end   = int(search_window[1] * fs)
    n = len(gy)

    to_idx = np.full(len(ic_idx), -1, dtype=int)
    for k, ic in enumerate(ic_idx):
        a = ic + start
        b = min(ic + end, n)
        if b - a < 2:
            continue
        to_idx[k] = a + int(np.argmax(gy[a:b]))
    return to_idx


def detect_events(az, gy, fs, height_g=4.0):
    """Detect IC and TO events. Returns (ic_indices, to_indices)."""
    ic = detect_initial_contacts(az, fs, height_g=height_g)
    to = detect_toe_offs(gy, ic, fs)
    return ic, to
