"""Low-pass filtering for IMU channels."""
import numpy as np
from scipy.signal import butter, filtfilt


def butterworth_lowpass(x, fs, cutoff_hz=5.0, order=4):
    """Apply a zero-phase Butterworth low-pass filter to a 1D signal."""
    nyq = 0.5 * fs
    b, a = butter(order, cutoff_hz / nyq, btype="low")
    return filtfilt(b, a, x)


def prefilter(accel, gyro, fs):
    """Filter all six IMU channels. Returns (accel_filtered, gyro_filtered)."""
    accel_f = np.column_stack([butterworth_lowpass(accel[:, i], fs) for i in range(3)])
    gyro_f  = np.column_stack([butterworth_lowpass(gyro[:, i],  fs) for i in range(3)])
    return accel_f, gyro_f
