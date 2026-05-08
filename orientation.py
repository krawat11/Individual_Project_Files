"""Orientation filters for shank pitch estimation.

Three filters are provided:
    complementary_filter  - blends gyro integration with accel-derived angle
    madgwick_filter       - quaternion update with gradient correction
    ekf_filter            - 7-state EKF with quaternion and gyro bias

All return a 1D array of pitch angles in degrees.
"""
import numpy as np


# ----- helpers -------------------------------------------------------------

def pitch_from_accel(accel):
    """Static pitch (deg) from a single accelerometer reading."""
    ax, ay, az = accel
    return np.degrees(np.arctan2(-ax, np.sqrt(ay * ay + az * az)))


def quat_normalise(q):
    """Normalise a quaternion to unit length."""
    return q / np.linalg.norm(q)


def quat_multiply(q1, q2):
    """Hamilton product q1 * q2."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def quat_to_pitch(q):
    """Extract the pitch angle (deg) from a unit quaternion."""
    w, x, y, z = q
    sin_p = np.clip(2.0 * (w * y - z * x), -1.0, 1.0)
    return np.degrees(np.arcsin(sin_p))


def quat_from_accel(accel):
    """Build an initial quaternion that aligns the sensor with gravity."""
    ax, ay, az = accel
    pitch = np.arctan2(-ax, np.sqrt(ay * ay + az * az))
    roll  = np.arctan2(ay, az)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cr, sr = np.cos(roll / 2),  np.sin(roll / 2)
    q = np.array([cp*cr, cp*sr, sp*cr, -sp*sr])
    return quat_normalise(q)


# ----- complementary filter ------------------------------------------------

def complementary_filter(t, accel, gyro, alpha=0.98):
    """Blend gyro integration with the accel-derived static angle.

    pitch[i] = alpha * (pitch[i-1] + gy*dt)  +  (1 - alpha) * pitch_from_accel
    """
    n = len(t)
    pitch = np.zeros(n)
    pitch[0] = pitch_from_accel(accel[0])

    for i in range(1, n):
        dt = t[i] - t[i-1]
        from_gyro  = pitch[i-1] + gyro[i, 1] * dt
        from_accel = pitch_from_accel(accel[i])
        pitch[i] = alpha * from_gyro + (1 - alpha) * from_accel

    return pitch


# ----- Madgwick filter -----------------------------------------------------

def madgwick_filter(t, accel, gyro, beta=0.1):
    """Quaternion gradient-descent filter.

    Each step:
      1. Compute the quaternion derivative from the gyroscope.
      2. Subtract a scaled gradient that pulls the predicted gravity
         direction toward the measured accelerometer direction.
      3. Integrate and renormalise.
    """
    n = len(t)
    q = quat_from_accel(accel[0])
    pitch = np.zeros(n)
    pitch[0] = quat_to_pitch(q)

    for i in range(1, n):
        dt = t[i] - t[i-1]
        gx, gy, gz = np.radians(gyro[i])

        # quaternion derivative from gyro
        q_dot = 0.5 * quat_multiply(q, np.array([0.0, gx, gy, gz]))

        # gradient correction toward measured gravity
        a_norm = np.linalg.norm(accel[i])
        if a_norm > 1e-6:
            ax, ay, az = accel[i] / a_norm
            qw, qx, qy, qz = q

            # error between predicted and measured gravity vector
            f = np.array([
                2 * (qx*qz - qw*qy) - ax,
                2 * (qw*qx + qy*qz) - ay,
                2 * (0.5 - qx*qx - qy*qy) - az,
            ])
            # Jacobian of f with respect to the quaternion
            J = np.array([
                [-2*qy,  2*qz, -2*qw,  2*qx],
                [ 2*qx,  2*qw,  2*qz,  2*qy],
                [  0.0, -4*qx, -4*qy,   0.0],
            ])
            grad = J.T @ f
            grad_n = np.linalg.norm(grad)
            if grad_n > 1e-9:
                q_dot = q_dot - beta * grad / grad_n

        q = quat_normalise(q + q_dot * dt)
        pitch[i] = quat_to_pitch(q)

    return pitch


# ----- Extended Kalman Filter ---------------------------------------------

def ekf_filter(t, accel, gyro, proc_q=1e-4, proc_b=1e-6, meas_r=0.05):
    """7-state EKF with quaternion (4) and gyro bias (3).

    Predict step: integrate the bias-corrected gyro to advance the quaternion.
    Update step:  use the accelerometer (gravity direction) to correct.
    """
    n = len(t)
    q = quat_from_accel(accel[0])
    bias = np.zeros(3)
    P = np.eye(7) * 1e-3

    # process and measurement noise covariances
    Q = np.eye(7)
    Q[:4, :4] *= proc_q
    Q[4:, 4:] *= proc_b
    R = np.eye(3) * meas_r

    pitch = np.zeros(n)
    pitch[0] = quat_to_pitch(q)

    for i in range(1, n):
        dt = t[i] - t[i-1]

        # remove estimated bias from the gyro reading
        wx, wy, wz = np.radians(gyro[i]) - bias

        # ----- predict -----
        # quaternion rate q_dot = 0.5 * Omega(w) * q
        Omega = np.array([
            [  0, -wx, -wy, -wz],
            [ wx,   0,  wz, -wy],
            [ wy, -wz,   0,  wx],
            [ wz,  wy, -wx,   0],
        ])
        q = quat_normalise(q + 0.5 * Omega @ q * dt)

        # state transition Jacobian F
        F = np.eye(7)
        F[:4, :4] += 0.5 * Omega * dt
        qw, qx, qy, qz = q
        Xi = np.array([
            [-qx, -qy, -qz],
            [ qw, -qz,  qy],
            [ qz,  qw, -qx],
            [-qy,  qx,  qw],
        ])
        F[:4, 4:] = -0.5 * Xi * dt

        P = F @ P @ F.T + Q * dt

        # ----- update (only when accel is close to 1 g) -----
        a_norm = np.linalg.norm(accel[i])
        if 0.5 < a_norm < 1.5:
            ax, ay, az = accel[i] / a_norm
            qw, qx, qy, qz = q

            # gravity direction predicted by the current quaternion
            g_pred = np.array([
                2 * (qx*qz - qw*qy),
                2 * (qw*qx + qy*qz),
                qw*qw - qx*qx - qy*qy + qz*qz,
            ])

            # measurement Jacobian (depends on q only)
            H = np.zeros((3, 7))
            H[0, :4] = [-2*qy,  2*qz, -2*qw,  2*qx]
            H[1, :4] = [ 2*qx,  2*qw,  2*qz,  2*qy]
            H[2, :4] = [ 2*qw, -2*qx, -2*qy,  2*qz]

            # standard EKF update
            innovation = np.array([ax, ay, az]) - g_pred
            S = H @ P @ H.T + R
            K = P @ H.T @ np.linalg.inv(S)
            dx = K @ innovation

            q    = quat_normalise(q + dx[:4])
            bias = bias + dx[4:]
            P    = (np.eye(7) - K @ H) @ P

        pitch[i] = quat_to_pitch(q)

    return pitch
