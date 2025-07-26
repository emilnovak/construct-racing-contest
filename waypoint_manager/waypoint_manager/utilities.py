import numpy as np


# https://stackoverflow.com/a/56207565
def euler_from_quat(x, y, z, w):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)

    t2 = np.clip(t2, a_min=-1.0, a_max=1.0)
    Y = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.arctan2(t3, t4)

    return X, Y, Z


def quat_from_euler(roll, pitch, yaw):
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return x, y, z, w


def catmull_rom_spline(P0, P1, P2, P3, n_points=20):
    """
    Compute Catmull-Rom spline points between P1 and P2.
    P0,P1,P2,P3 are (x,y) points.
    Returns n_points sampled positions as numpy array shape (n_points, 2).
    """
    t = np.linspace(0, 1, n_points)[:, None]  # column vector
    t2 = t * t
    t3 = t2 * t

    # Catmull-Rom basis matrix
    # https://en.wikipedia.org/wiki/Centripetal_Catmull%E2%80%93Rom_spline
    # Using uniform parameterization here:
    f1 = -0.5*t3 + t2 - 0.5*t
    f2 =  1.5*t3 - 2.5*t2 + 1.0
    f3 = -1.5*t3 + 2.0*t2 + 0.5*t
    f4 =  0.5*t3 - 0.5*t2

    points = f1*P0 + f2*P1 + f3*P2 + f4*P3
    return points


def uniform_resample(points, spacing=0.1):
    """
    Resample points uniformly by approximate arc length.
    points: Nx2 numpy array
    spacing: desired distance between points
    Returns uniformly spaced points along the path.
    """
    # Compute cumulative distances along points
    deltas = np.diff(points, axis=0)
    seg_lengths = np.hypot(deltas[:,0], deltas[:,1])
    cumulative = np.insert(np.cumsum(seg_lengths), 0, 0)

    total_length = cumulative[-1]
    num_samples = max(int(total_length / spacing), 2)
    uniform_dist = np.linspace(0, total_length, num_samples)

    # Interpolate x and y separately along cumulative distance
    x = np.interp(uniform_dist, cumulative, points[:,0])
    y = np.interp(uniform_dist, cumulative, points[:,1])

    return np.stack([x, y], axis=1)