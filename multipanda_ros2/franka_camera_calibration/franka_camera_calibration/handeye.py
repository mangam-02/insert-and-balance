"""Shared hand-eye math: quaternion helpers, the solve, and result I/O.

Used by both the online node (``calibrate_wrist_camera``) and the offline
solver (``calibrate_from_captures``) so they always agree.
"""

import math

import cv2
import numpy as np

HANDEYE_METHODS = {
    'tsai': cv2.CALIB_HAND_EYE_TSAI,
    'park': cv2.CALIB_HAND_EYE_PARK,
    'horaud': cv2.CALIB_HAND_EYE_HORAUD,
    'andreff': cv2.CALIB_HAND_EYE_ANDREFF,
    'daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def quat_to_rot(qx, qy, qz, qw):
    """Quaternion -> 3x3 rotation matrix."""
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n == 0.0:
        raise ValueError('zero-norm quaternion')
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def rot_to_quat(R):
    """3x3 rotation matrix -> (x, y, z, w)."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def _hom(R, t):
    H = np.eye(4)
    H[:3, :3] = R
    H[:3, 3] = np.asarray(t).reshape(3)
    return H


def consistency_residual(R_g2b, t_g2b, R_t2c, t_t2c, R_x, t_x):
    """Mean AX=XB translational residual [m]; large => bad/insufficient poses."""
    n = len(R_t2c)
    if n < 2:
        return float('nan')
    X = _hom(R_x, t_x)
    errs = []
    for i in range(n - 1):
        Bi = _hom(R_g2b[i], t_g2b[i])
        Bj = _hom(R_g2b[i + 1], t_g2b[i + 1])
        A_gripper = np.linalg.inv(Bj) @ Bi
        Ci = _hom(R_t2c[i], t_t2c[i])
        Cj = _hom(R_t2c[i + 1], t_t2c[i + 1])
        A_cam = Cj @ np.linalg.inv(Ci)
        lhs = A_gripper @ X
        rhs = X @ A_cam
        errs.append(np.linalg.norm(lhs[:3, 3] - rhs[:3, 3]))
    return float(np.mean(errs)) if errs else float('nan')


def solve_handeye(R_g2b, t_g2b, R_t2c, t_t2c, method='park'):
    """Run cv2.calibrateHandEye. Returns (R_cam2gripper, t_cam2gripper, residual)."""
    m = HANDEYE_METHODS.get(str(method).lower(), cv2.CALIB_HAND_EYE_PARK)
    R_x, t_x = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, method=m)
    res = consistency_residual(R_g2b, t_g2b, R_t2c, t_t2c, R_x, t_x)
    return R_x, t_x, res


def write_extrinsics_yaml(path, gripper_frame, R_x, t_x, n_samples, method, residual,
                          child_frame='camera_optical_frame'):
    """Write the camera-in-gripper transform to a YAML file."""
    qx, qy, qz, qw = rot_to_quat(R_x)
    t = np.asarray(t_x).reshape(3)
    with open(path, 'w') as f:
        f.write('# Eye-in-hand wrist-camera extrinsics.\n')
        f.write(f'# Transform: camera optical frame expressed in "{gripper_frame}".\n')
        f.write(f'# Method: cv2.calibrateHandEye / {method}, {n_samples} samples.\n')
        f.write('wrist_camera_extrinsics:\n')
        f.write(f'  parent_frame: {gripper_frame}\n')
        f.write(f'  child_frame: {child_frame}\n')
        f.write('  translation:\n')
        f.write(f'    x: {t[0]:.8f}\n    y: {t[1]:.8f}\n    z: {t[2]:.8f}\n')
        f.write('  rotation_quaternion_xyzw:\n')
        f.write(f'    x: {qx:.8f}\n    y: {qy:.8f}\n    z: {qz:.8f}\n    w: {qw:.8f}\n')
        f.write('  consistency_residual: %.8f\n' % residual)
        f.write('# Static TF example:\n')
        f.write(f'#   ros2 run tf2_ros static_transform_publisher '
                f'{t[0]:.6f} {t[1]:.6f} {t[2]:.6f} '
                f'{qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f} '
                f'{gripper_frame} {child_frame}\n')
    return (t[0], t[1], t[2]), (qx, qy, qz, qw)
