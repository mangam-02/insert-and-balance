#!/usr/bin/env python3
"""Validate an eye-in-hand calibration using the captured samples.

The ChArUco board is fixed in the world, so for each sample the estimated board
pose in the robot base frame,

    base_T_board(i) = base_T_gripper(i) @ gripper_T_cam @ cam_T_board(i)

should be identical for every pose. This tool computes it for all samples and
reports how much they disagree (translation in mm, rotation in deg). A tight
spread => good calibration; a large spread => bad extrinsics, board moved, or
poor detections.

    ros2 run franka_camera_calibration validate_calibration <input_dir> \
        [--extrinsics <yaml>] [--min-corners 6]

Uses the same folder layout produced by capture_samples, plus the
wrist_camera_extrinsics.yaml written by calibrate_from_captures.
"""

import argparse
import math
import os
import sys

import cv2
import numpy as np
import yaml

from franka_camera_calibration.charuco import CharucoTarget
from franka_camera_calibration.calibrate_from_captures import (
    _load_board,
    _load_intrinsics,
    _load_samples,
)
from franka_camera_calibration.handeye import quat_to_rot, rot_to_quat


def _hom(R, t):
    H = np.eye(4)
    H[:3, :3] = R
    H[:3, 3] = np.asarray(t).reshape(3)
    return H


def _load_extrinsics(path):
    with open(path) as f:
        data = yaml.safe_load(f)['wrist_camera_extrinsics']
    t = data['translation']
    q = data['rotation_quaternion_xyzw']
    R = quat_to_rot(q['x'], q['y'], q['z'], q['w'])
    tv = np.array([t['x'], t['y'], t['z']])
    return _hom(R, tv), data.get('parent_frame', 'gripper')


def _mean_quaternion(quats):
    """Markley average of unit quaternions (xyzw). Returns xyzw."""
    A = np.zeros((4, 4))
    for q in quats:
        v = np.array(q, dtype=float)
        v = v / np.linalg.norm(v)
        A += np.outer(v, v)
    w, V = np.linalg.eigh(A)
    return V[:, np.argmax(w)]


def _quat_angle_deg(q1, q2):
    d = abs(float(np.dot(np.array(q1) / np.linalg.norm(q1),
                         np.array(q2) / np.linalg.norm(q2))))
    d = min(1.0, max(-1.0, d))
    return math.degrees(2.0 * math.acos(d))


def build_arg_parser():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('input_dir', help='folder produced by capture_samples')
    p.add_argument('--extrinsics', default=None,
                   help='extrinsics yaml (default <input_dir>/wrist_camera_extrinsics.yaml)')
    p.add_argument('--min-corners', type=int, default=6)
    return p


def main(argv=None):
    if argv is None:
        argv = [a for a in sys.argv[1:] if a != '--ros-args']
    args = build_arg_parser().parse_args(argv)

    d = os.path.expanduser(args.input_dir)
    extr_path = args.extrinsics or os.path.join(d, 'wrist_camera_extrinsics.yaml')
    for path in (os.path.join(d, 'samples.csv'), os.path.join(d, 'intrinsics.yaml'),
                 os.path.join(d, 'board.yaml'), extr_path):
        if not os.path.isfile(path):
            sys.exit(f'Missing required file: {path}')

    K, D = _load_intrinsics(os.path.join(d, 'intrinsics.yaml'))
    target = CharucoTarget(_load_board(os.path.join(d, 'board.yaml')))
    samples = _load_samples(os.path.join(d, 'samples.csv'))
    gripper_T_cam, gripper_frame = _load_extrinsics(extr_path)

    board_positions = []
    board_quats = []
    names = []
    for row in samples:
        image = cv2.imread(os.path.join(d, row['image']))
        if image is None:
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ch_corners, ch_ids, _, _ = target.detect(gray)
        n = 0 if ch_ids is None else len(ch_ids)
        if n < args.min_corners:
            continue
        ok, rvec, tvec = target.estimate_pose(ch_corners, ch_ids, K, D)
        if not ok:
            continue
        Rg = quat_to_rot(float(row['qx']), float(row['qy']),
                         float(row['qz']), float(row['qw']))
        tg = [float(row['x']), float(row['y']), float(row['z'])]
        cam_T_board = _hom(cv2.Rodrigues(rvec)[0], tvec)
        base_T_board = _hom(Rg, tg) @ gripper_T_cam @ cam_T_board
        board_positions.append(base_T_board[:3, 3])
        board_quats.append(rot_to_quat(base_T_board[:3, :3]))
        names.append(row['image'])

    m = len(board_positions)
    if m < 2:
        sys.exit('Need >= 2 valid samples to validate.')

    P = np.array(board_positions)
    mean_p = P.mean(axis=0)
    q_mean = _mean_quaternion(board_quats)

    print(f'Validated against board pose (in {gripper_frame}-based '
          f'base frame), {m} samples.\n')
    print(f'  Estimated board position [m]: '
          f'{mean_p[0]:.4f} {mean_p[1]:.4f} {mean_p[2]:.4f}')
    print('  Per-sample deviation from the mean board pose:')
    print('    %-22s %10s %10s' % ('sample', 'trans[mm]', 'rot[deg]'))
    trans_errs, rot_errs = [], []
    for name, p, q in zip(names, board_positions, board_quats):
        te = np.linalg.norm(p - mean_p) * 1000.0
        re = _quat_angle_deg(q, q_mean)
        trans_errs.append(te)
        rot_errs.append(re)
        print('    %-22s %10.2f %10.3f' % (name, te, re))

    trans_errs = np.array(trans_errs)
    rot_errs = np.array(rot_errs)
    rms_t = float(np.sqrt(np.mean(trans_errs ** 2)))
    rms_r = float(np.sqrt(np.mean(rot_errs ** 2)))
    print('\n  ' + '-' * 44)
    print('  Translation spread: RMS %.2f mm, max %.2f mm'
          % (rms_t, trans_errs.max()))
    print('  Rotation    spread: RMS %.3f deg, max %.3f deg'
          % (rms_r, rot_errs.max()))

    if rms_t < 3.0 and rms_r < 0.5:
        verdict = 'EXCELLENT'
    elif rms_t < 8.0 and rms_r < 1.5:
        verdict = 'GOOD'
    elif rms_t < 20.0 and rms_r < 3.0:
        verdict = 'FAIR (usable; consider more/varied poses)'
    else:
        verdict = 'POOR (recalibrate: more tilt variety, better lighting, check board size)'
    print(f'  Verdict: {verdict}')
    print('\n  Tip: a single bad sample inflates both numbers -- check the worst')
    print('  rows above and its sample_XXX_overlay.png, drop it, and recalibrate.')


if __name__ == '__main__':
    main()
