#!/usr/bin/env python3
"""Offline hand-eye calibration from a folder captured by ``capture_samples``.

No robot, MoveIt, or camera needed -- it just reads the saved images + poses +
intrinsics and computes the camera-to-gripper transform. Re-runnable as often
as you like (e.g. to try a different method or after dropping bad samples).

    ros2 run franka_camera_calibration calibrate_from_captures <input_dir> \
        [--method park] [--min-corners 6] [--gripper-frame panda_link8]

The folder must contain: samples.csv, sample_XXX.png, intrinsics.yaml, board.yaml
(all produced by capture_samples).
"""

import argparse
import csv
import os
import sys

import cv2
import numpy as np
import yaml

from franka_camera_calibration.charuco import CharucoParams, CharucoTarget
from franka_camera_calibration.handeye import (
    quat_to_rot,
    solve_handeye,
    write_extrinsics_yaml,
)


def _load_intrinsics(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    K = np.array(data['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
    d = data.get('distortion_coefficients', {}).get('data', [0, 0, 0, 0, 0])
    D = np.array(d, dtype=np.float64).reshape(-1, 1)
    return K, D


def _load_board(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    return CharucoParams.from_dict(data['charuco_board'])


def _load_samples(path):
    rows = []
    with open(path, newline='') as f:
        for r in csv.DictReader(
                ln for ln in f if ln.strip() and not ln.lstrip().startswith('#')):
            rows.append(r)
    return rows


def build_arg_parser():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('input_dir', help='folder produced by capture_samples')
    p.add_argument('--method', default='park',
                   help='tsai | park | horaud | andreff | daniilidis')
    p.add_argument('--min-corners', type=int, default=6,
                   help='minimum ChArUco corners to use a sample')
    p.add_argument('--gripper-frame', default='panda_link8',
                   help='name of the camera-mount frame (for the output transform)')
    p.add_argument('--output', default=None,
                   help='output yaml (default <input_dir>/wrist_camera_extrinsics.yaml)')
    return p


def main(argv=None):
    # Drop ROS args if launched via `ros2 run`.
    if argv is None:
        argv = [a for a in sys.argv[1:] if a != '--ros-args']
    args = build_arg_parser().parse_args(argv)

    d = os.path.expanduser(args.input_dir)
    samples_csv = os.path.join(d, 'samples.csv')
    intr_yaml = os.path.join(d, 'intrinsics.yaml')
    board_yaml = os.path.join(d, 'board.yaml')
    for path in (samples_csv, intr_yaml, board_yaml):
        if not os.path.isfile(path):
            sys.exit(f'Missing required file: {path}')

    K, D = _load_intrinsics(intr_yaml)
    board = _load_board(board_yaml)
    target = CharucoTarget(board)
    samples = _load_samples(samples_csv)
    print(f'Loaded {len(samples)} samples from {samples_csv}')
    print(f'Board: {board.squares_x}x{board.squares_y} '
          f'square={board.square_length} marker={board.marker_length} '
          f'{board.dictionary}')

    R_g2b, t_g2b, R_t2c, t_t2c = [], [], [], []
    used = []
    for row in samples:
        img_path = os.path.join(d, row['image'])
        image = cv2.imread(img_path)
        if image is None:
            print(f'  ! {row["image"]}: could not read; skipping.')
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ch_corners, ch_ids, _, _ = target.detect(gray)
        n = 0 if ch_ids is None else len(ch_ids)
        if n < args.min_corners:
            print(f'  ! {row["image"]}: only {n} corners (< {args.min_corners}); skip.')
            continue
        ok, rvec, tvec = target.estimate_pose(ch_corners, ch_ids, K, D)
        if not ok:
            print(f'  ! {row["image"]}: pose estimation failed; skipping.')
            continue

        Rg = quat_to_rot(float(row['qx']), float(row['qy']),
                         float(row['qz']), float(row['qw']))
        tg = np.array([[float(row['x'])], [float(row['y'])], [float(row['z'])]])
        Rt, _ = cv2.Rodrigues(rvec)
        R_g2b.append(Rg)
        t_g2b.append(tg)
        R_t2c.append(Rt)
        t_t2c.append(np.asarray(tvec).reshape(3, 1))
        used.append(row['image'])
        print(f'  + {row["image"]}: {n} corners, used.')

    print(f'\nUsing {len(used)} / {len(samples)} samples.')
    if len(used) < 3:
        sys.exit('Need >= 3 valid samples for hand-eye calibration.')

    R_x, t_x, residual = solve_handeye(R_g2b, t_g2b, R_t2c, t_t2c, method=args.method)
    out = args.output or os.path.join(d, 'wrist_camera_extrinsics.yaml')
    trans, quat = write_extrinsics_yaml(
        out, args.gripper_frame, R_x, t_x, len(used), args.method, residual)

    print('=' * 60)
    print(f'Camera pose in {args.gripper_frame} (method={args.method}, '
          f'{len(used)} samples):')
    print(f'  translation [m]: {trans[0]:.5f} {trans[1]:.5f} {trans[2]:.5f}')
    print(f'  quaternion xyzw: {quat[0]:.6f} {quat[1]:.6f} {quat[2]:.6f} {quat[3]:.6f}')
    print(f'  consistency residual [m]: {residual:.5f}')
    print('=' * 60)
    print(f'Saved -> {out}')


if __name__ == '__main__':
    main()
