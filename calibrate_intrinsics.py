#!/usr/bin/env python3
"""Standalone Intel RealSense camera-intrinsics calibration with a ChArUco board.

Streams the RealSense color image, lets you capture views of a ChArUco board by
pressing a key, then runs OpenCV's camera calibration and writes the intrinsics
(camera matrix + distortion) to a YAML file.

Dependencies:
    pip install pyrealsense2 opencv-contrib-python numpy pyyaml

Print a ChArUco board first (any generator works, e.g. calib.io), then MEASURE a
printed black square and a marker with calipers and pass the real sizes via
--square-length / --marker-length. Calibration accuracy depends on these.

Usage:
    python3 calibrate_intrinsics.py \
        --squares-x 5 --squares-y 7 \
        --square-length 0.040 --marker-length 0.030 \
        --dictionary DICT_5X5_1000 \
        --width 1280 --height 720

Live controls:
    SPACE / c : capture the current view (only accepted if the board is seen)
    u         : undo the last capture
    g         : run calibration now with the captures collected so far
    q / ESC   : quit (auto-calibrates if enough captures exist)
"""

import argparse
import sys
import time

import cv2
import numpy as np
import yaml

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None


# --- OpenCV 4.6 (old) vs 4.7+ (new) aruco API shim --------------------------
_NEW_API = hasattr(cv2.aruco, 'CharucoDetector')


def make_board(args):
    if not hasattr(cv2.aruco, args.dictionary):
        sys.exit(f'Unknown ArUco dictionary: {args.dictionary!r}')
    dict_id = getattr(cv2.aruco, args.dictionary)
    size = (args.squares_x, args.squares_y)
    if _NEW_API:
        dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
        board = cv2.aruco.CharucoBoard(
            size, args.square_length, args.marker_length, dictionary)
        detector = cv2.aruco.CharucoDetector(board)
    else:
        dictionary = cv2.aruco.Dictionary_get(dict_id)
        board = cv2.aruco.CharucoBoard_create(
            args.squares_x, args.squares_y,
            args.square_length, args.marker_length, dictionary)
        detector = None
    return dictionary, board, detector


def detect(gray, dictionary, board, detector):
    """Return (charuco_corners, charuco_ids, marker_corners, marker_ids)."""
    if _NEW_API:
        ch_c, ch_id, mk_c, mk_id = detector.detectBoard(gray)
        return ch_c, ch_id, mk_c, mk_id
    mk_c, mk_id, _ = cv2.aruco.detectMarkers(gray, dictionary)
    if mk_id is None or len(mk_id) == 0:
        return None, None, mk_c, mk_id
    _, ch_c, ch_id = cv2.aruco.interpolateCornersCharuco(mk_c, mk_id, gray, board)
    return ch_c, ch_id, mk_c, mk_id


def calibrate(all_corners, all_ids, board, image_size):
    """Run ChArUco calibration. Returns (rms, K, dist)."""
    if _NEW_API:
        obj_pts, img_pts = [], []
        for c, i in zip(all_corners, all_ids):
            op, ip = board.matchImagePoints(c, i)
            if op is not None and len(op) >= 4:
                obj_pts.append(op)
                img_pts.append(ip)
        flags = 0
        rms, K, dist, _, _ = cv2.calibrateCamera(
            obj_pts, img_pts, image_size, None, None, flags=flags)
        return rms, K, dist
    rms, K, dist, _, _ = cv2.aruco.calibrateCameraCharuco(
        all_corners, all_ids, board, image_size, None, None)
    return rms, K, dist


def open_realsense(args):
    if rs is None:
        sys.exit('pyrealsense2 not installed. pip install pyrealsense2')
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, args.width, args.height,
                         rs.format.bgr8, args.fps)
    profile = pipeline.start(config)
    # Report the factory intrinsics for reference/comparison.
    vsp = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = vsp.get_intrinsics()
    print('--- RealSense factory color intrinsics (for reference) ---')
    print(f'  fx={intr.fx:.2f} fy={intr.fy:.2f} '
          f'cx={intr.ppx:.2f} cy={intr.ppy:.2f}')
    print(f'  model={intr.model} coeffs={list(intr.coeffs)}')
    print('-' * 58)
    return pipeline


def save_yaml(path, K, dist, image_size, rms, n_views, args):
    data = {
        'image_width': int(image_size[0]),
        'image_height': int(image_size[1]),
        'camera_matrix': {
            'rows': 3, 'cols': 3,
            'data': [float(x) for x in K.flatten()],
        },
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': {
            'rows': 1, 'cols': int(dist.size),
            'data': [float(x) for x in dist.flatten()],
        },
        'fx': float(K[0, 0]), 'fy': float(K[1, 1]),
        'cx': float(K[0, 2]), 'cy': float(K[1, 2]),
        'reprojection_error_rms_px': float(rms),
        'num_views_used': int(n_views),
        'board': {
            'squares_x': args.squares_x, 'squares_y': args.squares_y,
            'square_length': args.square_length,
            'marker_length': args.marker_length,
            'dictionary': args.dictionary,
        },
    }
    with open(path, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--squares-x', type=int, default=5)
    p.add_argument('--squares-y', type=int, default=7)
    p.add_argument('--square-length', type=float, default=0.040,
                   help='black square size in metres (MEASURED on the printout)')
    p.add_argument('--marker-length', type=float, default=0.030,
                   help='aruco marker size in metres (MEASURED on the printout)')
    p.add_argument('--dictionary', default='DICT_5X5_1000')
    p.add_argument('--width', type=int, default=1280)
    p.add_argument('--height', type=int, default=720)
    p.add_argument('--fps', type=int, default=30)
    p.add_argument('--min-views', type=int, default=12,
                   help='minimum captures before calibration is allowed')
    p.add_argument('--min-corners', type=int, default=6,
                   help='minimum ChArUco corners for a capture to count')
    p.add_argument('--output', default='realsense_intrinsics.yaml')
    return p.parse_args()


def main():
    args = parse_args()
    dictionary, board, detector = make_board(args)
    pipeline = open_realsense(args)

    all_corners, all_ids = [], []
    image_size = None  # (w, h)

    print('\nControls: [SPACE/c] capture  [u] undo  [g] calibrate now  '
          '[q/ESC] quit\n')
    print(f'Collect at least {args.min_views} views from varied angles, '
          'distances, and board positions (especially the image corners).')

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color = frames.get_color_frame()
            if not color:
                continue
            image = np.asanyarray(color.get_data())
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            image_size = (gray.shape[1], gray.shape[0])

            ch_c, ch_id, mk_c, mk_id = detect(gray, dictionary, board, detector)
            n_corners = 0 if ch_id is None else len(ch_id)

            view = image.copy()
            if mk_id is not None and len(mk_id) > 0:
                cv2.aruco.drawDetectedMarkers(view, mk_c, mk_id)
            if ch_c is not None and ch_id is not None:
                cv2.aruco.drawDetectedCornersCharuco(view, ch_c, ch_id)

            usable = n_corners >= args.min_corners
            color_txt = (0, 255, 0) if usable else (0, 165, 255)
            cv2.putText(view, f'captures: {len(all_corners)}  '
                        f'corners: {n_corners}',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color_txt, 2)
            cv2.putText(view, 'SPACE/c capture  u undo  g calibrate  q quit',
                        (10, image_size[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 255, 255), 2)
            cv2.imshow('RealSense ChArUco intrinsics calibration', view)
            key = cv2.waitKey(1) & 0xFF

            if key in (ord(' '), ord('c')):
                if usable:
                    all_corners.append(ch_c)
                    all_ids.append(ch_id)
                    print(f'  captured view {len(all_corners)} '
                          f'({n_corners} corners)')
                else:
                    print(f'  rejected: only {n_corners} corners '
                          f'(need {args.min_corners})')
            elif key == ord('u'):
                if all_corners:
                    all_corners.pop()
                    all_ids.pop()
                    print(f'  undid last; {len(all_corners)} remain')
            elif key == ord('g'):
                if len(all_corners) >= args.min_views:
                    break
                print(f'  need >= {args.min_views} views, '
                      f'have {len(all_corners)}')
            elif key in (ord('q'), 27):  # q or ESC
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

    if len(all_corners) < args.min_views:
        print(f'\nOnly {len(all_corners)} views captured '
              f'(< {args.min_views}); not calibrating.')
        return 1

    print(f'\nCalibrating from {len(all_corners)} views...')
    t0 = time.time()
    rms, K, dist = calibrate(all_corners, all_ids, board, image_size)
    print(f'Done in {time.time() - t0:.1f}s')
    print(f'\nRMS reprojection error: {rms:.4f} px  '
          '(aim for < 0.5; > 1.0 means recapture)')
    print('Camera matrix K:')
    print(K)
    print(f'Distortion coefficients: {dist.flatten()}')

    save_yaml(args.output, K, dist, image_size, rms, len(all_corners), args)
    print(f'\nWrote {args.output}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
