#!/usr/bin/env python3
"""Generate a printable ChArUco board image for camera calibration.

Renders a board with the SAME parameters you pass to calibrate_intrinsics.py and
saves it as a PNG sized for a real sheet of paper at a chosen DPI, so the printed
squares come out the intended physical size.

Dependencies:
    pip install opencv-contrib-python numpy

Usage (defaults: 5x7 board, 40 mm squares / 30 mm markers, A4 @ 300 DPI):
    python3 generate_charuco_board.py --out charuco_board.png

    python3 generate_charuco_board.py \
        --squares-x 5 --squares-y 7 \
        --square-length 0.040 --marker-length 0.030 \
        --dictionary DICT_5X5_1000 \
        --dpi 300 --paper A4 --out charuco_board.png

IMPORTANT: print at 100% / "actual size" (no "fit to page" scaling). After
printing, measure a black square with calipers and feed the MEASURED value to
calibrate_intrinsics.py --square-length.
"""

import argparse
import sys

import cv2
import numpy as np


_NEW_API = hasattr(cv2.aruco, 'CharucoDetector')

# Paper sizes in metres (width, height), portrait.
PAPER = {
    'A4': (0.210, 0.297),
    'A3': (0.297, 0.420),
    'LETTER': (0.2159, 0.2794),
}


def make_board(args):
    if not hasattr(cv2.aruco, args.dictionary):
        sys.exit(f'Unknown ArUco dictionary: {args.dictionary!r}')
    dict_id = getattr(cv2.aruco, args.dictionary)
    size = (args.squares_x, args.squares_y)
    if _NEW_API:
        dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
        board = cv2.aruco.CharucoBoard(
            size, args.square_length, args.marker_length, dictionary)
    else:
        dictionary = cv2.aruco.Dictionary_get(dict_id)
        board = cv2.aruco.CharucoBoard_create(
            args.squares_x, args.squares_y,
            args.square_length, args.marker_length, dictionary)
    return board


def render(board, width_px, height_px, margin_px):
    if _NEW_API:
        return board.generateImage((width_px, height_px),
                                   marginSize=margin_px, borderBits=1)
    return board.draw((width_px, height_px),
                      marginSize=margin_px, borderBits=1)


def m_to_px(metres, dpi):
    return int(round(metres / 0.0254 * dpi))


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--squares-x', type=int, default=5)
    p.add_argument('--squares-y', type=int, default=7)
    p.add_argument('--square-length', type=float, default=0.040,
                   help='black square size in metres')
    p.add_argument('--marker-length', type=float, default=0.030,
                   help='aruco marker size in metres (< square-length)')
    p.add_argument('--dictionary', default='DICT_5X5_1000')
    p.add_argument('--dpi', type=int, default=300, help='print resolution')
    p.add_argument('--paper', default='A4', choices=list(PAPER) + ['NONE'],
                   help='paper to center the board on; NONE = tight crop')
    p.add_argument('--margin', type=float, default=0.005,
                   help='white border around the board in metres (NONE paper)')
    p.add_argument('--out', default='charuco_board.png')
    return p.parse_args()


def main():
    args = parse_args()
    if args.marker_length >= args.square_length:
        sys.exit('marker-length must be smaller than square-length.')

    board = make_board(args)

    board_w_m = args.squares_x * args.square_length
    board_h_m = args.squares_y * args.square_length
    board_w_px = m_to_px(board_w_m, args.dpi)
    board_h_px = m_to_px(board_h_m, args.dpi)

    if args.paper == 'NONE':
        margin_px = m_to_px(args.margin, args.dpi)
        img = render(board, board_w_px + 2 * margin_px,
                     board_h_px + 2 * margin_px, margin_px)
    else:
        paper_w_m, paper_h_m = PAPER[args.paper]
        if board_w_m > paper_w_m or board_h_m > paper_h_m:
            print(f'WARNING: board {board_w_m*1000:.0f}x{board_h_m*1000:.0f} mm '
                  f'is larger than {args.paper} '
                  f'{paper_w_m*1000:.0f}x{paper_h_m*1000:.0f} mm; '
                  'it will be clipped when printed.')
        board_img = render(board, board_w_px, board_h_px, 0)
        page_w_px = m_to_px(paper_w_m, args.dpi)
        page_h_px = m_to_px(paper_h_m, args.dpi)
        img = np.full((page_h_px, page_w_px), 255, dtype=np.uint8)
        y0 = max((page_h_px - board_h_px) // 2, 0)
        x0 = max((page_w_px - board_w_px) // 2, 0)
        h = min(board_h_px, page_h_px)
        w = min(board_w_px, page_w_px)
        img[y0:y0 + h, x0:x0 + w] = board_img[:h, :w]

    cv2.imwrite(args.out, img)

    print(f'Wrote {args.out}  ({img.shape[1]}x{img.shape[0]} px @ {args.dpi} DPI)')
    print(f'Board: {args.squares_x}x{args.squares_y} squares, '
          f'{args.square_length*1000:.1f} mm squares / '
          f'{args.marker_length*1000:.1f} mm markers, {args.dictionary}')
    print(f'Physical size: {board_w_m*1000:.1f} x {board_h_m*1000:.1f} mm')
    print('Print at 100% / actual size (NO fit-to-page), then measure a square '
          'with calipers before calibrating.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
