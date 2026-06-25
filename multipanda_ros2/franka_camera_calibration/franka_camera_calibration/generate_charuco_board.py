#!/usr/bin/env python3
"""Generate a printable ChArUco board (PNG + PDF) at true physical scale.

The PDF/PNG embed the correct DPI so that, when printed at 100% / "actual
size" (no fit-to-page scaling), each black square measures exactly
``square_length`` metres.  After printing, measure a square with calipers and
put the *measured* values into ``config/charuco_board.yaml`` -- the calibration
accuracy depends directly on these numbers being correct.

Run standalone (no ROS needed):

    ros2 run franka_camera_calibration generate_charuco_board
    # or:  python3 generate_charuco_board.py --squares-x 5 --squares-y 7 ...
"""

import argparse
import os

import cv2
import numpy as np

from franka_camera_calibration.charuco import CharucoParams, CharucoTarget

MM_PER_M = 1000.0
INCH_PER_M = 39.3700787


def build_arg_parser():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--squares-x', type=int, default=5, help='number of squares in X')
    p.add_argument('--squares-y', type=int, default=7, help='number of squares in Y')
    p.add_argument('--square-length', type=float, default=0.04,
                   help='checker square side length [m] (default 0.04 = 40 mm)')
    p.add_argument('--marker-length', type=float, default=0.03,
                   help='aruco marker side length [m] (default 0.03 = 30 mm)')
    p.add_argument('--dictionary', type=str, default='DICT_5X5_1000',
                   help='ArUco dictionary name (default DICT_5X5_1000)')
    p.add_argument('--dpi', type=int, default=300, help='print resolution (default 300)')
    p.add_argument('--margin', type=float, default=0.01,
                   help='white border around the board [m] (default 0.01 = 10 mm)')
    p.add_argument('--output-dir', type=str, default='.',
                   help='where to write the board files')
    p.add_argument('--basename', type=str, default='charuco_board',
                   help='output file base name')
    return p


def _save_pdf(image_bgr, path, dpi):
    """Save a BGR/uint8 image to a single-page PDF at the given DPI.

    Tries Pillow first (embeds DPI -> correct print scale); falls back to a
    minimal hand-written PDF wrapping a PNG if Pillow is unavailable.
    """
    rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    try:
        from PIL import Image
        Image.fromarray(rgb).save(path, 'PDF', resolution=float(dpi))
        return True
    except Exception as exc:  # pragma: no cover - depends on environment
        print(f'[warn] Pillow PDF export failed ({exc}); writing PNG only.')
        return False


def main():
    args = build_arg_parser().parse_args()

    params = CharucoParams(
        squares_x=args.squares_x,
        squares_y=args.squares_y,
        square_length=args.square_length,
        marker_length=args.marker_length,
        dictionary=args.dictionary,
    )
    if params.marker_length >= params.square_length:
        raise SystemExit('marker_length must be smaller than square_length.')

    target = CharucoTarget(params)

    px_per_m = args.dpi * INCH_PER_M
    board_w_px = int(round(params.squares_x * params.square_length * px_per_m))
    board_h_px = int(round(params.squares_y * params.square_length * px_per_m))
    margin_px = int(round(args.margin * px_per_m))

    board_img = target.generate_image((board_w_px, board_h_px), margin_px=margin_px)
    board_bgr = cv2.cvtColor(board_img, cv2.COLOR_GRAY2BGR)

    # Caption strip with the parameters so the printout is self-documenting.
    w_m, h_m = target.physical_size()
    caption = (f'{params.dictionary}  {params.squares_x}x{params.squares_y}  '
               f'square={params.square_length * MM_PER_M:.1f}mm  '
               f'marker={params.marker_length * MM_PER_M:.1f}mm  '
               f'board={w_m * MM_PER_M:.0f}x{h_m * MM_PER_M:.0f}mm  '
               f'@ {args.dpi} DPI - PRINT AT 100% / ACTUAL SIZE')
    strip_h = max(40, int(round(0.012 * px_per_m)))
    strip = np.full((strip_h, board_bgr.shape[1], 3), 255, dtype=np.uint8)
    cv2.putText(strip, caption, (10, int(strip_h * 0.7)),
                cv2.FONT_HERSHEY_SIMPLEX, strip_h / 60.0, (0, 0, 0), 2, cv2.LINE_AA)
    sheet = np.vstack([board_bgr, strip])

    os.makedirs(args.output_dir, exist_ok=True)
    png_path = os.path.join(args.output_dir, args.basename + '.png')
    pdf_path = os.path.join(args.output_dir, args.basename + '.pdf')
    yaml_path = os.path.join(args.output_dir, args.basename + '.yaml')

    # PNG with DPI metadata so image viewers also report true size.
    try:
        from PIL import Image
        Image.fromarray(cv2.cvtColor(sheet, cv2.COLOR_BGR2RGB)).save(
            png_path, dpi=(args.dpi, args.dpi))
    except Exception:
        cv2.imwrite(png_path, sheet)

    have_pdf = _save_pdf(sheet, pdf_path, args.dpi)

    with open(yaml_path, 'w') as f:
        f.write('# ChArUco board parameters - keep in sync with the printed board.\n')
        f.write('# After printing, MEASURE a square and update square_length / '
                'marker_length.\n')
        f.write('charuco_board:\n')
        f.write(f'  squares_x: {params.squares_x}\n')
        f.write(f'  squares_y: {params.squares_y}\n')
        f.write(f'  square_length: {params.square_length}   # metres\n')
        f.write(f'  marker_length: {params.marker_length}   # metres\n')
        f.write(f'  dictionary: {params.dictionary}\n')

    print('Generated ChArUco board:')
    print(f'  PNG  : {png_path}')
    if have_pdf:
        print(f'  PDF  : {pdf_path}   <-- print this at 100% / actual size')
    print(f'  YAML : {yaml_path}')
    print(f'  Board physical size: {w_m * MM_PER_M:.1f} x {h_m * MM_PER_M:.1f} mm')
    print(f'  Square: {params.square_length * MM_PER_M:.1f} mm, '
          f'Marker: {params.marker_length * MM_PER_M:.1f} mm')
    print('\nNEXT: print the PDF without scaling, measure a black square, and put the\n'
          'measured square_length/marker_length into config/charuco_board.yaml.')


if __name__ == '__main__':
    main()
