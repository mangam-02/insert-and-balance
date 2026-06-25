"""ChArUco board helpers that work across OpenCV 4.6 (old API) and 4.7+ (new API).

The cv2.aruco interface was reworked in OpenCV 4.7.  This module hides the
difference behind a small ``CharucoTarget`` class used by both the board
generator and the calibration node, so the rest of the package never has to
branch on the OpenCV version.
"""

from dataclasses import dataclass

import cv2

# True for OpenCV >= 4.7 (CharucoDetector / object-oriented API present).
_NEW_API = hasattr(cv2.aruco, 'CharucoDetector')


def dictionary_from_name(name: str):
    """Resolve a predefined dictionary by name, e.g. 'DICT_5X5_1000'."""
    if not hasattr(cv2.aruco, name):
        raise ValueError(f'Unknown ArUco dictionary: {name!r}')
    dict_id = getattr(cv2.aruco, name)
    if _NEW_API:
        return cv2.aruco.getPredefinedDictionary(dict_id)
    return cv2.aruco.Dictionary_get(dict_id)


@dataclass
class CharucoParams:
    squares_x: int
    squares_y: int
    square_length: float   # metres
    marker_length: float   # metres
    dictionary: str        # e.g. 'DICT_5X5_1000'

    @classmethod
    def from_dict(cls, d: dict) -> 'CharucoParams':
        return cls(
            squares_x=int(d['squares_x']),
            squares_y=int(d['squares_y']),
            square_length=float(d['square_length']),
            marker_length=float(d['marker_length']),
            dictionary=str(d['dictionary']),
        )


class CharucoTarget:
    """A ChArUco board plus detection/pose-estimation, version independent."""

    def __init__(self, params: CharucoParams):
        self.params = params
        self.dictionary = dictionary_from_name(params.dictionary)
        size = (params.squares_x, params.squares_y)
        if _NEW_API:
            self.board = cv2.aruco.CharucoBoard(
                size, params.square_length, params.marker_length, self.dictionary)
            self.detector = cv2.aruco.CharucoDetector(self.board)
        else:
            self.board = cv2.aruco.CharucoBoard_create(
                params.squares_x, params.squares_y,
                params.square_length, params.marker_length, self.dictionary)
            self.detector = None

    # -- image generation ---------------------------------------------------
    def generate_image(self, out_size_px, margin_px=0, border_bits=1):
        """Render the board to a single-channel uint8 image."""
        if _NEW_API:
            return self.board.generateImage(out_size_px, marginSize=margin_px,
                                            borderBits=border_bits)
        return self.board.draw(out_size_px, marginSize=margin_px,
                               borderBits=border_bits)

    # -- detection ----------------------------------------------------------
    def detect(self, gray):
        """Detect ChArUco corners in a grayscale image.

        Returns ``(charuco_corners, charuco_ids, marker_corners, marker_ids)``
        where ``charuco_corners``/``charuco_ids`` are ``None`` when nothing
        usable is found.
        """
        if _NEW_API:
            ch_corners, ch_ids, mk_corners, mk_ids = self.detector.detectBoard(gray)
            return ch_corners, ch_ids, mk_corners, mk_ids

        mk_corners, mk_ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary)
        if mk_ids is None or len(mk_ids) == 0:
            return None, None, mk_corners, mk_ids
        _, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
            mk_corners, mk_ids, gray, self.board)
        return ch_corners, ch_ids, mk_corners, mk_ids

    def estimate_pose(self, charuco_corners, charuco_ids, camera_matrix, dist_coeffs):
        """Estimate board pose in the camera frame (target_T_cam).

        Returns ``(ok, rvec, tvec)``.  ``rvec``/``tvec`` follow the OpenCV
        convention: they transform points from the board frame into the
        camera frame.
        """
        if charuco_corners is None or charuco_ids is None or len(charuco_ids) < 4:
            return False, None, None

        if _NEW_API:
            obj_pts, img_pts = self.board.matchImagePoints(charuco_corners, charuco_ids)
            if obj_pts is None or len(obj_pts) < 4:
                return False, None, None
            ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, camera_matrix, dist_coeffs)
            return bool(ok), rvec, tvec

        ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            charuco_corners, charuco_ids, self.board, camera_matrix, dist_coeffs,
            None, None)
        return bool(ok), rvec, tvec

    def draw_detection(self, image, charuco_corners, charuco_ids):
        if charuco_corners is not None and charuco_ids is not None:
            cv2.aruco.drawDetectedCornersCharuco(image, charuco_corners, charuco_ids)
        return image

    def physical_size(self):
        """(width_m, height_m) of the printed board, markers excluded margin."""
        return (self.params.squares_x * self.params.square_length,
                self.params.squares_y * self.params.square_length)
