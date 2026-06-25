#!/usr/bin/env python3
"""Manually capture calibration samples (image + robot pose) by jogging the arm.

You drive the arm yourself (RViz interactive marker + Plan&Execute, or
hand-guiding) so the wrist camera sees the fixed ChArUco board, then press Enter
to capture a sample. Each capture saves:

  * raw image            -> sample_XXX.png        (used by the offline solver)
  * detection overlay    -> sample_XXX_overlay.png (visual sanity check)
  * the gripper pose      -> appended to samples.csv (base_T_gripper)

It also writes ``intrinsics.yaml`` and ``board.yaml`` once, so the whole folder
is self-contained and can be calibrated later, offline, with:

    ros2 run franka_camera_calibration calibrate_from_captures <output_dir>

Run:
    ros2 run franka_camera_calibration capture_samples --ros-args \
        -p output_dir:=/home/developer/wrist_cam_calibration/run1 \
        -p charuco.square_length:=0.025 -p charuco.marker_length:=0.019
"""

import os
import threading

import cv2
import numpy as np

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros

from franka_camera_calibration.charuco import CharucoParams, CharucoTarget

SAMPLES_HEADER = 'index,image,x,y,z,qx,qy,qz,qw\n'


class SampleCapturer(Node):
    def __init__(self):
        super().__init__('sample_capturer')
        p = self.declare_parameter
        p('output_dir', os.path.expanduser('~/wrist_cam_calibration/run1'))
        p('base_frame', 'panda_link0')
        p('gripper_frame', 'panda_link8')   # rigid camera-mount frame (no hand)
        p('image_topic', '/camera/camera/color/image_raw')
        p('camera_info_topic', '/camera/camera/color/camera_info')
        p('min_charuco_corners', 6)
        p('charuco.squares_x', 5)
        p('charuco.squares_y', 7)
        p('charuco.square_length', 0.025)
        p('charuco.marker_length', 0.019)
        p('charuco.dictionary', 'DICT_5X5_1000')

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.output_dir = os.path.expanduser(g('output_dir'))
        self.base_frame = g('base_frame')
        self.gripper_frame = g('gripper_frame')
        self.min_corners = int(g('min_charuco_corners'))

        self.board_params = CharucoParams(
            squares_x=int(g('charuco.squares_x')),
            squares_y=int(g('charuco.squares_y')),
            square_length=float(g('charuco.square_length')),
            marker_length=float(g('charuco.marker_length')),
            dictionary=str(g('charuco.dictionary')))
        self.target = CharucoTarget(self.board_params)

        self.bridge = CvBridge()
        self.latest_image = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.create_subscription(Image, g('image_topic'),
                                 self._image_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, g('camera_info_topic'),
                                 self._info_cb, qos_profile_sensor_data)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        os.makedirs(self.output_dir, exist_ok=True)
        self.samples_csv = os.path.join(self.output_dir, 'samples.csv')
        if not os.path.exists(self.samples_csv):
            with open(self.samples_csv, 'w') as f:
                f.write(SAMPLES_HEADER)
        self.count = self._count_existing()
        self._intrinsics_saved = False
        self._save_board_yaml()

    def _count_existing(self):
        try:
            with open(self.samples_csv) as f:
                return sum(1 for ln in f
                           if ln.strip() and not ln.lstrip().startswith(('#', 'index')))
        except FileNotFoundError:
            return 0

    def _image_cb(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            pass

    def _info_cb(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
            self._save_intrinsics_yaml(msg)

    def _save_intrinsics_yaml(self, msg):
        path = os.path.join(self.output_dir, 'intrinsics.yaml')
        k = list(np.array(msg.k, dtype=float).reshape(-1))
        d = list(np.array(msg.d, dtype=float).reshape(-1))
        with open(path, 'w') as f:
            f.write('# Camera intrinsics captured from camera_info.\n')
            f.write(f'image_width: {msg.width}\n')
            f.write(f'image_height: {msg.height}\n')
            f.write(f'distortion_model: {msg.distortion_model}\n')
            f.write('camera_matrix:\n  rows: 3\n  cols: 3\n')
            f.write('  data: [%s]\n' % ', '.join('%.10g' % v for v in k))
            f.write('distortion_coefficients:\n  rows: 1\n  cols: %d\n' % len(d))
            f.write('  data: [%s]\n' % ', '.join('%.10g' % v for v in d))
        self._intrinsics_saved = True
        self.get_logger().info(f'Saved intrinsics -> {path}')

    def _save_board_yaml(self):
        path = os.path.join(self.output_dir, 'board.yaml')
        bp = self.board_params
        with open(path, 'w') as f:
            f.write('charuco_board:\n')
            f.write(f'  squares_x: {bp.squares_x}\n')
            f.write(f'  squares_y: {bp.squares_y}\n')
            f.write(f'  square_length: {bp.square_length}\n')
            f.write(f'  marker_length: {bp.marker_length}\n')
            f.write(f'  dictionary: {bp.dictionary}\n')

    def _detect(self):
        if self.latest_image is None:
            return None, None
        gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)
        ch_corners, ch_ids, _, _ = self.target.detect(gray)
        n = 0 if ch_ids is None else len(ch_ids)
        return n, (ch_corners, ch_ids)

    def _current_pose(self):
        tf = self.tf_buffer.lookup_transform(
            self.base_frame, self.gripper_frame, rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=2.0))
        t = tf.transform.translation
        q = tf.transform.rotation
        return (t.x, t.y, t.z, q.x, q.y, q.z, q.w)

    def capture_loop(self):
        print('\n=== Sample capturer ===')
        print(f'Output dir : {self.output_dir}  (already has {self.count} samples)')
        print(f'Frames     : {self.base_frame} -> {self.gripper_frame}')
        print('Jog the arm so the camera sees the board, then press Enter to capture.')
        print("Type 'q' then Enter to finish.\n")
        while rclpy.ok():
            entry = input(f'[{self.count} captured] Enter=capture, q=quit: ').strip()
            if entry.lower() == 'q':
                break
            if self.latest_image is None:
                print('  ! No camera image yet; not capturing.')
                continue
            if self.camera_matrix is None:
                print('  ! No camera_info / intrinsics yet; not capturing.')
                continue
            try:
                pose = self._current_pose()
            except Exception as exc:
                print(f'  ! TF lookup failed: {exc}')
                continue

            image = self.latest_image.copy()
            n, det = self._detect()
            if n is None or n < self.min_corners:
                ans = input(f'  ! Only {n} ChArUco corners (< {self.min_corners}). '
                            f'Capture anyway? [y/N]: ').strip().lower()
                if ans != 'y':
                    print('  skipped.')
                    continue

            idx = self.count
            raw_name = f'sample_{idx:03d}.png'
            cv2.imwrite(os.path.join(self.output_dir, raw_name), image)

            overlay = image.copy()
            ch_corners, ch_ids = det if det else (None, None)
            self.target.draw_detection(overlay, ch_corners, ch_ids)
            if ch_ids is not None and n >= 4:
                ok, rvec, tvec = self.target.estimate_pose(
                    ch_corners, ch_ids, self.camera_matrix, self.dist_coeffs)
                if ok:
                    cv2.drawFrameAxes(overlay, self.camera_matrix, self.dist_coeffs,
                                      rvec, tvec, self.board_params.square_length)
            cv2.imwrite(os.path.join(self.output_dir, f'sample_{idx:03d}_overlay.png'),
                        overlay)

            with open(self.samples_csv, 'a') as f:
                f.write('%d,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n'
                        % (idx, raw_name, *pose))
            self.count += 1
            print(f'  captured sample_{idx:03d}  ({n} corners)  '
                  f'xyz=({pose[0]:.3f},{pose[1]:.3f},{pose[2]:.3f})')
        print(f'\nDone. {self.count} samples in {self.output_dir}')
        print('Now run:  ros2 run franka_camera_calibration '
              f'calibrate_from_captures {self.output_dir}')


def main(args=None):
    rclpy.init(args=args)
    node = SampleCapturer()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.capture_loop()
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
