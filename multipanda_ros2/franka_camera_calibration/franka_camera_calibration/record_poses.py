#!/usr/bin/env python3
"""Interactively record Cartesian poses into a CSV for wrist-camera calibration.

Jog the arm (e.g. drag the interactive marker in RViz and "Plan & Execute", or
hand-guide it) so the wrist camera sees the ChArUco board, then press Enter to
append the current ``pose_link`` pose (in ``base_frame``) to the CSV. Repeat for
8-15 varied viewpoints, then run the calibration with the produced file.

    ros2 run franka_camera_calibration record_poses \
        --ros-args -p output_csv:=/home/developer/wrist_cam_calibration/poses.csv

Optionally checks board visibility live if camera params are set, so you only
record poses where the board is actually detected.
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

CSV_HEADER = 'name,x,y,z,qx,qy,qz,qw\n'


class PoseRecorder(Node):
    def __init__(self):
        super().__init__('pose_recorder')
        p = self.declare_parameter
        p('output_csv', os.path.expanduser('~/wrist_cam_calibration/poses.csv'))
        p('base_frame', 'panda_link0')
        p('pose_link', 'panda_link8')   # arm without hand; use panda_hand_tcp if gripper attached
        p('check_board', True)
        p('image_topic', '/camera/camera/color/image_raw')
        p('camera_info_topic', '/camera/camera/color/camera_info')
        p('charuco.squares_x', 5)
        p('charuco.squares_y', 7)
        p('charuco.square_length', 0.04)
        p('charuco.marker_length', 0.03)
        p('charuco.dictionary', 'DICT_5X5_1000')

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.output_csv = g('output_csv')
        self.base_frame = g('base_frame')
        self.pose_link = g('pose_link')
        self.check_board = bool(g('check_board'))

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.bridge = CvBridge()
        self.latest_image = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.target = None
        if self.check_board:
            self.target = CharucoTarget(CharucoParams(
                squares_x=int(g('charuco.squares_x')),
                squares_y=int(g('charuco.squares_y')),
                square_length=float(g('charuco.square_length')),
                marker_length=float(g('charuco.marker_length')),
                dictionary=str(g('charuco.dictionary'))))
            self.create_subscription(Image, g('image_topic'),
                                     self._image_cb, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, g('camera_info_topic'),
                                     self._info_cb, qos_profile_sensor_data)

        os.makedirs(os.path.dirname(self.output_csv) or '.', exist_ok=True)
        if not os.path.exists(self.output_csv):
            with open(self.output_csv, 'w') as f:
                f.write(CSV_HEADER)
        self.count = self._count_existing()

    def _count_existing(self):
        try:
            with open(self.output_csv) as f:
                return sum(1 for ln in f
                           if ln.strip() and not ln.lstrip().startswith(('#', 'name')))
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

    def _board_corner_count(self):
        if self.latest_image is None:
            return None
        gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)
        _, ch_ids, _, _ = self.target.detect(gray)
        return 0 if ch_ids is None else len(ch_ids)

    def _current_pose(self):
        tf = self.tf_buffer.lookup_transform(
            self.base_frame, self.pose_link, rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=2.0))
        t = tf.transform.translation
        q = tf.transform.rotation
        return (t.x, t.y, t.z, q.x, q.y, q.z, q.w)

    def record_loop(self):
        print('\n=== Pose recorder ===')
        print(f'Writing to: {self.output_csv}  (already has {self.count} poses)')
        print('Jog the arm so the camera sees the board, then press Enter to record.')
        print("Type 'q' then Enter to finish.\n")
        while rclpy.ok():
            entry = input(f'[{self.count} recorded] Enter=record, q=quit: ').strip()
            if entry.lower() == 'q':
                break
            try:
                pose = self._current_pose()
            except Exception as exc:
                print(f'  ! TF lookup failed: {exc}')
                continue

            if self.check_board:
                n = self._board_corner_count()
                if n is None:
                    print('  ! No camera image yet; recording anyway.')
                elif n < 6:
                    ans = input(f'  ! Board barely visible ({n} corners). '
                                f'Record anyway? [y/N]: ').strip().lower()
                    if ans != 'y':
                        print('  skipped.')
                        continue
                else:
                    print(f'  board OK ({n} corners).')

            name = f'p{self.count:03d}'
            with open(self.output_csv, 'a') as f:
                f.write('%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n' % (name, *pose))
            self.count += 1
            print(f'  recorded {name}: '
                  f'xyz=({pose[0]:.3f},{pose[1]:.3f},{pose[2]:.3f})')
        print(f'\nDone. {self.count} poses in {self.output_csv}')


def main(args=None):
    rclpy.init(args=args)
    node = PoseRecorder()
    # Spin in a background thread so TF/image callbacks run while we block on input().
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.record_loop()
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
