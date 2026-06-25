#!/usr/bin/env python3
"""Eye-in-hand wrist-camera extrinsic calibration for the Franka Panda.

Workflow
--------
1. Read a CSV of Cartesian poses (panda_hand_tcp expressed in panda_link0).
2. For each pose: command the arm there through the running MoveIt
   ``/move_group``, let it settle, grab one camera image, detect the ChArUco
   board, and record both the board pose in the camera frame (target_T_cam)
   and the robot pose base->gripper (read live from TF).
3. After the last pose, run ``cv2.calibrateHandEye`` to obtain the camera pose
   relative to the gripper/flange frame, and save it to YAML.

Prerequisites (in separate terminals):
    ros2 launch franka_moveit_config moveit.launch.py robot_ip:=172.16.0.2 use_rviz:=true
    # RealSense driver publishing the configured image + camera_info topics
"""

import csv
import math
import os

import cv2
import numpy as np

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros

from franka_camera_calibration.charuco import CharucoParams, CharucoTarget
from franka_camera_calibration.move_group_client import MoveGroupClient

# OpenCV hand-eye method names -> enum.
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


class WristCameraCalibrator(Node):
    def __init__(self):
        super().__init__('wrist_camera_calibrator')

        # -- parameters -----------------------------------------------------
        p = self.declare_parameter
        p('poses_csv', '')
        p('image_topic', '/camera/camera/color/image_raw')
        p('camera_info_topic', '/camera/camera/color/camera_info')
        p('camera_info_yaml', '')          # optional fallback for intrinsics
        p('planning_group', 'panda_manipulator')
        p('base_frame', 'panda_link0')
        p('pose_link', 'panda_hand_tcp')   # link the CSV poses refer to
        p('gripper_frame', 'panda_hand')   # rigid camera-mount frame -> result is cam in here
        p('settle_time', 2.0)
        p('vel_scale', 0.1)
        p('acc_scale', 0.1)
        p('position_tolerance', 0.005)
        p('orientation_tolerance', 0.01)
        p('handeye_method', 'park')
        p('min_charuco_corners', 6)
        p('output_dir', os.path.expanduser('~/wrist_cam_calibration'))
        # ChArUco board (defaults match generate_charuco_board defaults).
        p('charuco.squares_x', 5)
        p('charuco.squares_y', 7)
        p('charuco.square_length', 0.04)
        p('charuco.marker_length', 0.03)
        p('charuco.dictionary', 'DICT_5X5_1000')

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.poses_csv = g('poses_csv')
        self.image_topic = g('image_topic')
        self.camera_info_topic = g('camera_info_topic')
        self.camera_info_yaml = g('camera_info_yaml')
        self.planning_group = g('planning_group')
        self.base_frame = g('base_frame')
        self.pose_link = g('pose_link')
        self.gripper_frame = g('gripper_frame')
        self.settle_time = float(g('settle_time'))
        self.min_corners = int(g('min_charuco_corners'))
        self.handeye_method = str(g('handeye_method')).lower()
        self.output_dir = g('output_dir')

        board_params = CharucoParams(
            squares_x=int(g('charuco.squares_x')),
            squares_y=int(g('charuco.squares_y')),
            square_length=float(g('charuco.square_length')),
            marker_length=float(g('charuco.marker_length')),
            dictionary=str(g('charuco.dictionary')),
        )
        self.target = CharucoTarget(board_params)

        # -- camera I/O -----------------------------------------------------
        self.bridge = CvBridge()
        self.latest_image = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.create_subscription(Image, self.image_topic,
                                 self._image_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.camera_info_topic,
                                 self._info_cb, qos_profile_sensor_data)
        if self.camera_info_yaml:
            self._load_intrinsics_yaml(self.camera_info_yaml)

        # -- TF -------------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # -- MoveIt ---------------------------------------------------------
        self.mover = MoveGroupClient(
            self, group_name=self.planning_group, base_frame=self.base_frame,
            ee_link=self.pose_link,
            position_tolerance=float(g('position_tolerance')),
            orientation_tolerance=float(g('orientation_tolerance')),
            vel_scale=float(g('vel_scale')), acc_scale=float(g('acc_scale')))

        # -- accumulators for hand-eye --------------------------------------
        self.R_gripper2base = []
        self.t_gripper2base = []
        self.R_target2cam = []
        self.t_target2cam = []

    # ---- callbacks --------------------------------------------------------
    def _image_cb(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge failed: {exc}')

    def _info_cb(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
            self.get_logger().info('Got camera intrinsics from camera_info.')

    def _load_intrinsics_yaml(self, path):
        import yaml
        with open(path) as f:
            data = yaml.safe_load(f)
        cm = data['camera_matrix']['data']
        self.camera_matrix = np.array(cm, dtype=np.float64).reshape(3, 3)
        dc = data.get('distortion_coefficients', {}).get('data', [0, 0, 0, 0, 0])
        self.dist_coeffs = np.array(dc, dtype=np.float64).reshape(-1, 1)
        self.get_logger().info(f'Loaded camera intrinsics from {path}.')

    # ---- helpers ----------------------------------------------------------
    def _spin_for(self, seconds):
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _wait_for_intrinsics(self, timeout=10.0):
        end = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while rclpy.ok() and self.camera_matrix is None:
            if self.get_clock().now().nanoseconds > end:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return True

    def _lookup_gripper_in_base(self):
        """Return (R, t) of gripper_frame expressed in base_frame (base_T_gripper)."""
        tf = self.tf_buffer.lookup_transform(
            self.base_frame, self.gripper_frame, rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=2.0))
        q = tf.transform.rotation
        tr = tf.transform.translation
        R = quat_to_rot(q.x, q.y, q.z, q.w)
        t = np.array([[tr.x], [tr.y], [tr.z]], dtype=np.float64)
        return R, t

    def _capture_and_detect(self, idx):
        """Grab the freshest image and detect the board. Returns (ok, rvec, tvec, vis)."""
        # Drain stale frames so we use one taken after motion settled.
        self.latest_image = None
        end = self.get_clock().now().nanoseconds + int(2.0 * 1e9)
        while rclpy.ok() and self.latest_image is None:
            if self.get_clock().now().nanoseconds > end:
                self.get_logger().warn(f'pose {idx}: no image received.')
                return False, None, None, None
            rclpy.spin_once(self, timeout_sec=0.05)

        image = self.latest_image.copy()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ch_corners, ch_ids, _, _ = self.target.detect(gray)
        n = 0 if ch_ids is None else len(ch_ids)
        if n < self.min_corners:
            self.get_logger().warn(
                f'pose {idx}: only {n} ChArUco corners (< {self.min_corners}); skipping.')
            return False, None, None, image
        ok, rvec, tvec = self.target.estimate_pose(
            ch_corners, ch_ids, self.camera_matrix, self.dist_coeffs)
        vis = self.target.draw_detection(image, ch_corners, ch_ids)
        if ok:
            cv2.drawFrameAxes(vis, self.camera_matrix, self.dist_coeffs,
                              rvec, tvec, self.target.params.square_length)
            self.get_logger().info(f'pose {idx}: board detected with {n} corners.')
        return ok, rvec, tvec, vis

    # ---- main routine -----------------------------------------------------
    def run(self):
        if not self.poses_csv or not os.path.isfile(self.poses_csv):
            self.get_logger().error(f'poses_csv not found: {self.poses_csv!r}')
            return 1
        poses = load_poses_csv(self.poses_csv)
        if not poses:
            self.get_logger().error('No poses parsed from CSV.')
            return 1
        self.get_logger().info(f'Loaded {len(poses)} poses from {self.poses_csv}.')

        os.makedirs(self.output_dir, exist_ok=True)

        self.get_logger().info('Waiting for /move_group action server...')
        if not self.mover.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('move_group action server not available. '
                                    'Is moveit.launch.py running?')
            return 1

        if not self._wait_for_intrinsics(timeout=10.0):
            self.get_logger().error(
                'No camera intrinsics. Check camera_info_topic or set camera_info_yaml.')
            return 1

        captured = 0
        for idx, (name, pose) in enumerate(poses):
            ps = PoseStamped()
            ps.header.frame_id = self.base_frame
            ps.pose = pose
            self.get_logger().info(f'[{idx + 1}/{len(poses)}] moving to "{name}"...')
            ok, code = self.mover.move_to_pose(ps)
            if not ok:
                self.get_logger().warn(f'pose {idx} ({name}): motion failed '
                                       f'(error_code={code}); skipping.')
                continue

            self._spin_for(self.settle_time)

            det_ok, rvec, tvec, vis = self._capture_and_detect(idx)
            if vis is not None:
                cv2.imwrite(os.path.join(self.output_dir, f'pose_{idx:03d}_{name}.png'), vis)
            if not det_ok:
                continue

            try:
                Rg, tg = self._lookup_gripper_in_base()
            except Exception as exc:
                self.get_logger().warn(f'pose {idx}: TF lookup failed ({exc}); skipping.')
                continue

            Rt, _ = cv2.Rodrigues(rvec)
            self.R_gripper2base.append(Rg)
            self.t_gripper2base.append(tg)
            self.R_target2cam.append(Rt)
            self.t_target2cam.append(tvec.reshape(3, 1))
            captured += 1

        self.get_logger().info(f'Captured {captured} valid samples.')
        if captured < 3:
            self.get_logger().error('Need >= 3 valid samples for hand-eye calibration.')
            return 1

        return self._compute_and_save()

    def _compute_and_save(self):
        method = HANDEYE_METHODS.get(self.handeye_method, cv2.CALIB_HAND_EYE_PARK)
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            self.R_gripper2base, self.t_gripper2base,
            self.R_target2cam, self.t_target2cam, method=method)

        qx, qy, qz, qw = rot_to_quat(R_cam2gripper)
        t = t_cam2gripper.reshape(3)

        rms = self._reprojection_self_check(R_cam2gripper, t_cam2gripper)

        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Camera pose in {self.gripper_frame} frame '
                               f'(method={self.handeye_method}):')
        self.get_logger().info(f'  translation [m]: {t[0]:.5f} {t[1]:.5f} {t[2]:.5f}')
        self.get_logger().info(f'  quaternion xyzw: {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}')
        self.get_logger().info(f'  AX=XB consistency residual: {rms:.5f}')
        self.get_logger().info('=' * 60)

        out = os.path.join(self.output_dir, 'wrist_camera_extrinsics.yaml')
        with open(out, 'w') as f:
            f.write('# Eye-in-hand wrist-camera extrinsics.\n')
            f.write(f'# Transform: camera optical frame expressed in '
                    f'"{self.gripper_frame}".\n')
            f.write(f'# Method: cv2.calibrateHandEye / {self.handeye_method}, '
                    f'{len(self.R_target2cam)} samples.\n')
            f.write('wrist_camera_extrinsics:\n')
            f.write(f'  parent_frame: {self.gripper_frame}\n')
            f.write('  child_frame: camera_optical_frame\n')
            f.write('  translation:\n')
            f.write(f'    x: {t[0]:.8f}\n    y: {t[1]:.8f}\n    z: {t[2]:.8f}\n')
            f.write('  rotation_quaternion_xyzw:\n')
            f.write(f'    x: {qx:.8f}\n    y: {qy:.8f}\n    z: {qz:.8f}\n    w: {qw:.8f}\n')
            f.write('  consistency_residual: %.8f\n' % rms)
            f.write('# Static TF example:\n')
            f.write(f'#   ros2 run tf2_ros static_transform_publisher '
                    f'{t[0]:.6f} {t[1]:.6f} {t[2]:.6f} '
                    f'{qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f} '
                    f'{self.gripper_frame} camera_optical_frame\n')
        self.get_logger().info(f'Saved extrinsics to {out}')
        return 0

    def _reprojection_self_check(self, R_x, t_x):
        """Rough AX=XB residual: how well a constant cam->gripper explains the data.

        Compares relative gripper motions against relative camera motions; a
        large value flags bad poses / wrong board params.
        """
        n = len(self.R_target2cam)
        if n < 2:
            return float('nan')
        errs = []
        X = np.eye(4)
        X[:3, :3] = R_x
        X[:3, 3] = t_x.reshape(3)

        def hom(R, t):
            H = np.eye(4)
            H[:3, :3] = R
            H[:3, 3] = np.asarray(t).reshape(3)
            return H

        for i in range(n - 1):
            Bi = hom(self.R_gripper2base[i], self.t_gripper2base[i])
            Bj = hom(self.R_gripper2base[i + 1], self.t_gripper2base[i + 1])
            A_gripper = np.linalg.inv(Bj) @ Bi  # gripper motion j<-i
            Ci = hom(self.R_target2cam[i], self.t_target2cam[i])
            Cj = hom(self.R_target2cam[i + 1], self.t_target2cam[i + 1])
            A_cam = Cj @ np.linalg.inv(Ci)      # camera-observed motion
            lhs = A_gripper @ X
            rhs = X @ A_cam
            errs.append(np.linalg.norm(lhs[:3, 3] - rhs[:3, 3]))
        return float(np.mean(errs)) if errs else float('nan')


def load_poses_csv(path):
    """Parse the poses CSV.

    Expected header columns (order-independent, '#'-comment lines ignored):
        name, x, y, z, qx, qy, qz, qw
    'name' is optional.  Returns a list of (name, geometry_msgs/Pose).
    """
    from geometry_msgs.msg import Pose
    poses = []
    with open(path, newline='') as f:
        rows = [r for r in f if r.strip() and not r.lstrip().startswith('#')]
    reader = csv.DictReader(rows)
    required = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
    if reader.fieldnames is None:
        return poses
    field = {n.strip().lower(): n for n in reader.fieldnames}
    for col in required:
        if col not in field:
            raise ValueError(f'CSV missing required column "{col}". '
                             f'Found: {reader.fieldnames}')
    for i, row in enumerate(reader):
        pose = Pose()
        pose.position.x = float(row[field['x']])
        pose.position.y = float(row[field['y']])
        pose.position.z = float(row[field['z']])
        pose.orientation.x = float(row[field['qx']])
        pose.orientation.y = float(row[field['qy']])
        pose.orientation.z = float(row[field['qz']])
        pose.orientation.w = float(row[field['qw']])
        has_name = 'name' in field and row[field['name']]
        name = row[field['name']].strip() if has_name else f'p{i:03d}'
        poses.append((name, pose))
    return poses


def main(args=None):
    rclpy.init(args=args)
    node = WristCameraCalibrator()
    rc = 1
    try:
        rc = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == '__main__':
    main()
