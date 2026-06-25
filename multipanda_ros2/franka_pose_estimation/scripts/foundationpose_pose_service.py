#!/usr/bin/env python3
"""ROS 2 service node that serves the live FoundationPose object pose.

A background worker drives the same pipeline as FoundationPose's ``live_pose.py``:
start a RealSense D405 -> wait until the text-prompted object is detected ->
segment it -> ``FoundationPose.register()`` on that frame -> ``track_one()`` every
frame after. The 4x4 ``ob_in_cam`` from each track step (object pose in *camera
optical coordinates*, meters) is cached. A ``GetObjectPose`` service hands the
latest cached pose back as a ``geometry_msgs/PoseStamped``.

This is meant to run *inside the FoundationPose ``live`` container* (Ubuntu 22.04,
system python 3.10), with ROS 2 Humble apt-installed alongside the FP deps so a
single interpreter has both ``rclpy`` and ``estimater``. With ``--network host``
and a matching ``ROS_DOMAIN_ID`` it joins the franka DDS graph automatically.

The heavy FoundationPose / RealSense imports are deferred to the worker thread so
the node (and ``colcon build``) never need a GPU or the FP env to merely load.
"""
import os
import sys
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from franka_pose_estimation.srv import GetObjectPose


def _mat_to_quaternion(R):
    """3x3 rotation matrix -> (x, y, z, w) quaternion. Dependency-free."""
    t = np.trace(R)
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


class FoundationPoseService(Node):

    def __init__(self):
        super().__init__('foundationpose_pose_service')

        # --- parameters ---
        fp_dir = self.declare_parameter(
            'foundationpose_dir',
            os.environ.get('FOUNDATIONPOSE_DIR', '/workspace')).value
        self.fp_dir = os.path.abspath(os.path.expanduser(fp_dir))
        mesh_file = self.declare_parameter(
            'mesh_file', 'demo_data/nut/mesh/nut.obj').value
        self.mesh_file = mesh_file if os.path.isabs(mesh_file) \
            else os.path.join(self.fp_dir, mesh_file)
        self.prompt = self.declare_parameter('prompt', 'nut').value
        self.camera_frame = self.declare_parameter(
            'camera_frame', 'camera_color_optical_frame').value
        self.zfar = float(self.declare_parameter('zfar', 1.0).value)
        self.est_refine_iter = int(self.declare_parameter('est_refine_iter', 5).value)
        self.track_refine_iter = int(self.declare_parameter('track_refine_iter', 2).value)
        self.width = int(self.declare_parameter('width', 640).value)
        self.height = int(self.declare_parameter('height', 480).value)
        self.fps = int(self.declare_parameter('fps', 30).value)

        # --- shared state (worker writes, service reads) ---
        self._lock = threading.Lock()
        self._pose = None          # latest 4x4 ob_in_cam, or None before registration
        self._stamp = None         # rclpy Time of the latest pose
        self._status = 'starting'

        self.srv = self.create_service(
            GetObjectPose, '~/get_object_pose', self._on_request)

        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._run_pipeline, daemon=True)
        self._worker.start()
        self.get_logger().info(
            f"service ready on '{self.srv.srv_name}' "
            f"(prompt='{self.prompt}', mesh='{self.mesh_file}')")

    # --- service ---
    def _on_request(self, request, response):
        with self._lock:
            pose = None if self._pose is None else self._pose.copy()
            stamp = self._stamp
            response.message = self._status
        if pose is None:
            response.valid = False
            return response
        response.valid = True
        ps = PoseStamped()
        ps.header.stamp = (stamp or self.get_clock().now()).to_msg()
        ps.header.frame_id = self.camera_frame
        ps.pose.position.x = float(pose[0, 3])
        ps.pose.position.y = float(pose[1, 3])
        ps.pose.position.z = float(pose[2, 3])
        qx, qy, qz, qw = _mat_to_quaternion(pose[:3, :3])
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        response.pose = ps
        return response

    def _set_status(self, status):
        with self._lock:
            self._status = status
        self.get_logger().info(status)

    # --- worker: the FoundationPose live pipeline ---
    def _run_pipeline(self):
        try:
            self._pipeline_impl()
        except Exception as exc:  # surface the failure instead of dying silently
            self.get_logger().error(f"pose pipeline crashed: {exc!r}")
            self._set_status(f"error: {exc}")

    def _pipeline_impl(self):
        if self.fp_dir not in sys.path:
            sys.path.insert(0, self.fp_dir)
        # Heavy, env-specific imports happen here (not at module load).
        import trimesh
        import pyrealsense2 as rs
        import seg_prompt
        from estimater import (
            FoundationPose, ScorePredictor, PoseRefinePredictor,
            set_logging_format, set_seed, dr)

        set_logging_format()
        set_seed(0)

        # --- camera ---
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        cfg.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        profile = pipe.start(cfg)
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        align = rs.align(rs.stream.color)
        for _ in range(15):
            pipe.wait_for_frames()  # warm up / let auto-exposure settle

        def grab():
            f = align.process(pipe.wait_for_frames())
            color = np.asanyarray(f.get_color_frame().get_data())          # BGR
            depth_m = np.asanyarray(f.get_depth_frame().get_data()).astype(np.float32) * depth_scale
            depth_m[(depth_m < 0.001) | (depth_m > self.zfar)] = 0
            intr = f.get_color_frame().profile.as_video_stream_profile().intrinsics
            K = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]], float)
            return color, depth_m, K

        # --- estimator ---
        mesh = trimesh.load(self.mesh_file)
        est = FoundationPose(
            model_pts=mesh.vertices, model_normals=mesh.vertex_normals, mesh=mesh,
            scorer=ScorePredictor(), refiner=PoseRefinePredictor(),
            debug_dir='/tmp/foundationpose_service', debug=0,
            glctx=dr.RasterizeCudaContext())
        self._set_status(f"estimator ready; looking for '{self.prompt}'")

        # --- register: wait until the prompted object is detected ---
        pose = None
        while pose is None and not self._stop.is_set():
            color, depth_m, K = grab()
            rgb = np.ascontiguousarray(color[..., ::-1])  # BGR->RGB, contiguous (torch needs it)
            mask, _ = seg_prompt.segment(rgb, self.prompt)
            valid_depth = int(((depth_m > 0) & mask).sum())
            if mask.sum() < 200 or valid_depth < 100:
                continue
            self._set_status(
                f"detected '{self.prompt}' ({int(mask.sum())} px) -> register")
            pose = est.register(K=K, rgb=rgb, depth=depth_m, ob_mask=mask,
                                iteration=self.est_refine_iter)

        # --- track every frame, caching ob_in_cam ---
        self._set_status('tracking')
        while not self._stop.is_set():
            color, depth_m, K = grab()
            rgb = np.ascontiguousarray(color[..., ::-1])
            pose = est.track_one(rgb=rgb, depth=depth_m, K=K,
                                 iteration=self.track_refine_iter)
            with self._lock:
                self._pose = np.asarray(pose, dtype=float).reshape(4, 4)
                self._stamp = self.get_clock().now()
                self._status = 'tracking'

        pipe.stop()

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FoundationPoseService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
