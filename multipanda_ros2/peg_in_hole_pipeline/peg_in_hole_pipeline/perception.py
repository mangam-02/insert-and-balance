"""Perception: read FoundationPose object poses and express them in the robot base.

FoundationPose runs in another container and publishes one geometry_msgs/PoseStamped
topic per object (built in pose_ros2.py): <ns>/<obj>/pose, with header.frame_id set to
the camera frame (camera_optical_frame). We subscribe to that topic, grab a fresh pose,
then transform it into the planning base frame (panda_link0) via TF -- the wrist-camera
hand-eye calibration connects the two frames.

(Previously this used a std_srvs/Trigger get_pose service; the FoundationPose stack went
back to plain topic publishing, so we subscribe instead of calling a service.)

TODO: add visual-servoing / multi-sample averaging here for a more robust detection.
"""
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  registers PoseStamped transform support


class FoundationPoseClient:
    def __init__(self, node, base_frame='panda_link0', topic_ns='/foundationpose'):
        self.node = node
        self.base_frame = base_frame
        self.topic_ns = topic_ns.rstrip('/')
        self._subs = {}     # obj -> Subscription
        self._latest = {}   # obj -> latest PoseStamped
        # Match the publisher's QoS (default rclpy publisher: RELIABLE, KEEP_LAST depth 10).
        self._qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                               history=HistoryPolicy.KEEP_LAST, depth=10)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)

    def _ensure_sub(self, obj):
        if obj not in self._subs:
            topic = f'{self.topic_ns}/{obj}/pose'
            self._subs[obj] = self.node.create_subscription(
                PoseStamped, topic, lambda msg, o=obj: self._latest.__setitem__(o, msg),
                self._qos)
        return self._subs[obj]

    def get_pose_in_camera(self, obj, timeout_sec=5.0):
        """Wait for a fresh pose on <ns>/<obj>/pose and return (PoseStamped, raw dict) or
        (None, None). The raw dict mirrors the old service payload for compatibility."""
        import rclpy
        self._ensure_sub(obj)
        # Drop any stale message so we block until a genuinely fresh frame arrives.
        self._latest.pop(obj, None)
        deadline = self.node.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while rclpy.ok() and obj not in self._latest:
            if self.node.get_clock().now().nanoseconds > deadline:
                self.node.get_logger().error(
                    f'no pose on {self.topic_ns}/{obj}/pose within {timeout_sec:.1f}s '
                    f'(is FoundationPose publishing for "{obj}"?)')
                return None, None
            rclpy.spin_once(self.node, timeout_sec=0.1)
        ps = self._latest[obj]
        p, q = ps.pose.position, ps.pose.orientation
        raw = {'object': obj, 'frame': ps.header.frame_id,
               'position': [p.x, p.y, p.z],
               'quaternion_xyzw': [q.x, q.y, q.z, q.w]}
        return ps, raw

    def get_pose_in_base(self, obj, timeout_sec=5.0):
        """Detect `obj` and return its PoseStamped in the base frame, or None on failure."""
        import rclpy
        ps_cam, raw = self.get_pose_in_camera(obj, timeout_sec=timeout_sec)
        if ps_cam is None:
            return None
        # Wait until TF can connect the camera frame to the base frame.
        deadline = self.node.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while rclpy.ok():
            try:
                return self._tf_buffer.transform(
                    ps_cam, self.base_frame, timeout=rclpy.duration.Duration(seconds=0.2))
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                    tf2_ros.ConnectivityException) as exc:
                if self.node.get_clock().now().nanoseconds > deadline:
                    self.node.get_logger().error(
                        f'{obj}: TF {ps_cam.header.frame_id} -> {self.base_frame} failed: {exc}')
                    return None
                rclpy.spin_once(self.node, timeout_sec=0.1)
