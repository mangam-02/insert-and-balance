"""Grasp + insertion pose generation (the geometry component).

Everything here is a HARDCODED fixed transform relative to the detected object pose, as a
first pass. Offsets are parameters and are printed when applied.

  * compute_grasp     : peg pose -> (approach pose above the peg, final grasp pose)
  * compute_insertion : hole pose -> (approach pose above the hole, insertion target pose)

TODO: replace the fixed top-down orientation with an adaptive grasp derived from the peg's
detected orientation / mesh geometry (e.g. principal axis), and add multiple grasp candidates.
"""
import copy

from geometry_msgs.msg import PoseStamped


def _stamp(node, ps):
    ps.header.stamp = node.get_clock().now().to_msg()
    return ps


def _shift_z(ps, dz):
    out = copy.deepcopy(ps)
    out.pose.position.z += dz
    return out


def _fmt(ps):
    p, o = ps.pose.position, ps.pose.orientation
    return (f'[{ps.header.frame_id}] xyz=({p.x:.4f}, {p.y:.4f}, {p.z:.4f}) '
            f'quat=({o.x:.4f}, {o.y:.4f}, {o.z:.4f}, {o.w:.4f})')


def compute_grasp(node, peg_ps, params):
    """Return (approach_ps, grasp_ps) for the peg, both in peg_ps's (base) frame."""
    log = node.get_logger()
    grasp = PoseStamped()
    grasp.header.frame_id = peg_ps.header.frame_id
    # Position: peg position + a fixed offset.
    ox, oy, oz = params.grasp_offset_xyz
    grasp.pose.position.x = peg_ps.pose.position.x + ox
    grasp.pose.position.y = peg_ps.pose.position.y + oy
    grasp.pose.position.z = peg_ps.pose.position.z + oz
    # Orientation: fixed top-down grasp (hardcoded).
    qx, qy, qz, qw = params.grasp_orientation_xyzw
    grasp.pose.orientation.x = qx
    grasp.pose.orientation.y = qy
    grasp.pose.orientation.z = qz
    grasp.pose.orientation.w = qw
    _stamp(node, grasp)
    approach = _stamp(node, _shift_z(grasp, params.grasp_approach_height))

    log.info('--- GRASP GENERATION ---')
    log.info(f'peg pose         : {_fmt(peg_ps)}')
    log.info(f'applied offset   : xyz=({ox:.4f}, {oy:.4f}, {oz:.4f}), '
             f'orientation(xyzw)=({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f})')
    log.info(f'approach height  : +{params.grasp_approach_height:.4f} m in base Z')
    log.info(f'grasp pose       : {_fmt(grasp)}')
    log.info(f'approach pose    : {_fmt(approach)}')
    return approach, grasp


def compute_insertion(node, hole_ps, params):
    """Return (approach_ps above the hole, insertion_ps) in hole_ps's (base) frame.

    Approach is offset in +Z above the hole; insertion is straight down from the approach by
    `insertion_depth` (the downward Cartesian move executed in the INSERT state)."""
    log = node.get_logger()
    approach = PoseStamped()
    approach.header.frame_id = hole_ps.header.frame_id
    approach.pose.position.x = hole_ps.pose.position.x
    approach.pose.position.y = hole_ps.pose.position.y
    approach.pose.position.z = hole_ps.pose.position.z + params.hole_approach_height
    qx, qy, qz, qw = params.grasp_orientation_xyzw   # keep the same top-down tool orientation
    approach.pose.orientation.x = qx
    approach.pose.orientation.y = qy
    approach.pose.orientation.z = qz
    approach.pose.orientation.w = qw
    _stamp(node, approach)
    insertion = _stamp(node, _shift_z(approach, -params.insertion_depth))

    log.info('--- INSERTION GENERATION ---')
    log.info(f'hole pose        : {_fmt(hole_ps)}')
    log.info(f'approach height  : +{params.hole_approach_height:.4f} m above hole in base Z')
    log.info(f'insertion depth  : -{params.insertion_depth:.4f} m (downward Cartesian)')
    log.info(f'approach pose    : {_fmt(approach)}')
    log.info(f'insertion pose   : {_fmt(insertion)}')
    return approach, insertion
