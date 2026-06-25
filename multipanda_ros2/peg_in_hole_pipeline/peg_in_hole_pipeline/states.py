"""State-machine for the peg grasp + insertion pipeline.

Each state is a handler that takes the shared Context and returns True (SUCCESS) or False
(FAILURE). Any failure transitions to ERROR. The machine is a flat ordered list so the flow
is easy to read, extend (insert a state in ORDER), and debug (every transition is logged).

TODO: swap this flat state list for a real behavior tree (e.g. py_trees) once the flow grows
branches/retries (re-detect on grasp failure, retry insertion with search, etc.).
"""
from . import grasp

# --- states ---------------------------------------------------------------------
HOME = 'HOME'
DETECT_OBJECTS = 'DETECT_OBJECTS'
COMPUTE_GRASP = 'COMPUTE_GRASP'
MOVE_TO_GRASP = 'MOVE_TO_GRASP'
CLOSE_GRIPPER = 'CLOSE_GRIPPER'
MOVE_TO_HOLE = 'MOVE_TO_HOLE'
INSERT = 'INSERT'
FINISHED = 'FINISHED'
ERROR = 'ERROR'

# Linear happy-path order; ERROR is reached from any failing state.
ORDER = [HOME, DETECT_OBJECTS, COMPUTE_GRASP, MOVE_TO_GRASP,
         CLOSE_GRIPPER, MOVE_TO_HOLE, INSERT, FINISHED]


class Context:
    """Shared blackboard + component handles passed to every state."""
    def __init__(self, node, params, perception, motion, gripper):
        self.node = node
        self.params = params
        self.perception = perception
        self.motion = motion
        self.gripper = gripper
        # blackboard (filled in as the pipeline runs)
        self.peg_pose = None
        self.hole_pose = None
        self.grasp_pose = None
        self.grasp_approach = None
        self.hole_approach = None
        self.insertion_pose = None
        self.failed_state = None
        self.error_msg = ''


# --- state handlers -------------------------------------------------------------
def state_home(ctx):
    log = ctx.node.get_logger()
    log.info('Initialization: moving to HOME and opening the gripper.')
    ok_grip, msg = ctx.gripper.open(width=ctx.params.gripper_open_width)
    if not ok_grip:
        log.warn(f'gripper open during init failed: {msg} (continuing)')
    ok, code = ctx.motion.move_to_joints(ctx.params.joint_names, ctx.params.home_joints)
    log.info(f'HOME motion: plan+execute {"OK" if ok else "FAILED"} (MoveItErrorCode={code})')
    if not ok:
        ctx.error_msg = f'failed to reach HOME (error code {code})'
    return ok


def state_detect(ctx):
    log = ctx.node.get_logger()
    log.info('Detecting peg + hole via FoundationPose pose topics...')
    peg = ctx.perception.get_pose_in_base(ctx.params.peg_object)
    hole = ctx.perception.get_pose_in_base(ctx.params.hole_object)
    if peg is None or hole is None:
        ctx.error_msg = (f'object detection incomplete: peg={"ok" if peg else "MISSING"}, '
                         f'hole={"ok" if hole else "MISSING"}')
        return False
    ctx.peg_pose, ctx.hole_pose = peg, hole
    p, h = peg.pose.position, hole.pose.position
    log.info(f'PEG  pose  [{peg.header.frame_id}]: xyz=({p.x:.4f}, {p.y:.4f}, {p.z:.4f})')
    log.info(f'HOLE pose  [{hole.header.frame_id}]: xyz=({h.x:.4f}, {h.y:.4f}, {h.z:.4f})')
    return True


def state_compute_grasp(ctx):
    ctx.grasp_approach, ctx.grasp_pose = grasp.compute_grasp(ctx.node, ctx.peg_pose, ctx.params)
    return True


def state_move_to_grasp(ctx):
    log = ctx.node.get_logger()
    log.info('Moving to grasp APPROACH pose...')
    ok, code = ctx.motion.move_to_pose(ctx.grasp_approach)
    log.info(f'approach motion: {"OK" if ok else "FAILED"} (error code {code})')
    if not ok:
        ctx.error_msg = f'approach motion failed (error code {code})'
        return False
    log.info('Moving to final GRASP pose...')
    ok, code = ctx.motion.move_to_pose(ctx.grasp_pose)
    log.info(f'grasp motion: {"OK" if ok else "FAILED"} (error code {code})')
    if not ok:
        ctx.error_msg = f'grasp motion failed (error code {code})'
    return ok


def state_close_gripper(ctx):
    log = ctx.node.get_logger()
    log.info(f'Closing gripper on the peg (force={ctx.params.grasp_force} N).')
    ok, msg = ctx.gripper.grasp(
        width=ctx.params.grasp_width, force=ctx.params.grasp_force,
        speed=ctx.params.grasp_speed,
        epsilon_inner=ctx.params.grasp_epsilon_inner,
        epsilon_outer=ctx.params.grasp_epsilon_outer)
    log.info(f'gripper grasp result: success={ok} msg="{msg}"')
    if not ok:
        ctx.error_msg = f'grasp failed: {msg}'
    return ok


def state_move_to_hole(ctx):
    log = ctx.node.get_logger()
    ctx.hole_approach, ctx.insertion_pose = grasp.compute_insertion(
        ctx.node, ctx.hole_pose, ctx.params)
    log.info('Moving to insertion APPROACH pose above the hole...')
    ok, code = ctx.motion.move_to_pose(ctx.hole_approach)
    log.info(f'hole-approach motion: {"OK" if ok else "FAILED"} (error code {code})')
    if not ok:
        ctx.error_msg = f'move to hole approach failed (error code {code})'
    return ok


def state_insert(ctx):
    log = ctx.node.get_logger()
    log.info('Executing downward Cartesian insertion...')
    ok, fraction = ctx.motion.move_cartesian(
        [ctx.insertion_pose.pose], eef_step=ctx.params.cartesian_step,
        min_fraction=ctx.params.cartesian_min_fraction)
    log.info(f'insertion Cartesian path: {"OK" if ok else "FAILED"} (fraction={fraction:.2f})')
    if not ok:
        ctx.error_msg = f'insertion failed: only {fraction:.2f} of the path was planned'
    return ok


def state_finished(ctx):
    log = ctx.node.get_logger()
    log.info('==================  PEG-IN-HOLE SUCCESS  ==================')
    if ctx.params.open_gripper_at_end:
        ctx.gripper.open(width=ctx.params.gripper_open_width)
    log.info('Returning to HOME (safe pose).')
    ok, code = ctx.motion.move_to_joints(ctx.params.joint_names, ctx.params.home_joints)
    log.info(f'return-home motion: {"OK" if ok else "FAILED"} (error code {code})')
    return True


def state_error(ctx):
    log = ctx.node.get_logger()
    log.error('==================  PIPELINE ERROR  ==================')
    log.error(f'failed state : {ctx.failed_state}')
    log.error(f'diagnostic   : {ctx.error_msg or "(none)"}')
    log.error(f'peg detected : {ctx.peg_pose is not None}, hole detected: {ctx.hole_pose is not None}')
    log.error('Aborting. Robot left in place for inspection.')
    return False


HANDLERS = {
    HOME: state_home,
    DETECT_OBJECTS: state_detect,
    COMPUTE_GRASP: state_compute_grasp,
    MOVE_TO_GRASP: state_move_to_grasp,
    CLOSE_GRIPPER: state_close_gripper,
    MOVE_TO_HOLE: state_move_to_hole,
    INSERT: state_insert,
    FINISHED: state_finished,
    ERROR: state_error,
}


class StateMachine:
    """Runs ORDER top-to-bottom; any FAILURE jumps to ERROR. Logs every transition."""
    def __init__(self, ctx, handlers=HANDLERS, order=ORDER):
        self.ctx = ctx
        self.handlers = handlers
        self.order = order

    def run(self):
        log = self.ctx.node.get_logger()
        for state in self.order:
            log.info(f'==>  ENTER STATE: {state}')
            ok = self.handlers[state](self.ctx)
            log.info(f'<==  STATE {state}: {"SUCCESS" if ok else "FAILURE"}')
            if not ok:
                self.ctx.failed_state = state
                log.info(f'==>  TRANSITION: {state} -> {ERROR}')
                self.handlers[ERROR](self.ctx)
                return False
        return True
