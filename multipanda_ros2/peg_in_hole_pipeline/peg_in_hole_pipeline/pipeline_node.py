"""Peg grasp + insertion pipeline node.

Wires the four components (perception / grasp generation / motion / gripper) into the
state machine in states.py and runs it once. Everything tunable is a ROS2 parameter.

Prereqs (all in the multipanda container / on ROS_DOMAIN_ID=1):
  * MoveIt:   ros2 launch franka_moveit_config moveit.launch.py robot_ip:=... load_gripper:=true
  * Camera:   ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true ...
  * FoundationPose (other container) publishing /foundationpose/<obj>/get_pose
Run:
  ros2 run peg_in_hole_pipeline pipeline
  ros2 run peg_in_hole_pipeline pipeline --ros-args -p grasp_force:=30.0 -p grasp_width:=0.02
"""
from types import SimpleNamespace

import rclpy
from rclpy.node import Node

from .gripper import GripperClient
from .motion import MotionClient
from .perception import FoundationPoseClient
from .states import Context, StateMachine


class PipelineNode(Node):
    def __init__(self):
        super().__init__('peg_in_hole_pipeline')
        p = self._declare_params()

        self.perception = FoundationPoseClient(self, base_frame=p.base_frame,
                                               topic_ns=p.foundationpose_ns)
        self.motion = MotionClient(
            self, group_name=p.planning_group, base_frame=p.base_frame, ee_link=p.ee_link,
            vel_scale=p.vel_scale, acc_scale=p.acc_scale, planning_time=p.planning_time)
        self.gripper = GripperClient(self, ns=p.gripper_ns)
        self.params = p

    def _declare_params(self):
        d = self.declare_parameter
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        # frames / groups
        d('base_frame', 'panda_link0')
        d('planning_group', 'panda_manipulator')
        d('ee_link', 'panda_hand_tcp')
        d('foundationpose_ns', '/foundationpose')
        d('gripper_ns', '/panda_gripper')
        d('peg_object', 'peg')
        d('hole_object', 'insert')
        # home / motion
        d('joint_names', ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
                          'panda_joint5', 'panda_joint6', 'panda_joint7'])
        d('home_joints', [0.561829, 0.1989799, -0.0193699, -2.865767, 0.82115, 3.720473, 0.74422])  # Franka "ready"
        d('vel_scale', 0.1)
        d('acc_scale', 0.1)
        d('planning_time', 10.0)
        # grasp generation (hardcoded fixed transform, all tunable)
        d('grasp_orientation_xyzw', [1.0, 0.0, 0.0, 0.0])   # top-down: tool Z points down
        d('grasp_offset_xyz', [0.0, 0.0, 0.0])              # m, added to peg position
        d('grasp_approach_height', 0.10)                    # m above grasp, in base Z
        # gripper grasp
        d('grasp_width', 0.02)        # m, expected peg width between fingers
        d('grasp_force', 20.0)        # N  (franka_msgs/Grasp supports force)
        d('grasp_speed', 0.1)         # m/s
        d('grasp_epsilon_inner', 0.005)
        d('grasp_epsilon_outer', 0.005)
        d('gripper_open_width', 0.08)
        d('open_gripper_at_end', True)
        # insertion
        d('hole_approach_height', 0.10)   # m above hole
        d('insertion_depth', 0.08)        # m straight down
        d('cartesian_step', 0.005)
        d('cartesian_min_fraction', 0.9)
        names = ['base_frame', 'planning_group', 'ee_link', 'foundationpose_ns', 'gripper_ns',
                 'peg_object', 'hole_object', 'joint_names', 'home_joints', 'vel_scale',
                 'acc_scale', 'planning_time', 'grasp_orientation_xyzw', 'grasp_offset_xyz',
                 'grasp_approach_height', 'grasp_width', 'grasp_force', 'grasp_speed',
                 'grasp_epsilon_inner', 'grasp_epsilon_outer', 'gripper_open_width',
                 'open_gripper_at_end', 'hole_approach_height', 'insertion_depth',
                 'cartesian_step', 'cartesian_min_fraction']
        return SimpleNamespace(**{n: g(n) for n in names})

    def wait_for_infra(self):
        log = self.get_logger()
        log.info('Waiting for MoveIt /move_group + gripper action servers...')
        if not self.motion.wait_for_servers(timeout_sec=20.0):
            log.error('MoveIt action/service servers not available — is moveit.launch.py running?')
            return False
        if not self.gripper.wait_for_servers(timeout_sec=10.0):
            log.error('gripper action servers not available — is the franka_gripper running '
                      '(load_gripper:=true)?')
            return False
        return True

    def run(self):
        if not self.wait_for_infra():
            return False
        ctx = Context(self, self.params, self.perception, self.motion, self.gripper)
        return StateMachine(ctx).run()


def main(args=None):
    rclpy.init(args=args)
    node = PipelineNode()
    try:
        success = node.run()
        node.get_logger().info(f'PIPELINE {"COMPLETED" if success else "ABORTED"}.')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
