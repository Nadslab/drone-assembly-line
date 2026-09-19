#!/usr/bin/env python3
"""
Assembly orchestrator — lerobot_1 station.

Picks a drone arm from bin A and places it on the main conveyor using
geometric IK + direct FollowJointTrajectory control (no MoveIt).

The arm is expected to spawn with yaw=pi/2 so its forward (+X arm) direction
points toward world +Y, putting bin A at arm-frame coords (+0.05, +0.15, z).

State machine on startup:
  HOME → OPEN_GRIPPER → APPROACH_A → GRASP_A → CLOSE_GRIPPER
       → WAIT_ATTACH → RETREAT_A → HOME → (hold / extend here)
"""

import os
import sys
import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from ros_gz_interfaces.msg import Contacts

# Import IK from sibling script
sys.path.insert(0, os.path.dirname(__file__))
from so101_ik import ik_multi_seed, tcp_position

# ── Joint name constants ──────────────────────────────────────────────────
_ARM  = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
_ALL  = _ARM + ['gripper']

# ── Gripper positions ─────────────────────────────────────────────────────
_GRIPPER_OPEN  = 1.5    # rad — triggers release in gripper_attach_node
_GRIPPER_CLOSE = 0.0    # rad — triggers attach when contact detected

# ── Arm-frame targets for lerobot_1 (spawn yaw = pi/2) ───────────────────
#
#  World coords of bin_A centre: (-1.25, -0.35)
#  lerobot_1 spawn:              (-1.10, -0.40)
#  World delta:                  (-0.15, +0.05)
#
#  With yaw=pi/2, arm +X = world +Y, arm +Y = world -X:
#    arm_x = world_dy = +0.05
#    arm_y = -world_dx = +0.15
#
_BIN_A = dict(x=0.05, y=0.15)
_PART_Z     = 0.013   # part surface above arm base_link (world z=0.913, base z=0.900)
_APPROACH_Z = 0.10    # approach height — clears 45 mm bin walls + margin
_GRASP_Z    = 0.018   # grasp height — fingers around part

_HOME_Q = [0.0, 0.0, 0.0, 0.0, 0.0]

# ── Motion timing ─────────────────────────────────────────────────────────
_T_APPROACH = 3.0   # seconds for approach / retreat moves
_T_GRASP    = 2.0   # seconds for grasp descent / ascent
_T_GRIPPER  = 1.5   # seconds for gripper open / close
_T_HOME     = 4.0   # seconds for home moves


class AssemblyOrchestrator(Node):
    def __init__(self):
        super().__init__('assembly_orchestrator')
        cb_srv = ReentrantCallbackGroup()
        cb_cli = MutuallyExclusiveCallbackGroup()

        ns = self.declare_parameter('namespace', 'lerobot_1').get_parameter_value().string_value
        self._ns = ns

        self._lock    = threading.Lock()
        self._joints  = {j: 0.0 for j in _ALL}
        self._contact = False

        self.create_subscription(JointState, f'/{ns}/joint_states',
                                 self._js_cb, 10, callback_group=cb_srv)
        self.create_subscription(Contacts, f'/{ns}/gripper/contact',
                                 self._contact_cb, 10, callback_group=cb_srv)

        self._jtc = ActionClient(
            self, FollowJointTrajectory,
            f'/{ns}/so101_arm_controller/follow_joint_trajectory',
            callback_group=cb_cli,
        )

        # Pre-compute IK solutions at startup so failures are caught early
        self._q_approach = None
        self._q_grasp    = None
        self._compute_ik()

        # Start sequence after controllers come up
        self._seq_timer = self.create_timer(2.0, self._start_sequence,
                                            callback_group=cb_srv)

    # ── IK pre-computation ────────────────────────────────────────────────

    def _compute_ik(self):
        approach_target = [_BIN_A['x'], _BIN_A['y'], _APPROACH_Z]
        grasp_target    = [_BIN_A['x'], _BIN_A['y'], _GRASP_Z]

        self.get_logger().info(f'Computing IK for approach {approach_target}...')
        try:
            self._q_approach = ik_multi_seed(approach_target, tol=5e-3)
            pos = tcp_position(self._q_approach)
            self.get_logger().info(
                f'  approach IK: q={[f"{v:+.3f}" for v in self._q_approach]}'
                f'  TCP={[f"{v:.3f}" for v in pos]}')
        except RuntimeError as e:
            self.get_logger().error(f'Approach IK failed: {e}')

        self.get_logger().info(f'Computing IK for grasp {grasp_target}...')
        try:
            # Seed with approach solution for a smooth trajectory
            seed = self._q_approach if self._q_approach else None
            self._q_grasp = ik_multi_seed(grasp_target, tol=5e-3)
            pos = tcp_position(self._q_grasp)
            self.get_logger().info(
                f'  grasp IK: q={[f"{v:+.3f}" for v in self._q_grasp]}'
                f'  TCP={[f"{v:.3f}" for v in pos]}')
        except RuntimeError as e:
            self.get_logger().error(f'Grasp IK failed: {e}')

    # ── Subscriptions ─────────────────────────────────────────────────────

    def _js_cb(self, msg: JointState):
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                if name in self._joints:
                    self._joints[name] = pos

    def _contact_cb(self, _msg: Contacts):
        with self._lock:
            self._contact = True

    # ── Startup ───────────────────────────────────────────────────────────

    def _start_sequence(self):
        self._seq_timer.cancel()
        if not self._jtc.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('JTC server not available — aborting')
            return
        if self._q_approach is None or self._q_grasp is None:
            self.get_logger().error('IK solutions missing — aborting')
            return

        self.get_logger().info('Starting pick sequence for bin A...')
        self._run_pick()

    # ── Pick sequence ─────────────────────────────────────────────────────

    def _run_pick(self):
        log = self.get_logger().info

        log('[1/8] Moving to home...')
        self._move_arm(_HOME_Q, _T_HOME)

        log('[2/8] Opening gripper...')
        self._gripper(_GRIPPER_OPEN)

        log('[3/8] Moving to approach above bin A...')
        if not self._move_arm(self._q_approach, _T_APPROACH):
            self.get_logger().error('Approach move failed')
            return

        log('[4/8] Descending to grasp pose...')
        if not self._move_arm(self._q_grasp, _T_GRASP):
            self.get_logger().error('Grasp move failed')
            return

        log('[5/8] Closing gripper...')
        with self._lock:
            self._contact = False
        self._gripper(_GRIPPER_CLOSE)

        log('[6/8] Waiting for contact/attach...')
        attached = self._wait_contact(timeout=4.0)
        if attached:
            log('[6/8] Part attached!')
        else:
            self.get_logger().warn('[6/8] No contact detected — part may be missing')

        log('[7/8] Retreating to approach height...')
        self._move_arm(self._q_approach, _T_GRASP)

        log('[8/8] Returning to home...')
        self._move_arm(_HOME_Q, _T_HOME)

        if attached:
            log('Pick sequence COMPLETE — part in gripper, ready for place.')
        else:
            log('Pick sequence done — but no part confirmed.')

    # ── Motion helpers ────────────────────────────────────────────────────

    def _move_arm(self, arm_q: list[float], duration: float) -> bool:
        """Send arm joints to arm_q, gripper holds current position."""
        with self._lock:
            gripper_pos = self._joints.get('gripper', 0.0)

        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = _ALL

        pt = JointTrajectoryPoint()
        pt.positions = list(arm_q) + [gripper_pos]
        pt.velocities = [0.0] * len(_ALL)
        sec = int(duration)
        ns  = int((duration - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=ns)
        traj.points = [pt]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        result = self._send_and_wait(self._jtc, goal, timeout=duration + 8.0)
        if result is None:
            self.get_logger().error('JTC move timed out')
            return False
        ok = result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        if not ok:
            self.get_logger().warn(f'JTC error code: {result.result.error_code}')
        return ok

    def _gripper(self, target: float, duration: float = _T_GRIPPER) -> bool:
        """Move gripper only; arm stays at current positions."""
        with self._lock:
            arm_q = [self._joints[j] for j in _ARM]

        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = _ALL

        pt = JointTrajectoryPoint()
        pt.positions = arm_q + [target]
        pt.velocities = [0.0] * len(_ALL)
        sec = int(duration)
        ns  = int((duration - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=ns)
        traj.points = [pt]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self._send_and_wait(self._jtc, goal, timeout=duration + 5.0)
        return True

    def _wait_contact(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._contact:
                    return True
            time.sleep(0.05)
        return False

    def _send_and_wait(self, client, goal, timeout: float):
        done   = threading.Event()
        holder = [None]

        def _result_cb(future):
            holder[0] = future.result()
            done.set()

        def _accepted_cb(future):
            handle = future.result()
            if not handle.accepted:
                self.get_logger().warn('Goal rejected')
                done.set()
                return
            handle.get_result_async().add_done_callback(_result_cb)

        client.send_goal_async(goal).add_done_callback(_accepted_cb)
        done.wait(timeout=timeout)
        return holder[0]


def main(args=None):
    rclpy.init(args=args)
    node = AssemblyOrchestrator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
