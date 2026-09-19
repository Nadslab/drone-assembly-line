#!/usr/bin/env python3
"""
Pick-and-place action server — one instance per arm namespace.

Motion strategy
---------------
Arm moves use MoveIt's MoveGroup action (plan_only=True) to get OMPL-planned,
collision-aware, velocity-profiled trajectories, then execute them via the
FollowJointTrajectory action client.  Gripper open/close uses the JTC directly
(all 6 joints, arm stays in place).

Provides  /{ns}/pick_part   (PickPart action)
          /{ns}/place_part  (PlacePart action)
"""

import os
import threading
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from ros_gz_interfaces.msg import Contacts

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, JointConstraint,
    MoveItErrorCodes, PlanningOptions, RobotState,
)

from drone_assembly_cell.action import PickPart, PlacePart


_ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
_ALL_JOINTS  = _ARM_JOINTS + ['gripper']

# XY offsets (arm-frame) for each approach pose — used to pick the nearest bin
_APPROACH_XY: dict[str, tuple[float, float]] = {
    'bin_A_approach': (-0.15, +0.10),
    'bin_B_approach': (-0.15, -0.05),
    'bin_C_approach': (+0.20, +0.10),
    'bin_D_approach': (+0.20, -0.05),
}


def _load_srdf(ns: str) -> dict[str, list[float]]:
    """Return {state_name: [j0..j4]} for group 'arm' from the SRDF."""
    path = os.path.join(
        get_package_share_directory('assembly_line_moveit_config'),
        'config', f'{ns}.srdf')
    poses: dict[str, list[float]] = {}
    for state in ET.parse(path).getroot().findall('group_state'):
        if state.get('group') == 'arm':
            j = {e.get('name'): float(e.get('value')) for e in state.findall('joint')}
            poses[state.get('name')] = [j.get(n, 0.0) for n in _ARM_JOINTS]
    return poses


class PickPlaceServer(Node):
    def __init__(self):
        super().__init__('pick_place_server')
        server_cb = ReentrantCallbackGroup()
        client_cb = MutuallyExclusiveCallbackGroup()

        ns = self.declare_parameter('namespace', '').get_parameter_value().string_value
        bx = self.declare_parameter('base_x', 0.0).get_parameter_value().double_value
        by = self.declare_parameter('base_y', 0.0).get_parameter_value().double_value
        bz = self.declare_parameter('base_z', 0.0).get_parameter_value().double_value

        self._ns   = ns
        self._bx, self._by, self._bz = bx, by, bz
        self._lock    = threading.Lock()
        self._joints  = {j: 0.0 for j in _ALL_JOINTS}
        self._contact = False
        self._named   = _load_srdf(ns)

        self.create_subscription(JointState, f'/{ns}/joint_states',
                                 self._js_cb, 10, callback_group=server_cb)
        self.create_subscription(Contacts, f'/{ns}/gripper/contact',
                                 self._contact_cb, 10, callback_group=server_cb)

        # MoveGroup action — plans arm trajectories (plan_only=True)
        self._mg = ActionClient(
            self, MoveGroup,
            f'/{ns}/move_group',
            callback_group=client_cb,
        )

        # JTC action — executes planned trajectories + direct gripper commands
        self._jtc = ActionClient(
            self, FollowJointTrajectory,
            f'/{ns}/so101_arm_controller/follow_joint_trajectory',
            callback_group=client_cb,
        )

        ActionServer(self, PickPart,  f'/{ns}/pick_part',
                     execute_callback=self._pick,  callback_group=server_cb)
        ActionServer(self, PlacePart, f'/{ns}/place_part',
                     execute_callback=self._place, callback_group=server_cb)

        # Move to home once servers are ready (fires once)
        self._startup_timer = self.create_timer(3.0, self._startup_home,
                                                callback_group=server_cb)

        self.get_logger().info(
            f'PickPlaceServer ready — /{ns}/pick_part  /{ns}/place_part\n'
            f'  Named poses: {list(self._named.keys())}')

    # ── subscriptions ──────────────────────────────────────────────────────

    def _js_cb(self, msg: JointState):
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                if name in self._joints:
                    self._joints[name] = pos

    def _contact_cb(self, _msg: Contacts):
        with self._lock:
            self._contact = True

    # ── startup ────────────────────────────────────────────────────────────

    def _startup_home(self):
        """Run once at startup: wait for servers then move to home."""
        self._startup_timer.cancel()

        if not self._jtc.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('JTC server not ready — skipping home')
            return
        if not self._mg.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('MoveGroup server not ready — skipping home')
            return

        self.get_logger().info('Homing arm to safe starting position...')
        ok = self._move_to_named('home', velocity_scale=0.1)
        if ok:
            self.get_logger().info('Home complete')
        else:
            self.get_logger().warn('Home move failed — check named state and MoveIt config')

    # ── MoveIt planning ────────────────────────────────────────────────────

    def _move_to_named(self, pose_name: str, velocity_scale: float = 0.3) -> bool:
        """Plan with MoveIt to a named SRDF state, then execute via JTC."""
        joints = self._named.get(pose_name)
        if joints is None:
            self.get_logger().error(f'Named state "{pose_name}" not in SRDF')
            return False

        # Build joint constraints
        constraints = Constraints()
        for joint_name, value in zip(_ARM_JOINTS, joints):
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        req = MotionPlanRequest()
        req.group_name = 'arm'
        req.start_state = RobotState(is_diff=True)
        req.goal_constraints = [constraints]
        req.num_planning_attempts = 3
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = velocity_scale
        req.max_acceleration_scaling_factor = velocity_scale * 0.67
        req.pipeline_id = 'ompl'

        opts = PlanningOptions()
        opts.plan_only = True

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = opts

        # Send to MoveGroup and wait
        plan_result = self._send_and_wait(self._mg, goal, timeout=20.0)
        if plan_result is None:
            self.get_logger().error(f'Planning timed out for "{pose_name}"')
            return False

        result = plan_result.result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f'Planning failed for "{pose_name}": error_code={result.error_code.val}')
            return False

        traj = result.planned_trajectory.joint_trajectory
        self.get_logger().info(
            f'Plan OK for "{pose_name}": {len(traj.points)} waypoints, '
            f'{traj.points[-1].time_from_start.sec:.1f}s')

        return self._execute_trajectory(traj)

    def _execute_trajectory(self, traj: JointTrajectory) -> bool:
        """Execute a JointTrajectory via the FollowJointTrajectory action."""
        traj.header.stamp = self.get_clock().now().to_msg()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        result = self._send_and_wait(self._jtc, goal,
                                     timeout=traj.points[-1].time_from_start.sec + 10.0)
        if result is None:
            self.get_logger().error('JTC execution timed out')
            return False

        ok = result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        if not ok:
            self.get_logger().warn(f'JTC execution error: {result.result.error_code}')
        return ok

    # ── gripper command ────────────────────────────────────────────────────

    def _gripper_cmd(self, target_pos: float, duration: float = 1.0):
        """Move only the gripper; arm joints hold current position."""
        with self._lock:
            arm_positions = [self._joints[j] for j in _ARM_JOINTS]
            # gripper position is ignored; we set the new target below

        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = list(_ALL_JOINTS)

        pt = JointTrajectoryPoint()
        pt.positions = arm_positions + [target_pos]
        pt.velocities = [0.0] * len(_ALL_JOINTS)
        pt.time_from_start = Duration(
            sec=int(duration), nanosec=int((duration % 1) * 1e9))
        traj.points = [pt]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        result = self._send_and_wait(self._jtc, goal, timeout=duration + 5.0)
        return result is not None

    # ── action client helper ───────────────────────────────────────────────

    def _send_and_wait(self, client, goal, timeout: float):
        """Send a goal to an action client and block until result or timeout."""
        done = threading.Event()
        holder = [None]

        def _result_cb(future):
            holder[0] = future.result()
            done.set()

        def _accepted_cb(future):
            handle = future.result()
            if not handle.accepted:
                self.get_logger().warn(f'Goal rejected by {client._action_name}')
                done.set()
                return
            handle.get_result_async().add_done_callback(_result_cb)

        client.send_goal_async(goal).add_done_callback(_accepted_cb)
        done.wait(timeout=timeout)
        return holder[0]

    # ── bin selection ──────────────────────────────────────────────────────

    def _approach_name(self, world_pose) -> str | None:
        """Return the SRDF approach pose name nearest to the world XY position."""
        x = world_pose.pose.position.x - self._bx
        y = world_pose.pose.position.y - self._by
        best, best_d = None, float('inf')
        for name, (ax, ay) in _APPROACH_XY.items():
            if name not in self._named:
                continue
            d = ((x - ax) ** 2 + (y - ay) ** 2) ** 0.5
            if d < best_d:
                best_d, best = d, name
        if best:
            self.get_logger().info(f'Approach: {best}  dist={best_d:.3f} m')
        return best

    def _wait_contact(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._contact:
                    return True
            time.sleep(0.05)
        return False

    # ── pick ───────────────────────────────────────────────────────────────

    def _pick(self, goal_handle):
        req = goal_handle.request

        def fb(phase: str):
            f = PickPart.Feedback()
            f.phase = phase
            goal_handle.publish_feedback(f)
            self.get_logger().info(f'[pick/{self._ns}] {phase}')

        approach_name = self._approach_name(req.source_pose)
        if approach_name is None:
            goal_handle.abort()
            return PickPart.Result(success=False, message='no matching approach pose in SRDF')

        grasp_name = approach_name.replace('_approach', '_grasp')
        if grasp_name not in self._named:
            goal_handle.abort()
            return PickPart.Result(
                success=False,
                message=f'{grasp_name} not in SRDF — add a grasp state for this bin')

        fb('approaching')
        if not self._move_to_named(approach_name):
            goal_handle.abort()
            return PickPart.Result(success=False, message='approach move failed')

        fb('descending')
        if not self._move_to_named(grasp_name):
            goal_handle.abort()
            return PickPart.Result(success=False, message='grasp move failed')

        fb('closing_gripper')
        self._gripper_cmd(req.gripper_close_pos)

        fb('checking_contact')
        with self._lock:
            self._contact = False
        got = self._wait_contact(4.0)

        fb('retreating')
        self._move_to_named(approach_name)

        goal_handle.succeed()
        return PickPart.Result(
            success=got,
            message='contact confirmed' if got else 'no contact — check part position')

    # ── place ──────────────────────────────────────────────────────────────

    def _place(self, goal_handle):
        req = goal_handle.request
        gripper_open = req.gripper_open_pos if req.gripper_open_pos > 0 else 1.5

        def fb(phase: str):
            f = PlacePart.Feedback()
            f.phase = phase
            goal_handle.publish_feedback(f)
            self.get_logger().info(f'[place/{self._ns}] {phase}')

        if 'conveyor_place' not in self._named or 'conveyor_place_grasp' not in self._named:
            goal_handle.abort()
            return PlacePart.Result(
                success=False, message='conveyor_place / conveyor_place_grasp not in SRDF')

        fb('approaching')
        if not self._move_to_named('conveyor_place'):
            goal_handle.abort()
            return PlacePart.Result(success=False, message='conveyor approach failed')

        fb('descending')
        if not self._move_to_named('conveyor_place_grasp'):
            goal_handle.abort()
            return PlacePart.Result(success=False, message='conveyor descend failed')

        fb('opening_gripper')
        self._gripper_cmd(gripper_open)
        time.sleep(0.5)

        fb('retreating')
        self._move_to_named('conveyor_place')

        goal_handle.succeed()
        return PlacePart.Result(success=True, message='placed')


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceServer()
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
