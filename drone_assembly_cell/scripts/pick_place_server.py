#!/usr/bin/env python3
"""
Generic pick-and-place action server (one instance per arm namespace).

Provides:
  /{namespace}/pick_part   (drone_assembly_cell/action/PickPart)
  /{namespace}/place_part  (drone_assembly_cell/action/PlacePart)

Motion is executed via:
  IK:     /{namespace}/compute_ik  (moveit_msgs/srv/GetPositionIK)
  Motion: /{namespace}/so101_arm_controller/follow_joint_trajectory
          (control_msgs/action/FollowJointTrajectory)

The gripper_attach_node (running in parallel) handles teleporting picked
objects automatically — this server only needs to control joints.

Parameters
----------
namespace : str
    Arm ROS/Gazebo namespace, e.g. "lerobot_1"
base_x, base_y, base_z : float
    Arm base position in world frame (no rotation at spawn)
default_approach_height : float
    Default Z offset above pick/place target (m, default 0.10)
contact_timeout : float
    Seconds to wait for gripper contact after closing (default 5.0)
ik_timeout : float
    Timeout for compute_ik service call in seconds (default 5.0)
"""

import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from ros_gz_interfaces.msg import Contacts

from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from control_msgs.action import FollowJointTrajectory

from drone_assembly_cell.action import PickPart, PlacePart


# Joint names in controller order (must match allow_partial_joints_goal=False)
_JOINT_NAMES = [
    'shoulder_pan', 'shoulder_lift', 'elbow_flex',
    'wrist_flex', 'wrist_roll', 'gripper',
]

_DEFAULT_MOVE_DURATION = 3.0   # seconds per motion segment
_DEFAULT_GRIPPER_DURATION = 1.0


class PickPlaceServer(Node):
    def __init__(self):
        super().__init__('pick_place_server')
        cb = ReentrantCallbackGroup()

        ns      = self.declare_parameter('namespace', '').get_parameter_value().string_value
        base_x  = self.declare_parameter('base_x', 0.0).get_parameter_value().double_value
        base_y  = self.declare_parameter('base_y', 0.0).get_parameter_value().double_value
        base_z  = self.declare_parameter('base_z', 0.0).get_parameter_value().double_value
        self._approach_default = self.declare_parameter(
            'default_approach_height', 0.10).get_parameter_value().double_value
        self._contact_timeout  = self.declare_parameter(
            'contact_timeout', 5.0).get_parameter_value().double_value
        self._ik_timeout       = self.declare_parameter(
            'ik_timeout', 5.0).get_parameter_value().double_value

        self._ns     = ns
        self._base_x = base_x
        self._base_y = base_y
        self._base_z = base_z

        # Current joint state (protected by lock)
        self._lock         = threading.Lock()
        self._joint_pos    = {j: 0.0 for j in _JOINT_NAMES}
        self._contact_seen = False

        # Subscriptions
        self.create_subscription(
            JointState, f'/{ns}/joint_states', self._joint_state_cb, 10,
            callback_group=cb)
        self.create_subscription(
            Contacts, f'/{ns}/gripper/contact', self._contact_cb, 10,
            callback_group=cb)

        # IK service client
        self._ik_client = self.create_client(
            GetPositionIK, f'/{ns}/compute_ik', callback_group=cb)

        # JTC action client
        self._jtc_client = rclpy.action.ActionClient(
            self, FollowJointTrajectory,
            f'/{ns}/so101_arm_controller/follow_joint_trajectory',
            callback_group=cb)

        # Action servers
        self._pick_server = ActionServer(
            self, PickPart, f'/{ns}/pick_part',
            execute_callback=self._execute_pick,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=cb)

        self._place_server = ActionServer(
            self, PlacePart, f'/{ns}/place_part',
            execute_callback=self._execute_place,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=cb)

        self.get_logger().info(
            f'PickPlaceServer ready — ns={ns} '
            f'base=({base_x:.2f},{base_y:.2f},{base_z:.2f})')

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _joint_state_cb(self, msg: JointState):
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                if name in self._joint_pos:
                    self._joint_pos[name] = pos

    def _contact_cb(self, _msg: Contacts):
        with self._lock:
            self._contact_seen = True

    # ------------------------------------------------------------------
    # Pick action
    # ------------------------------------------------------------------

    async def _execute_pick(self, goal_handle):
        goal = goal_handle.request
        approach_h = goal.approach_height if goal.approach_height > 0 else self._approach_default
        gripper_close = goal.gripper_close_pos  # 0.0 if not set

        def feedback(phase):
            fb = PickPart.Feedback()
            fb.phase = phase
            goal_handle.publish_feedback(fb)
            self.get_logger().info(f'[pick/{self._ns}] {phase}')

        # Convert source pose to arm frame and compute approach
        arm_pick   = self._to_arm_frame(goal.source_pose)
        arm_approach = _offset_z(arm_pick, approach_h)

        # 1 — Approaching
        feedback('approaching')
        approach_joints = await self._compute_ik(arm_approach)
        if approach_joints is None:
            goal_handle.abort()
            return PickPart.Result(success=False, message='IK failed for approach pose')
        ok = await self._move_arm(approach_joints)
        if not ok:
            goal_handle.abort()
            return PickPart.Result(success=False, message='Motion failed: approach')

        # 2 — Descending
        feedback('descending')
        pick_joints = await self._compute_ik(arm_pick)
        if pick_joints is None:
            goal_handle.abort()
            return PickPart.Result(success=False, message='IK failed for pick pose')
        ok = await self._move_arm(pick_joints)
        if not ok:
            goal_handle.abort()
            return PickPart.Result(success=False, message='Motion failed: descend')

        # 3 — Closing gripper
        feedback('closing_gripper')
        await self._move_gripper(gripper_close)

        # 4 — Checking contact
        feedback('checking_contact')
        with self._lock:
            self._contact_seen = False
        got_contact = await self._wait_for_contact(self._contact_timeout)

        # 5 — Retreating (always retreat even if no contact)
        feedback('retreating')
        await self._move_arm(approach_joints)

        msg = 'contact confirmed' if got_contact else 'no contact detected — check alignment'
        goal_handle.succeed()
        return PickPart.Result(success=got_contact, message=msg)

    # ------------------------------------------------------------------
    # Place action
    # ------------------------------------------------------------------

    async def _execute_place(self, goal_handle):
        goal = goal_handle.request
        approach_h  = goal.approach_height if goal.approach_height > 0 else self._approach_default
        gripper_open = goal.gripper_open_pos if goal.gripper_open_pos > 0 else 1.5

        def feedback(phase):
            fb = PlacePart.Feedback()
            fb.phase = phase
            goal_handle.publish_feedback(fb)
            self.get_logger().info(f'[place/{self._ns}] {phase}')

        arm_place    = self._to_arm_frame(goal.target_pose)
        arm_approach = _offset_z(arm_place, approach_h)

        # 1 — Approaching
        feedback('approaching')
        approach_joints = await self._compute_ik(arm_approach)
        if approach_joints is None:
            goal_handle.abort()
            return PlacePart.Result(success=False, message='IK failed for approach pose')
        ok = await self._move_arm(approach_joints)
        if not ok:
            goal_handle.abort()
            return PlacePart.Result(success=False, message='Motion failed: approach')

        # 2 — Descending
        feedback('descending')
        place_joints = await self._compute_ik(arm_place)
        if place_joints is None:
            goal_handle.abort()
            return PlacePart.Result(success=False, message='IK failed for place pose')
        ok = await self._move_arm(place_joints)
        if not ok:
            goal_handle.abort()
            return PlacePart.Result(success=False, message='Motion failed: descend')

        # 3 — Opening gripper (gripper_attach auto-releases)
        feedback('opening_gripper')
        await self._move_gripper(gripper_open)
        # Brief settle time
        await self._sleep(0.5)

        # 4 — Retreating
        feedback('retreating')
        await self._move_arm(approach_joints)

        goal_handle.succeed()
        return PlacePart.Result(success=True, message='placed')

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _to_arm_frame(self, world_pose: PoseStamped) -> PoseStamped:
        """Subtract arm base world position; orientation unchanged (no rotation at spawn)."""
        p = PoseStamped()
        p.header.frame_id = f'{self._ns}/base_link'
        p.pose.position.x = world_pose.pose.position.x - self._base_x
        p.pose.position.y = world_pose.pose.position.y - self._base_y
        p.pose.position.z = world_pose.pose.position.z - self._base_z
        p.pose.orientation = world_pose.pose.orientation
        # Default to identity orientation if zero
        if (p.pose.orientation.x == 0 and p.pose.orientation.y == 0
                and p.pose.orientation.z == 0 and p.pose.orientation.w == 0):
            p.pose.orientation.w = 1.0
        return p

    async def _compute_ik(self, pose_stamped: PoseStamped):
        """Call /{ns}/compute_ik; return joint positions list or None."""
        if not self._ik_client.wait_for_service(timeout_sec=self._ik_timeout):
            self.get_logger().error(f'/{self._ns}/compute_ik not available')
            return None

        req = GetPositionIK.Request()
        req.ik_request.group_name = 'arm'
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.timeout.sec = int(self._ik_timeout)
        req.ik_request.avoid_collisions = True
        # Seed from current joint state
        with self._lock:
            seed_positions = [self._joint_pos[j] for j in _JOINT_NAMES]
        seed_state = RobotState()
        seed_state.joint_state.name     = _JOINT_NAMES
        seed_state.joint_state.position = seed_positions
        req.ik_request.robot_state = seed_state

        future = self._ik_client.call_async(req)
        await self._spin_until_future(future, self._ik_timeout + 1.0)

        if future.result() is None:
            self.get_logger().error('IK service call failed')
            return None

        resp = future.result()
        if resp.error_code.val != 1:  # 1 = SUCCESS
            self.get_logger().warning(f'IK error code: {resp.error_code.val}')
            return None

        js = resp.solution.joint_state
        name_to_pos = dict(zip(js.name, js.position))
        try:
            return [name_to_pos[j] for j in _JOINT_NAMES]
        except KeyError as e:
            self.get_logger().error(f'IK response missing joint: {e}')
            return None

    async def _move_arm(self, joint_positions, duration=_DEFAULT_MOVE_DURATION):
        """Send JTC goal for all joints; gripper stays at current position."""
        with self._lock:
            gripper = self._joint_pos['gripper']
        positions = list(joint_positions[:5]) + [gripper]
        return await self._send_jtc(positions, duration)

    async def _move_gripper(self, gripper_pos, duration=_DEFAULT_GRIPPER_DURATION):
        """Send JTC goal holding arm joints at current state, moving only gripper."""
        with self._lock:
            positions = [self._joint_pos[j] for j in _JOINT_NAMES[:5]] + [gripper_pos]
        return await self._send_jtc(positions, duration)

    async def _send_jtc(self, positions, duration):
        if not self._jtc_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('JTC action server not available')
            return False

        traj = JointTrajectory()
        traj.joint_names = _JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = positions
        sec   = int(duration)
        nsec  = int((duration - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nsec)
        traj.points = [pt]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        future = self._jtc_client.send_goal_async(goal)
        await self._spin_until_future(future, 5.0)
        if future.result() is None:
            return False

        gh = future.result()
        if not gh.accepted:
            return False

        result_future = gh.get_result_async()
        await self._spin_until_future(result_future, duration + 5.0)
        return result_future.result() is not None

    async def _wait_for_contact(self, timeout) -> bool:
        deadline = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while self.get_clock().now().nanoseconds < deadline:
            with self._lock:
                if self._contact_seen:
                    return True
            await self._sleep(0.05)
        return False

    async def _spin_until_future(self, future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            await self._sleep(0.02)

    async def _sleep(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0)
            await _yield()


async def _yield():
    """Yield control back to the event loop once."""
    import asyncio
    await asyncio.sleep(0)


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
