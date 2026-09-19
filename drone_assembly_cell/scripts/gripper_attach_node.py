#!/usr/bin/env python3
"""
Gripper attach node.

Monitors a single arm's gripper contact sensor and joint state.  When the
gripper closes on a screw (contact detected + joint angle < CLOSE_THRESH),
the screw is locked to the gripper tip by teleporting it every 50 ms via the
gz set_pose service.  When the gripper opens past OPEN_THRESH the screw is
released (teleporting stops).

Parameters
----------
namespace : str
    ROS / Gazebo model namespace, e.g. "lerobot_1"
world : str
    Gazebo world name (default "frame_assembly_cell")
base_x, base_y, base_z : float
    Arm spawn position in Gazebo world frame (arm base is fixed)
"""

import math
import threading

import gz.transport13 as gz_transport
import gz.msgs10.pose_pb2 as gz_pose_msg
import gz.msgs10.boolean_pb2 as gz_bool_msg

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ros_gz_interfaces.msg import Contacts

import tf2_ros
from geometry_msgs.msg import TransformStamped


# ------------------------------------------------------------------
# Gripper joint thresholds (radians)
# ------------------------------------------------------------------
_CLOSE_THRESH = 0.3   # below this → gripper is gripping
_OPEN_THRESH  = 0.8   # above this → gripper is open (release)

_TICK_HZ = 20         # teleport update rate while gripping


class GripperAttach(Node):
    def __init__(self):
        super().__init__('gripper_attach')

        ns         = self.declare_parameter('namespace', '').get_parameter_value().string_value
        world      = self.declare_parameter('world', 'frame_assembly_cell').get_parameter_value().string_value
        base_x     = self.declare_parameter('base_x', 0.0).get_parameter_value().double_value
        base_y     = self.declare_parameter('base_y', 0.0).get_parameter_value().double_value
        base_z     = self.declare_parameter('base_z', 0.0).get_parameter_value().double_value
        spawn_yaw  = self.declare_parameter('spawn_yaw', 0.0).get_parameter_value().double_value

        self._ns    = ns
        self._world = world
        self._gz_node = gz_transport.Node()
        self._set_pose_srv = f'/world/{world}/set_pose'

        # Fixed arm base in world frame
        self._base_x   = base_x
        self._base_y   = base_y
        self._base_z   = base_z
        self._cos_yaw  = math.cos(spawn_yaw)
        self._sin_yaw  = math.sin(spawn_yaw)

        attachable_raw = self.declare_parameter(
            'attachable_prefixes',
            ['screw_', 'drone_'],
        ).get_parameter_value().string_array_value
        self._attachable_prefixes = list(attachable_raw)

        # State (protected by _lock)
        self._lock         = threading.Lock()
        self._gripper_pos  = 1.5      # assume open at start
        self._contact_screw: str | None = None  # model name from latest contacts
        self._gripping_screw: str | None = None
        self._local_offset = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)  # x,y,z,qx,qy,qz,qw relative to tip

        # TF
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._tip_frame   = f'{ns}/gripper_frame_link'
        self._base_frame  = f'{ns}/base_link'

        # Subscriptions
        self.create_subscription(
            JointState,
            f'/{ns}/joint_states',
            self._joint_state_cb,
            10,
        )
        self.create_subscription(
            Contacts,
            f'/{ns}/gripper/contact',
            self._contact_cb,
            10,
        )

        # Attach tick timer
        self._timer = self.create_timer(1.0 / _TICK_HZ, self._tick)

        self.get_logger().info(
            f'GripperAttach ready — ns={ns} base=({base_x:.3f},{base_y:.3f},{base_z:.3f})'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _joint_state_cb(self, msg: JointState):
        try:
            idx = msg.name.index('gripper')
        except ValueError:
            return
        pos = msg.position[idx]
        with self._lock:
            self._gripper_pos = pos
            # Release if gripper has opened
            if self._gripping_screw is not None and pos > _OPEN_THRESH:
                self.get_logger().info(
                    f'Released {self._gripping_screw} (gripper opened to {pos:.3f} rad)')
                self._gripping_screw = None

    def _contact_cb(self, msg: Contacts):
        found = None
        for contact in msg.contacts:
            for coll_name in (contact.collision1.name, contact.collision2.name):
                model = coll_name.split('::')[0]
                if any(model.startswith(p) for p in self._attachable_prefixes):
                    found = model
                    break
            if found:
                break
        with self._lock:
            self._contact_screw = found

    # ------------------------------------------------------------------
    # Tick
    # ------------------------------------------------------------------

    def _tick(self):
        with self._lock:
            gripper_pos   = self._gripper_pos
            contact_screw = self._contact_screw
            gripping      = self._gripping_screw

        # Transition: IDLE → GRIPPING
        if gripping is None and contact_screw is not None and gripper_pos < _CLOSE_THRESH:
            tip_tf = self._get_tip_transform()
            if tip_tf is None:
                return
            tip_world = self._tip_world_pose(tip_tf)
            # Record offset as the screw's current world pose relative to tip
            # For now store (0,0,0) offset — screw will snap to tip position on first tick
            # (a more precise offset would require querying screw pose, but snap is acceptable)
            with self._lock:
                self._gripping_screw = contact_screw
                self._local_offset   = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
            self.get_logger().info(f'Attached {contact_screw}')
            return

        # GRIPPING: teleport screw to tip each tick
        if gripping is None:
            return

        tip_tf = self._get_tip_transform()
        if tip_tf is None:
            return
        tx, ty, tz, qx, qy, qz, qw = self._tip_world_pose(tip_tf)
        self._set_pose(gripping, tx, ty, tz, qx, qy, qz, qw)

    # ------------------------------------------------------------------
    # TF helpers
    # ------------------------------------------------------------------

    def _get_tip_transform(self) -> TransformStamped | None:
        try:
            return self._tf_buffer.lookup_transform(
                self._base_frame,
                self._tip_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except Exception:
            return None

    def _tip_world_pose(self, tf: TransformStamped):
        """Convert TF (relative to arm base frame) to world-frame pose."""
        t = tf.transform.translation
        r = tf.transform.rotation
        # Apply spawn yaw rotation: world = Rz(yaw) @ arm_frame_translation + base_pos
        wx = self._base_x + self._cos_yaw * t.x - self._sin_yaw * t.y
        wy = self._base_y + self._sin_yaw * t.x + self._cos_yaw * t.y
        wz = self._base_z + t.z
        return wx, wy, wz, r.x, r.y, r.z, r.w

    # ------------------------------------------------------------------
    # gz set_pose
    # ------------------------------------------------------------------

    def _set_pose(self, model: str, x, y, z, qx, qy, qz, qw):
        req = gz_pose_msg.Pose()
        req.name = model
        req.position.x = x
        req.position.y = y
        req.position.z = z
        req.orientation.x = qx
        req.orientation.y = qy
        req.orientation.z = qz
        req.orientation.w = qw
        rep, ok = self._gz_node.request(
            self._set_pose_srv, req, gz_bool_msg.Boolean, 200)
        if not ok:
            self.get_logger().warn(f'set_pose failed for {model}')


def main(args=None):
    rclpy.init(args=args)
    node = GripperAttach()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
