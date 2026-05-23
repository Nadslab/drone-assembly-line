#!/usr/bin/env python3
"""
Screw feeder spawner node.

Provides /screw_feeder/spawn (std_srvs/Trigger).
Each call creates a new m3_screw entity in the running Gazebo world
at the feeder pickup point (ramp exit, just in front of the feeder column).

Pickup point (world frame):
  x = 0.800  (feeder centre x, station 4)
  y = -0.379 (≈ 29 mm in front of feeder column centre at y = -0.350)
  z = 0.967  (≈ 6 mm above ramp surface at world z ≈ 0.961)

Small uniform jitter (±4 mm x, ±2 mm y) is applied so consecutive screws
don't spawn on top of each other.
"""

import random
import subprocess
import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory


# Inline m3_screw SDF — kept here so no file-path resolution is needed at
# spawn time.  Name placeholder is replaced per-call.
_M3_SCREW_SDF = """\
<sdf version="1.9">
  <model name="{name}">
    <link name="link">
      <inertial>
        <mass>0.001</mass>
        <inertia>
          <ixx>2.19e-8</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>2.19e-8</iyy><iyz>0</iyz>
          <izz>1.13e-9</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <cylinder><radius>0.0015</radius><length>0.016</length></cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder><radius>0.0015</radius><length>0.016</length></cylinder>
        </geometry>
        <material>
          <ambient>0.4 0.4 0.4 1</ambient>
          <diffuse>0.65 0.65 0.65 1</diffuse>
          <specular>0.8 0.8 0.8 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

# Base pickup coordinates (world frame).  See module docstring.
# Screw feeder relocated to station 4 at (0.8, -0.35, 0.950).
# Pickup point = feeder pose + same relative offset as before.
_PICKUP_X = 0.800
_PICKUP_Y = -0.379
_PICKUP_Z = 0.967

# Jitter ranges
_JITTER_X = 0.004   # ± metres
_JITTER_Y = 0.002


class ScrewSpawner(Node):
    def __init__(self):
        super().__init__('screw_spawner')
        self._count = 0
        self._world = self.declare_parameter(
            'world', 'frame_assembly_cell').get_parameter_value().string_value
        self._gz_bin = self.declare_parameter(
            'gz_bin',
            '/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz'
        ).get_parameter_value().string_value

        self._srv = self.create_service(
            Trigger, '/screw_feeder/spawn', self._spawn_cb)
        self.get_logger().info(
            f'Screw spawner ready — service /screw_feeder/spawn '
            f'(world: {self._world})')

    def _spawn_cb(self, _req, response):
        self._count += 1
        name = f'screw_{self._count:03d}'

        x = _PICKUP_X + random.uniform(-_JITTER_X, _JITTER_X)
        y = _PICKUP_Y + random.uniform(-_JITTER_Y, _JITTER_Y)
        z = _PICKUP_Z

        sdf_str = _M3_SCREW_SDF.format(name=name)

        # EntityFactory protobuf text-format request
        req = (
            f'sdf: "{_escape_proto_str(sdf_str)}" '
            f'name: "{name}" '
            f'allow_renaming: false '
            f'pose {{position {{x: {x:.5f} y: {y:.5f} z: {z:.5f}}}}}'
        )

        service_path = f'/world/{self._world}/create'
        cmd = [
            self._gz_bin, 'service',
            '-s', service_path,
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000',
            '--req', req,
        ]

        try:
            result = subprocess.run(
                cmd, capture_output=True, text=True, timeout=8.0)
            ok = result.returncode == 0 and 'data: true' in result.stdout
        except subprocess.TimeoutExpired:
            ok = False
            result = type('R', (), {'stderr': 'timeout', 'stdout': ''})()

        if ok:
            response.success = True
            response.message = (
                f'Spawned {name} at ({x:.3f}, {y:.3f}, {z:.3f})')
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = (
                f'Failed to spawn {name}: {result.stderr.strip()}'
                f' | stdout: {result.stdout.strip()}')
            self.get_logger().error(response.message)

        return response


def _escape_proto_str(s: str) -> str:
    """Escape a string for use inside proto text-format double-quotes."""
    return s.replace('\\', '\\\\').replace('"', '\\"').replace('\n', '\\n')


def main(args=None):
    rclpy.init(args=args)
    node = ScrewSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
