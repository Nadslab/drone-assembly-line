#!/usr/bin/env python3
"""
Virtual conveyor transport node.

Service  /conveyor/move_to_station  (drone_assembly_cell/srv/MoveToStation)
  Teleports a named Gazebo model to one of the 5 assembly stations on the
  conveyor centreline using the gz-sim set_pose world service.

Publisher /conveyor/state  (std_msgs/String, 1 Hz)
  "drone_id:<id> station:<n>"  — reflects the last successful move.
"""

import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from drone_assembly_cell.srv import MoveToStation


# Station poses on the conveyor centreline (world frame).
# Values match the station marker positions in frame_assembly.sdf.
_STATIONS = {
    1: (-1.2, 0.0, 0.95),  # screw_robot + lerobot_1
    2: (-0.6, 0.0, 0.95),  # lerobot_2
    3: ( 0.0, 0.0, 0.95),  # solder_robot
    4: ( 0.6, 0.0, 0.95),  # lerobot_3
}


class ConveyorNode(Node):
    def __init__(self):
        super().__init__('conveyor_node')

        self._world = self.declare_parameter(
            'world', 'frame_assembly_cell').get_parameter_value().string_value
        self._gz_bin = self.declare_parameter(
            'gz_bin',
            '/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz'
        ).get_parameter_value().string_value

        # Station poses as parameters (informational; runtime uses _STATIONS dict)
        for n, (x, y, z) in _STATIONS.items():
            self.declare_parameter(f'station_{n}.x', x)
            self.declare_parameter(f'station_{n}.y', y)
            self.declare_parameter(f'station_{n}.z', z)

        self._srv = self.create_service(
            MoveToStation, '/conveyor/move_to_station', self._move_cb)

        self._pub = self.create_publisher(String, '/conveyor/state', 10)
        self._timer = self.create_timer(1.0, self._publish_state)

        self._current_drone_id = 'none'
        self._current_station  = 0

        self.get_logger().info(
            f'Conveyor node ready — world: {self._world} '
            f'| service: /conveyor/move_to_station '
            f'| state topic: /conveyor/state')

    # ── service handler ────────────────────────────────────────────────────

    def _move_cb(self, request, response):
        drone_id = request.drone_id.strip()
        station  = request.target_station

        if not drone_id:
            response.success = False
            response.message = 'drone_id must not be empty'
            return response

        if station not in _STATIONS:
            response.success = False
            response.message = (
                f'Invalid station {station}; valid range is 1–4')
            return response

        x, y, z = _STATIONS[station]

        # gz-sim set_pose service (same pattern as screw_spawner)
        req_str = (
            f'name: "{drone_id}" '
            f'position {{x: {x:.4f} y: {y:.4f} z: {z:.4f}}} '
            f'orientation {{w: 1.0}}'
        )
        cmd = [
            self._gz_bin, 'service',
            '-s', f'/world/{self._world}/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000',
            '--req', req_str,
        ]

        try:
            result = subprocess.run(
                cmd, capture_output=True, text=True, timeout=8.0)
            ok = result.returncode == 0 and 'data: true' in result.stdout
        except subprocess.TimeoutExpired:
            ok = False
            result = type('R', (), {'stderr': 'timeout', 'stdout': ''})()

        if ok:
            self._current_drone_id = drone_id
            self._current_station  = station
            response.success = True
            response.message = (
                f'Moved {drone_id!r} to station {station} '
                f'({x:.3f}, {y:.3f}, {z:.3f})')
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = (
                f'set_pose failed for {drone_id!r} → station {station}: '
                f'{result.stderr.strip()} | stdout: {result.stdout.strip()}')
            self.get_logger().error(response.message)

        return response

    # ── state publisher ────────────────────────────────────────────────────

    def _publish_state(self):
        msg = String()
        msg.data = (
            f'drone_id:{self._current_drone_id} '
            f'station:{self._current_station}')
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ConveyorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
