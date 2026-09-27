#!/usr/bin/env python3
"""P5.4 measurement: every controller comes up, then the gantry visits three stations.

Needs rclpy, so it runs on the ROS system python (sourced by line_smoke.sh), not tools/.venv —
which is also why it reads the spec as plain YAML (the merged file `spec.py --dump` writes) and
carries its own small gz pose reader instead of importing world_measure.py, whose imports need
the venv.

  1. wait for every controller in the generated controllers yaml to be `active`, <= 30 s from
     the launch start;
  2. for each of the first three stations the sequence places at with a gantry tool, command the
     gantry whose zone holds it (`gantries.<id>.zone`, D-008) to that station's pose
     (pose - mount, yaw on C), wait for it to settle, then read where **Gazebo** says the TCP is —
     the pose of the Gantry_<n>_c link, which is where Gantry_<n>_Tcp sits (no offset) — and
     compare it with the station pose from the spec.

The error is measured against the physics engine, not against /joint_states: a joint that reports
its target and a link that is somewhere else would pass the second and fail the first.
"""

import argparse
import json
import math
import subprocess
import sys
import time

import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

N_STATIONS = 3


def tcp_link(gid):
    """Gantry_<n>_Tcp is fixed on Gantry_<n>_c with no offset (gantry.urdf.xacro)."""
    return f"Gantry_{gid.removeprefix('gantry_')}_c"


class Probe(Node):

    def __init__(self):
        super().__init__('line_probe')
        self.positions = {}
        self.create_subscription(JointState, '/joint_states', self._on_js, 50)
        self.clients_by_gantry = {}
        self.list_cli = self.create_client(ListControllers, '/controller_manager/list_controllers')

    def client(self, gid):
        if gid not in self.clients_by_gantry:
            self.clients_by_gantry[gid] = ActionClient(
                self, FollowJointTrajectory, f'/{gid}_controller/follow_joint_trajectory')
        return self.clients_by_gantry[gid]

    def _on_js(self, msg):
        self.positions.update(zip(msg.name, msg.position))

    def controller_states(self):
        """Same query as `ros2 control list_controllers`; {} if the manager is not up."""
        if not self.list_cli.service_is_ready():
            return {}
        fut = self.list_cli.call_async(ListControllers.Request())
        spin_until(self, fut.done, 2.0)
        if not fut.done() or fut.result() is None:
            return {}
        return {c.name: c.state for c in fut.result().controller}


def spin_until(node, pred, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
        if pred():
            return True
    return False


def gz_link_pose(world, link, timeout=5.0):
    """[x, y, z] of one link from a single Pose_V message, or None.

    Text protobuf omits zero-valued fields, so a missing `x:` line means x = 0. The first entry
    for a name wins, as in world_measure.parse_pose_v.
    """
    try:
        text = subprocess.run(['gz', 'topic', '-e', '-t', f'/world/{world}/pose/info', '-n', '1'],
                              capture_output=True, text=True, timeout=timeout).stdout
    except (OSError, subprocess.TimeoutExpired):
        return None
    name, section, xyz = None, None, [0.0, 0.0, 0.0]
    for raw in text.splitlines():
        line = raw.strip()
        if line.startswith('pose {'):
            name, section, xyz = None, None, [0.0, 0.0, 0.0]
        elif line.startswith('name:') and '"' in line:
            name = line.split('"')[1]
        elif line.startswith(('position {', 'orientation {')):
            section = line.split()[0]
        elif line == '}':
            if section is not None:
                section = None
            elif name == link:
                return xyz
            else:
                name = None
        elif section == 'position' and ':' in line:
            axis, _, value = line.partition(':')
            if axis.strip() in ('x', 'y', 'z'):
                xyz['xyz'.index(axis.strip())] = float(value)
    return None


def expected_controllers(path):
    with open(path) as f:
        manager = yaml.safe_load(f)['controller_manager']['ros__parameters']
    return [k for k, v in manager.items() if isinstance(v, dict) and 'type' in v]


def gantry_stations(spec):
    """The first N stations the sequence places at with a gantry tool, in sequence order, each
    with the gantry whose zone holds it."""
    owner = {st: gid for gid, g in spec['gantries'].items() for st in (g.get('zone') or [])}
    seen = []
    for step in spec['sequence']:
        if step.get('tool') and step['station'] not in [s for s, _ in seen]:
            seen.append((step['station'], owner.get(step['station'])))
    return seen[:N_STATIONS]


def move_time(delta, axes, fraction):
    """Seconds for a rest-to-rest move of `delta` on every axis, from the spec's limits.

    The controller interpolates a cubic between the two rest points, whose peaks are
    v = 1.5 d / t and a = 6 d / t^2, so t is chosen to keep both under `fraction` of the axis's
    v_max / a_max. Planning at the limit is not a station-reach check: the X axis (moved mass
    ~50 kg, effort = mass x a_max) fell 5 cm behind a move that asked for 87% of a_max.
    """
    t = 1.0
    for name, d in delta.items():
        d = abs(d)
        v, a = fraction * axes[name]['v_max'], fraction * axes[name]['a_max']
        t = max(t, 1.5 * d / v, math.sqrt(6.0 * d / a))
    return t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--spec', required=True, help='merged spec YAML (spec.py --dump)')
    ap.add_argument('--controllers', required=True, help='generated controllers yaml')
    ap.add_argument('--world', default='main_line')
    ap.add_argument('--tol-mm', type=float, default=1.0)
    ap.add_argument('--speed-fraction', type=float, default=0.5,
                    help='plan moves at this fraction of the spec v_max / a_max')
    ap.add_argument('--settle', type=float, default=1.5, help='s to wait after a move ends')
    ap.add_argument('--controller-timeout', type=float, default=30.0,
                    help='s, counted from --launch-start')
    ap.add_argument('--launch-start', type=float, default=None,
                    help='epoch time the launch started (default: now)')
    ap.add_argument('--extra', default='{}', help='JSON merged into the report')
    args = ap.parse_args()

    result = json.loads(args.extra)
    result.update({'tolerance_mm': args.tol_mm, 'speed_fraction': args.speed_fraction,
                   'stations': []})
    errors = []
    launch_start = args.launch_start if args.launch_start is not None else time.time()
    with open(args.spec) as f:
        spec = yaml.safe_load(f)
    wanted = expected_controllers(args.controllers)
    result['controllers_expected'] = wanted

    rclpy.init()
    node = Probe()
    try:
        states = {}

        def all_active():
            nonlocal states
            states = node.controller_states()
            return all(states.get(c) == 'active' for c in wanted)

        ok = spin_until(node, all_active, max(0.0, args.controller_timeout
                                              - (time.time() - launch_start)))
        result['controller_states'] = states
        if not ok:
            result['stage'] = 'controllers'
            errors.append(f'controllers not all active within {args.controller_timeout:g} s of '
                          f'launch: {states or "no controller_manager"}')
        else:
            result['controllers_active_s'] = round(time.time() - launch_start, 2)
            result['stage'] = 'gantry'
            print(f"line_smoke: {len(wanted)} controllers active after "
                  f"{result['controllers_active_s']} s")

        names = gantry_stations(spec) if not errors else []
        if not errors and len(names) < N_STATIONS:
            errors.append(f'the sequence places at {len(names)} station(s) with a gantry '
                          f'tool, need {N_STATIONS}')
        for sid, gid in names if not errors else []:
            if gid is None:
                errors.append(f'{sid}: no gantry zone holds it')
            elif not node.client(gid).wait_for_server(timeout_sec=5.0):
                errors.append(f'action server /{gid}_controller/follow_joint_trajectory '
                              'not available')
            elif not spin_until(node, lambda: f'{gid}_x' in node.positions, 5.0):
                errors.append(f'no /joint_states for {gid} within 5 s')

        if not errors:
            for sid, gid in names:
                axes = spec['gantries'][gid]['axes']
                mount = spec['gantries'][gid]['mount']
                joints = [f'{gid}_{a}' for a in 'xyz'] + ([f'{gid}_c'] if axes.get('c') else [])
                pose = spec['stations'][sid]['pose']
                target = {f'{gid}_{a}': pose[i] - mount[i] for i, a in enumerate('xyz')}
                if f'{gid}_c' in joints:
                    target[f'{gid}_c'] = pose[3] or 0.0
                delta = {j: target[j] - node.positions[j] for j in joints}
                t = move_time({j[-1]: d for j, d in delta.items()}, axes, args.speed_fraction)

                goal = FollowJointTrajectory.Goal()
                goal.trajectory.joint_names = joints
                pt = JointTrajectoryPoint()
                pt.positions = [target[j] for j in joints]
                pt.velocities = [0.0] * len(joints)
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                goal.trajectory.points = [pt]

                row = {'station': sid, 'gantry': gid, 'spec_path': f'stations.{sid}.pose',
                       'target_pose_m': pose[:3], 'target_joints': target,
                       'move_time_s': round(t, 2)}
                result['stations'].append(row)
                fut = node.client(gid).send_goal_async(goal)
                spin_until(node, fut.done, 5.0)
                handle = fut.result() if fut.done() else None
                if handle is None or not handle.accepted:
                    errors.append(f'{sid}: trajectory goal rejected or not acknowledged')
                    break
                res = handle.get_result_async()
                spin_until(node, res.done, t + 10.0)
                if not res.done():
                    errors.append(f'{sid}: no result within {t + 10.0:.0f} s')
                    break
                r = res.result().result
                row['action_error'] = f'{r.error_code}: {r.error_string}'
                if r.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                    errors.append(f'{sid}: action error_code {r.error_code}: {r.error_string}')
                    break
                spin_until(node, lambda: False, args.settle)

                tcp = gz_link_pose(args.world, tcp_link(gid))
                row['joint_error_m'] = {j: node.positions[j] - target[j] for j in joints}
                if tcp is None:
                    errors.append(f'{sid}: no pose for link {tcp_link(gid)} from gz')
                    break
                err_mm = math.dist(tcp, pose[:3]) * 1000.0
                row.update(tcp_gz_m=tcp, tcp_error_mm=round(err_mm, 4),
                           tcp_error_xyz_mm=[round((tcp[i] - pose[i]) * 1000.0, 4)
                                             for i in range(3)])
                print(f'line_smoke: {sid}: TCP error {err_mm:.3f} mm')
                if not err_mm < args.tol_mm:
                    errors.append(f'{sid}: TCP is {err_mm:.3f} mm from the station pose '
                                  f'(limit {args.tol_mm:g} mm)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

    result['errors'] = errors
    result['pass'] = not errors
    with open(args.out, 'w') as f:
        json.dump(result, f, indent=2)
    return 0 if not errors else 1


if __name__ == '__main__':
    sys.exit(main())
