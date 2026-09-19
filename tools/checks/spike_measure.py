#!/usr/bin/env python3
"""Send a step trajectory to the P1 spike axis and measure the response from /joint_states.

Needs rclpy, so it runs on the ROS system python (sourced by spike.sh), not tools/.venv.
Writes a JSON result to --out and exits non-zero if the response fails the criteria.
"""

import argparse
import json
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINT = 'rail_to_cart'
ACTION = '/joint_trajectory_controller/follow_joint_trajectory'
CONTROLLERS = ('joint_state_broadcaster', 'joint_trajectory_controller')


class Probe(Node):

    def __init__(self):
        super().__init__('spike_probe')
        self.samples = []  # (t_wall, position, velocity)
        self.create_subscription(JointState, '/joint_states', self._on_js, 50)
        self.client = ActionClient(self, FollowJointTrajectory, ACTION)
        self.list_cli = self.create_client(ListControllers, '/controller_manager/list_controllers')

    def controller_states(self):
        """Same query as `ros2 control list_controllers`; {} if the manager is not up."""
        if not self.list_cli.service_is_ready():
            return {}
        fut = self.list_cli.call_async(ListControllers.Request())
        spin_until(self, fut.done, 2.0)
        if not fut.done() or fut.result() is None:
            return {}
        return {c.name: c.state for c in fut.result().controller}

    def _on_js(self, msg):
        if JOINT not in msg.name:
            return
        i = msg.name.index(JOINT)
        vel = msg.velocity[i] if len(msg.velocity) > i else float('nan')
        self.samples.append((time.monotonic(), msg.position[i], vel))


def spin_until(node, pred, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
        if pred():
            return True
    return False


def analyse(samples, t0, start, target, tol):
    """Overshoot, settling time and oscillation count of the step response after t0."""
    resp = [(t - t0, p) for t, p, _ in samples if t >= t0]
    if not resp:
        return None
    step = target - start
    sign = 1.0 if step >= 0 else -1.0
    peak = max(sign * (p - start) for _, p in resp)
    overshoot_m = max(0.0, peak - abs(step))
    # Settling time: last instant the error was outside the tolerance band.
    settling = 0.0
    for t, p in resp:
        if abs(p - target) > tol:
            settling = t
    settled = abs(resp[-1][1] - target) <= tol
    # Oscillation: sign changes of the error beyond the band, after first reaching the band.
    crossings, last_sign, reached = 0, 0, False
    for _, p in resp:
        err = p - target
        if abs(err) <= tol:
            reached = True
            continue
        if reached:
            s = 1 if err > 0 else -1
            if last_sign and s != last_sign:
                crossings += 1
            last_sign = s
    return {
        'final_position_m': resp[-1][1],
        'final_error_m': resp[-1][1] - target,
        'peak_travel_m': peak,
        'overshoot_m': overshoot_m,
        'overshoot_pct': 100.0 * overshoot_m / abs(step) if step else 0.0,
        'settling_time_s': settling,
        'settled': settled,
        'oscillation_crossings': crossings,
        'n_samples': len(resp),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--distance', type=float, default=0.5)
    ap.add_argument('--move-time', type=float, default=2.0)
    ap.add_argument('--observe', type=float, default=3.0,
                    help='seconds to keep recording after the move time')
    ap.add_argument('--tol', type=float, default=0.005, help='settling band, m')
    ap.add_argument('--max-overshoot', type=float, default=0.005, help='m')
    ap.add_argument('--max-settling', type=float, default=3.0, help='s after goal sent')
    ap.add_argument('--controller-timeout', type=float, default=30.0,
                    help='s, counted from --launch-start')
    ap.add_argument('--launch-start', type=float, default=None,
                    help='epoch time the launch started (default: now)')
    ap.add_argument('--extra', default='{}', help='JSON merged into the report')
    args = ap.parse_args()

    result = json.loads(args.extra)
    result.update({'joint': JOINT, 'distance_m': args.distance, 'move_time_s': args.move_time,
                   'tolerance_m': args.tol, 'max_overshoot_m': args.max_overshoot,
                   'max_settling_s': args.max_settling})
    errors = []
    launch_start = args.launch_start if args.launch_start is not None else time.time()

    rclpy.init()
    node = Probe()
    try:
        states = {}

        def controllers_active():
            nonlocal states
            states = node.controller_states()
            return all(states.get(c) == 'active' for c in CONTROLLERS)

        wait = max(0.0, args.controller_timeout - (time.time() - launch_start))
        ok = spin_until(node, controllers_active, wait)
        result['controller_states'] = states
        if not ok:
            result['stage'] = 'controllers'
            errors.append(f'controllers not both active within {args.controller_timeout:g} s '
                          f'of launch: {states or "no controller_manager"}')
        else:
            result['controllers_active_s'] = round(time.time() - launch_start, 2)
            result['stage'] = 'trajectory'
            print(f"spike: controllers active after {result['controllers_active_s']} s")

        if errors:
            pass
        elif not spin_until(node, lambda: len(node.samples) > 5, 5.0):
            errors.append('no /joint_states for rail_to_cart within 5 s')
        elif not node.client.wait_for_server(timeout_sec=5.0):
            errors.append(f'action server {ACTION} not available')
        else:
            start = node.samples[-1][1]
            target = start + args.distance
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = [JOINT]
            pt = JointTrajectoryPoint()
            pt.positions = [target]
            pt.velocities = [0.0]
            sec = int(args.move_time)
            pt.time_from_start = Duration(sec=sec, nanosec=int((args.move_time - sec) * 1e9))
            goal.trajectory.points = [pt]

            t0 = time.monotonic()
            fut = node.client.send_goal_async(goal)
            spin_until(node, fut.done, 5.0)
            handle = fut.result() if fut.done() else None
            if handle is None or not handle.accepted:
                errors.append('trajectory goal rejected or not acknowledged')
            else:
                res_fut = handle.get_result_async()
                spin_until(node, lambda: False, args.move_time + args.observe)
                if res_fut.done():
                    r = res_fut.result().result
                    result['action_error_code'] = r.error_code
                    result['action_error_string'] = r.error_string
                    if r.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                        errors.append(f'action error_code {r.error_code}: {r.error_string}')
                else:
                    errors.append('action result not received')
                result['start_position_m'] = start
                result['target_position_m'] = target
                metrics = analyse(node.samples, t0, start, target, args.tol)
                if metrics is None:
                    errors.append('no samples after goal sent')
                else:
                    result.update(metrics)
                    if metrics['overshoot_m'] > args.max_overshoot:
                        errors.append(f"overshoot {metrics['overshoot_m']:.4f} m "
                                      f'> {args.max_overshoot} m')
                    if not metrics['settled']:
                        errors.append(f"not settled: final error {metrics['final_error_m']:.4f} m")
                    elif metrics['settling_time_s'] > args.max_settling:
                        errors.append(f"settling {metrics['settling_time_s']:.2f} s "
                                      f'> {args.max_settling} s')
                    if metrics['oscillation_crossings'] > 0:
                        errors.append(f"oscillation: {metrics['oscillation_crossings']} "
                                      'error sign changes outside the band')
    finally:
        node.destroy_node()
        rclpy.shutdown()

    result['errors'] = errors
    result['pass'] = not errors
    with open(args.out, 'w') as f:
        json.dump(result, f, indent=2)
    print(json.dumps(result, indent=2))
    return 0 if not errors else 1


if __name__ == '__main__':
    sys.exit(main())
