#!/usr/bin/env python3
"""/sweep action server (mona_interfaces/action/Sweep) running the AI Festa sweep.

Single process: the sweep_only sequence node runs inside this process (the
per-sweep subprocess approach lost accept replies). Feedback `state` is each
stage the sequence publishes on /sweep_stage. Result: SUCCESS / STALL (safety
cutoff) / ERROR (anything else).

A goal is rejected while another one runs. Cancel is rejected: the arm cannot
be stopped mid-motion.

Parameter safety_monitor (default false): the sim has no joint current, so the
current/tracking-error cutoff is off; enable it on the real arm.
"""
import os
import sys
import threading

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def result_code(ok, stalled):
    if ok:
        return 'SUCCESS'
    return 'STALL' if stalled else 'ERROR'


def main():
    import rclpy
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from mona_interfaces.action import Sweep
    import obstacle_clear_sequence as ocs
    import sweep_only

    class SweepServer(Node):
        def __init__(self):
            super().__init__('sweep_server')
            self.safety_monitor = bool(self.declare_parameter('safety_monitor', False).value)
            self.running = False
            self.lock = threading.Lock()
            self.seq = None
            self.on_stage = None
            self.server = ActionServer(
                self, Sweep, 'sweep', self._execute,
                goal_callback=self._goal, cancel_callback=self._cancel,
                callback_group=ReentrantCallbackGroup())

        def _goal(self, _request):
            with self.lock:
                if self.running:
                    self.get_logger().warning('sweep goal rejected: already running')
                    return GoalResponse.REJECT
                self.running = True
            return GoalResponse.ACCEPT

        def _cancel(self, _goal_handle):
            self.get_logger().warning('sweep cancel rejected: the arm cannot be stopped mid-motion')
            return CancelResponse.REJECT

        def _sequence_node(self):
            if self.seq is None:
                server = self
                base = sweep_only.make_class(ocs)

                class Seq(base):
                    def publish_stage(self, stage):
                        super().publish_stage(stage)
                        if server.on_stage:
                            server.on_stage(stage)

                # a separate node: the sequence spins itself while it runs
                seq = Seq(safety_enabled=self.safety_monitor)
                if not seq.wait_servers():
                    self.get_logger().error('sweep: arm/gripper action servers not found')
                    seq.destroy_node()
                    return None
                self.seq = seq
            return self.seq

        def _execute(self, goal_handle):
            stalled = []

            def on_stage(stage):
                if stage == 'SAFETY_STOP':
                    stalled.append(stage)
                goal_handle.publish_feedback(Sweep.Feedback(state=stage))

            ok = False
            try:
                seq = self._sequence_node()
                if seq is not None:
                    self.on_stage = on_stage
                    ok = bool(seq.run_all())
            except Exception as e:  # noqa: BLE001 - any failure is reported as ERROR
                self.get_logger().error(f'sweep failed: {e!r}')
            finally:
                self.on_stage = None
                with self.lock:
                    self.running = False
            code = result_code(ok, bool(stalled))
            self.get_logger().info(f'sweep: {code}')
            if ok:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return Sweep.Result(result_code=code)

    rclpy.init()
    node = SweepServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
