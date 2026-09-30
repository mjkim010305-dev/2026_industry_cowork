#!/usr/bin/env python3
"""/sweep without the part (festa_scenario.launch.py pick:=false).

The collaborator's /sweep server (festa_action/sweep_action_server.py) runs
obstacle_clear_sequence.py `all`: rear place -> sweep -> rear re-pick. With
nothing carried, this file runs that same server with the sequence swapped
for one that starts and ends at P_HOME:

    P_HOME -> turned right (P_HOME shape) -> P_SIDE_PRE_SWEEP -> P_PRE_SWEEP
    -> P_CONTACT -> P_SWEEP_END (safety cutoff) -> P_RETREAT   (run_sweep)
    -> turned left (P_HOME shape) -> P_HOME

The arm turns while folded so it does not come down through the box. Poses,
safety cutoff and /sweep_stage are the collaborator's; festa_action is not
modified.

    python3 sweep_only.py server <festa_action dir> [--ros-args ...]
    (the server runs this file again as: sweep_only.py all [--no-safety])
"""
import os
import sys


def serve(festa_action):
    os.environ['FESTA_ACTION_DIR'] = festa_action
    sys.path.insert(0, festa_action)
    import sweep_action_server
    sweep_action_server.SEQUENCE_SCRIPT = os.path.abspath(__file__)
    sweep_action_server.main()


def run_sequence():
    sys.path.insert(0, os.environ['FESTA_ACTION_DIR'])
    import obstacle_clear_sequence as ocs
    from rear_pick import P_HOME

    class SweepOnly(ocs.ObstacleClearSequence):

        def moves(self, steps):
            for label, pose in steps:
                self.publish_stage(label)
                if self.move_arm(pose, label) != 'ok':
                    return False
            return True

        def run_all(self):
            self.publish_stage('SEQUENCE_START')
            ok = (self.check_pose(P_HOME, 'P_HOME')
                  and self.moves([('P_HOME_RIGHT', [ocs.P_SIDE_PRE_SWEEP[0]] + P_HOME[1:]),
                                  ('P_SIDE_PRE_SWEEP', ocs.P_SIDE_PRE_SWEEP),
                                  ('P_PRE_SWEEP', ocs.P_PRE_SWEEP)])
                  and self.run_sweep()
                  and self.moves([('P_HOME_LEFT', [ocs.P_RETREAT[0]] + P_HOME[1:]),
                                  ('P_HOME', P_HOME)]))
            self.publish_stage('SEQUENCE_COMPLETE' if ok else 'SEQUENCE_FAILED')
            return ok

    # ocs.main() builds the node from this module-level name.
    ocs.ObstacleClearSequence = SweepOnly
    ocs.main()


if __name__ == '__main__':
    if len(sys.argv) >= 3 and sys.argv[1] == 'server':
        serve(sys.argv[2])
    else:
        run_sequence()
