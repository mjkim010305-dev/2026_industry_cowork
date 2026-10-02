#!/usr/bin/env python3
"""/sweep without the part (festa_demo, festa_bringup's pick:=false mode).

The collaborator's /sweep server (festa_action/sweep_action_server.py) runs
obstacle_clear_sequence.py `all`: rear place -> sweep -> rear re-pick. With
nothing carried, this file runs that same server with the sequence swapped
for one that ends at P_HOME:

    (current pose; the sim starts at all zeros) -> P_HOME
    -> turned right (P_HOME shape) -> P_SIDE_PRE_SWEEP -> P_PRE_SWEEP
    -> P_CONTACT -> P_SWEEP_END (safety cutoff) -> P_RETREAT   (run_sweep)
    -> turned left (P_HOME shape) -> P_HOME

The arm turns while folded so it does not come down through the box. Poses,
safety cutoff and /sweep_stage are the collaborator's; obstacle_clear_sequence
is not modified.

festa_demo: self-contained copy of festa_bringup/scripts/sweep_only.py's
run_sequence() part - obstacle_clear_sequence is imported from this same
directory (no FESTA_ACTION_DIR env var, no `server` mode: this package's
sweep_action_server.py points SEQUENCE_SCRIPT directly at this file).

    python3 sweep_only.py all [--no-safety]
"""
import os
import sys

# Copied from src/festa_manipulation/festa_action/rear_pick.py (P_HOME).
P_HOME = [
    -0.0015339807878856412,
    -1.0461748973380072,
     1.0753205323078345,
     0.009203884727313847,
]


def make_class(ocs):
    """festa_demo (2026-10-02): the sweep-only sequence class, also used in-process by handle_box.py."""

    class SweepOnly(ocs.ObstacleClearSequence):

        def moves(self, steps):
            for label, pose in steps:
                self.publish_stage(label)
                if self.move_arm(pose, label) != 'ok':
                    return False
            return True

        def run_all(self):
            self.publish_stage('SEQUENCE_START')
            ok = (self.moves([('P_HOME', P_HOME),
                              ('P_HOME_RIGHT', [ocs.P_SIDE_PRE_SWEEP[0]] + P_HOME[1:]),
                              ('P_SIDE_PRE_SWEEP', ocs.P_SIDE_PRE_SWEEP),
                              ('P_PRE_SWEEP', ocs.P_PRE_SWEEP)])
                  and self.run_sweep()
                  and self.moves([('P_HOME_LEFT', [ocs.P_RETREAT[0]] + P_HOME[1:]),
                              ('P_HOME', P_HOME)]))
            self.publish_stage('SEQUENCE_COMPLETE' if ok else 'SEQUENCE_FAILED')
            return ok

    return SweepOnly


def run_sequence():
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import obstacle_clear_sequence as ocs

    # ocs.main() builds the node from this module-level name.
    ocs.ObstacleClearSequence = make_class(ocs)
    ocs.main()


if __name__ == '__main__':
    run_sequence()
