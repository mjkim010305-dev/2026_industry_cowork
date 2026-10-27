#!/usr/bin/env python3
"""Pure-logic tests for sweep_server / sweep_only. Import only, no ROS graph."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from sweep_server import result_code  # noqa: E402
import sweep_only  # noqa: E402


def test_1_result_codes():
    assert result_code(True, False) == "SUCCESS"
    assert result_code(False, True) == "STALL"
    assert result_code(False, False) == "ERROR"


def test_2_p_home_unchanged():
    assert len(sweep_only.P_HOME) == 4
    assert sweep_only.P_HOME[1:] == [-1.0461748973380072, 1.0753205323078345, 0.009203884727313847]
