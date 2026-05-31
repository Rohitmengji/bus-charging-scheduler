"""Subprocess worker — runs OR-Tools CP-SAT solver in an isolated process.

Streamlit's threaded script runner causes OR-Tools' native C++ solver to
deadlock on macOS.  This script is invoked via subprocess to bypass that.
Reads a pickled Scenario from stdin, solves, writes a pickled ScheduleResult
to stdout.
"""
import pickle
import sys

from scheduler import _run_scheduler_impl

if __name__ == "__main__":
    scenario = pickle.loads(sys.stdin.buffer.read())
    result = _run_scheduler_impl(scenario)
    sys.stdout.buffer.write(pickle.dumps(result))
