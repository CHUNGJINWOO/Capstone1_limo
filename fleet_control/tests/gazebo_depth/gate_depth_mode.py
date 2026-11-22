"""SIMULATION ONLY (2026-11-22, Claude): run a Gate script with depth_mode taken from $GATE_DEPTH_MODE.

tests/web_mapping_isolated/stack.launch.py (shared, in use by another session, NOT modified) always passes depth_mode:=disabled.
This wrapper is given as GATE_SCRIPT=<this file> with GATE_SCRIPT_REAL=<the Gate under test>; it only replaces that one
'-p depth_mode:=...' argument and then runs the Gate file unchanged (runpy, same process, same node name and remaps).
"""
import os
import runpy
import sys

mode = os.environ.get('GATE_DEPTH_MODE', 'required')
assert mode in ('required', 'optional', 'disabled'), mode
real = os.environ['GATE_SCRIPT_REAL']
sys.argv = [real] + [f'depth_mode:={mode}' if a.startswith('depth_mode:=') else a for a in sys.argv[1:]]
runpy.run_path(real, run_name='__main__')
