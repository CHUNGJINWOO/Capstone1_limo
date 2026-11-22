"""Run the original snapshot's full Phase 5 suite with only executor-thread shutdown joined."""
import os
from pathlib import Path
original=Path(os.environ['CAPSTONE45_SNAPSHOT'])/'fleet_control/tests/phase5/run_phase5.py'
s=original.read_text()
a='threading.Thread(target=ex.spin, daemon=True).start()'
b='spin_thread = threading.Thread(target=ex.spin, daemon=True); spin_thread.start()'
assert s.count(a)==1;s=s.replace(a,b)
a='        ex.shutdown(); rclpy.try_shutdown()'
b='        ex.shutdown(); spin_thread.join(timeout=5); obs.destroy_node(); rclpy.try_shutdown()'
assert s.count(a)==1;s=s.replace(a,b)
exec(compile(s,str(original),'exec'),{'__name__':'__main__','__file__':str(original)})
