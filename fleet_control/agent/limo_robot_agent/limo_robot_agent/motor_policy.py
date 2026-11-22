"""Fail-closed simulation-only exception. Does not modify or bypass the Safety Gate."""
def motor_path_allowed(publishers, *, mode='real', simulated=False, allow_sim_safety_gate=False, use_sim_time=False):
    if publishers is None:
        return False
    if not publishers:
        return True  # Existing locked/no-publisher behavior, unchanged for real robots.
    return (mode == 'sim' and simulated is True and allow_sim_safety_gate is True and use_sim_time is True
            and len(publishers) == 1 and publishers[0].node_name == 'stall_detector_node'
            and publishers[0].node_namespace == '/')
