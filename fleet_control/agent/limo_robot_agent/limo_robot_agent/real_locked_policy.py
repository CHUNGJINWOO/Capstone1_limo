"""REAL_LOCKED deployment candidate: unconditionally blocked pending physical evidence.

No software parameter, ROS topic, log or operator-supplied JSON can prove that a
particular robot's motor energy path is physically isolated. This module cannot
issue or validate a physical-interlock attestation and never authorizes startup.
"""

BLOCK_REASON = (
    'REAL_LOCKED unavailable: no independently verified physical motor isolation '
    'evidence producer and no approved process ownership contract'
)


def require_available(*, mode, requested, recording_enabled=False):
    """Reject real recording and the candidate before creating robot/child processes."""
    normalized = str(mode).strip().lower().replace('-', '_')
    if requested or normalized == 'real_locked' or (normalized == 'real' and recording_enabled):
        raise RuntimeError(BLOCK_REASON)
