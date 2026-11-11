"""깊이 관측 heartbeat 수신과 실제 원본 timestamp를 함께 검사한다."""
import math


def depth_issue(status, received_at, now, ros_now, timeout):
    if status is None:
        return 'DEPTH_MISSING', 'No depth filter status received'
    if now-received_at > timeout:
        return 'DEPTH_STATUS_STALE', f'Depth heartbeat age={now-received_at:.3f}s'
    if status.get('state') != 'ready':
        return 'DEPTH_NOT_READY', str(status.get('code', 'unknown'))+': '+str(status.get('message', ''))
    stamp = status.get('source_stamp_sec')
    if isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
        return 'DEPTH_INVALID_STATUS', 'Missing finite camera stamp'
    age = ros_now-stamp
    if not -.2 <= age <= timeout:
        return 'DEPTH_SOURCE_STALE', f'Camera source age={age:.3f}s'
    return None
