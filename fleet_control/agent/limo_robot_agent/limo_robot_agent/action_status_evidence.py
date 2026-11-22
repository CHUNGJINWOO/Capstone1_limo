"""Idle evidence for one ROS 2 action from its status topic, without assuming periodic status messages.

rclcpp/rclpy action servers publish ``<action>/_action/status`` (reliable, transient-local, depth 1) only when a goal
changes state. An idle server that never had a goal publishes nothing at all (checked 2026-11-20 with the Humble
``action_tutorials_cpp`` and ``action_tutorials_py`` Fibonacci servers: 0 messages in 3 s, cancel-all -> rc 0, empty).
A message-age freshness rule therefore locks out every idle Nav2 forever. Continuity is used instead:

* exactly one publisher on the status topic (one action server; a second could hide goals),
* the same publisher GID has been present for at least ``settle_s`` (its transient-local status, if any, had time
  to arrive),
* the newest received status shows no non-terminal goal, or none was ever received (a goal acceptance always
  publishes a status, so a silent unique server has no goal).

A status message that arrived before the current publisher appeared may belong to a previous server process. If it
shows an active goal it is not dropped: the caller must supply an independent proof made for the *current* server
(``vouched_gid``, e.g. the Motion Watchdog's cancel-all idle proof); otherwise the evidence stays negative.

Identity and attribution (2026-11-20, after Codex review):
* every endpoint counted on the topic must have a readable, non-empty GID; otherwise identity is UNKNOWN (a failed
  lookup is never treated as "fewer publishers");
* Humble rclpy does not expose the sending GID per message. A status received within `settle_s` after a publisher-set
  change cannot be attributed to the old or the new writer and is UNATTRIBUTED;
* after any change (generation > 1) positive evidence additionally needs an independent vouch for the current server
  (`vouched_gid` = Motion Watchdog cancel-all proof for this GID, or `vouched=True` from the caller), or a status that
  arrived after the attribution window.
This is graph/consistency evidence, not DDS authentication and not proof that a goal ended (recovery uses Nav2
terminal results per goal UUID). It does not replace the Safety Gate or the watchdog.
"""
import threading

from .ros_graph import gid_hex as _gid_hex

from action_msgs.msg import GoalStatus

TERMINAL = (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED)


def gid_hex(info):
    return _gid_hex(info)          # Phase 3.5: shared helper; an all-zero GID is returned (the callers reject it)


def active_uuids(msg):
    out = []
    for g in getattr(msg, 'status_list', []) or []:
        if getattr(g, 'status', None) not in TERMINAL:
            try:
                out.append(bytes(g.goal_info.goal_id.uuid).hex())
            except Exception:
                out.append('?')
    return out


class ActionStatusEvidence:
    def __init__(self, label, expected_node=None, settle_s=1.0):
        self.label, self.expected_node, self.settle_s = label, expected_node, float(settle_s)
        self.lock = threading.Lock()
        self.gids = None            # tuple of publisher GIDs last seen in the graph
        self.since = None           # monotonic time the current publisher set appeared
        self.last = None            # (monotonic receipt time, tuple(active uuids), total entries)
        self.generation = 0         # number of distinct publisher sets observed
        self.identity_ok = None

    def on_message(self, now, msg):
        with self.lock:
            unattributed = self.generation > 1 and self.since is not None and now < self.since + self.settle_s
            self.last = (now, tuple(active_uuids(msg)), len(getattr(msg, 'status_list', []) or []), unattributed)

    def observe(self, now, infos):
        raw = [gid_hex(i) for i in infos]
        identity_ok = all(g and any(bytes.fromhex(g)) for g in raw)
        gids = tuple(sorted(g for g in raw if g)) if identity_ok else None
        names = tuple(sorted(getattr(i, 'node_name', '') for i in infos))
        with self.lock:
            self.identity_ok = identity_ok
            if gids is None:
                pass        # unknown identity: evaluate() refuses; the last known set is kept for comparison
            elif gids != self.gids:
                # A GID is unique per writer: the SAME set reappearing after a failed lookup is the same writer(s), so
                # continuity is kept; any different set starts a new generation and a new settle window.
                self.gids, self.since = gids, now
                if gids:
                    self.generation += 1
            self.names = names
            self.count = len(infos)

    def evaluate(self, now, infos, vouched_gid=None, vouched=False):
        """Return a JSON-safe dict; ``ok`` is True only with positive idle evidence."""
        self.observe(now, infos)
        with self.lock:
            gids, since, last, names = self.gids, self.since, self.last, getattr(self, 'names', ())
            identity_ok, generation, count = self.identity_ok, self.generation, getattr(self, 'count', 0)
        if not identity_ok:
            return {'action': self.label, 'publishers': count, 'publisher_gid': None, 'node': None, 'stable_s': None,
                    'received': last is not None, 'status_from_current_server': None, 'active_count': None,
                    'active_goal_uuids': [], 'generation': generation, 'ok': False,
                    'reason': f'{self.label}_SERVER_IDENTITY_UNKNOWN'}
        n = len(gids or ())
        stable = None if since is None else round(max(0.0, now - since), 3)
        current = None if last is None or since is None else last[0] >= since
        active = None if last is None else len(last[1])
        ev = {'action': self.label, 'publishers': n, 'publisher_gid': gids[0] if n == 1 else None,
              'node': names[0] if n == 1 else None, 'stable_s': stable, 'received': last is not None,
              'status_from_current_server': current, 'active_count': active,
              'active_goal_uuids': list(last[1]) if last else [], 'generation': generation, 'ok': False, 'reason': None}
        vouch = vouched or (vouched_gid is not None and n == 1 and vouched_gid == gids[0])
        attributed = last is not None and current and not last[3]
        if n == 0:
            ev['reason'] = f'{self.label}_SERVER_MISSING'
        elif n != 1:
            ev['reason'] = f'{self.label}_SERVER_NOT_UNIQUE'
        elif self.expected_node and names[0] != self.expected_node:
            ev['reason'] = f'{self.label}_UNEXPECTED_SERVER_NODE'
        elif stable is None or stable < self.settle_s:
            ev['reason'] = f'{self.label}_STATUS_SETTLING'
        elif last is not None and last[3] and not vouch:
            ev['reason'] = f'{self.label}_STATUS_UNATTRIBUTED'      # may be a delayed message of the previous writer
        elif active:
            if current is False and vouch:
                ev['ok'] = True          # stale goal of a previous server; current server independently proven idle
                ev['reason'] = None
            else:
                ev['reason'] = f'{self.label}_GOAL_ACTIVE' if current else f'{self.label}_SERVER_CHANGED_WITH_ACTIVE_GOAL'
        elif generation > 1 and not vouch and not attributed:
            ev['reason'] = f'{self.label}_SERVER_CHANGED_UNVOUCHED'
        else:
            ev['ok'] = True
        if ev['ok'] and active is None:
            ev['active_count'] = 0
        return ev
