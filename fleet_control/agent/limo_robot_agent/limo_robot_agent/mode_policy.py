"""Motion Watchdog mode policy (G2, 2026-11-23): which velocity source may be forwarded in the operating mode the Agent reports.

A real Agent has no Arbiter, so nothing selected a motion source by operating mode. This is the Watchdog-side selector: the Agent includes the
server's operating mode in /fleet/speed_limit; the Watchdog forwards a non-zero command only from the input channel that mode selects. Everything else is zero (the command is dropped and zero is published).

  MANUAL      channel 'manual' = /fleet/manual_cmd_vel  (the on-board joystick, publisher manual_mapping_teleop)
  MAPPING     channel 'nav'    = /fleet/nav_cmd_vel     (Nav2 smoother while the Agent owns a Frontier goal; goal ownership is the Watchdog's own rule)
  NAVIGATION  channel 'nav'                              (Nav2 smoother for a Fleet mission goal)
Both channels can exist in the graph at the same time; only the selected one is forwarded, and a non-zero command on the OTHER channel while a mode
selects one is a source conflict (the Watchdog goes FAILSAFE).
  IDLE / TRANSITIONING / FAULT / unknown / no message / stale message: nothing

Same trust model as the speed clamp: a message counts only from the single allowed publisher named by the graph and carrying the Agent boot the Watchdog
sees; narrowing (to IDLE or to any mode that allows less) takes effect at once, but GRANTING a source needs the same message twice, >= 0.15 s apart
(a second writer that appears between graph updates cannot grant itself motion with one message). DDS node names and GIDs are graph identifiers,
not authentication. Disabled unless the Watchdog parameter `mode_gate` is true; the state is never persisted (a restarted Watchdog allows nothing until
the Agent tells it the current mode again, twice).
"""
import threading
import time

# mode -> the ONE input channel the Watchdog forwards: 'manual' = /fleet/manual_cmd_vel (on-board joystick), 'nav' = /fleet/nav_cmd_vel (Nav2 smoother)
MODE_CHANNEL = {'MANUAL': 'manual', 'MAPPING': 'nav', 'NAVIGATION': 'nav'}
ALLOWED_INPUT_NODES = {'manual': ('manual_mapping_teleop',), 'nav': ('velocity_smoother',)}   # who may publish each channel (graph names, not authentication)
KNOWN_MODES = ('IDLE', 'MANUAL', 'MAPPING', 'NAVIGATION', 'TRANSITIONING', 'FAULT', 'UNKNOWN')
# who must own the Nav2 goal for the nav channel to be forwarded (the Arbiter's GOAL_OWNER): Frontier goals in MAPPING, Fleet mission goals in NAVIGATION
GOAL_OWNER = {'MAPPING': 'frontier', 'NAVIGATION': 'fleet'}
FRESH_S = 1.5
GRANT_CONFIRM_S = 0.15


class ModePolicy:
    def __init__(self, mono=time.monotonic):
        self.lock = threading.RLock()
        self.mono = mono
        self.cur = None            # {'name','revision','epoch','seen_t','boot'}
        self.pending = None        # (key, first_seen)
        self.counters = {'accepted': 0, 'rejected': 0, 'mode_blocked': 0}
        self.last_reject = None

    def accept(self, data, hb_boot, publisher_ok):
        with self.lock:
            now = self.mono()
            if not publisher_ok or not isinstance(data, dict) or hb_boot is None or data.get('agent_boot_id') != hb_boot:
                self.counters['rejected'] += 1; self.last_reject = 'PUBLISHER_OR_BOOT_MISMATCH'; self._narrow(); return False
            m = data.get('mode')
            if not isinstance(m, dict) or m.get('name') not in KNOWN_MODES or type(m.get('revision')) is not int:
                self.counters['rejected'] += 1; self.last_reject = 'MALFORMED_OR_MISSING_MODE'; self._narrow(); return False
            g = data.get('goal') if isinstance(data.get('goal'), dict) else {}
            owner, cancel = g.get('owner'), g.get('cancel_requested') is True
            # MANUAL needs NO Agent-owned goal (a goal under MANUAL is a conflict: nothing is granted); MAPPING/NAVIGATION need the goal the mode owns and no cancel in progress
            goal_ok = (owner is None) if m['name'] == 'MANUAL' else (owner == GOAL_OWNER.get(m['name']) and not cancel)
            new = {'name': m['name'], 'revision': m['revision'], 'epoch': m.get('epoch') if type(m.get('epoch')) is int else None, 'seen_t': now, 'granted_t': now, 'boot': hb_boot, 'goal_ok': goal_ok, 'owner': owner}
            key = (new['name'], new['revision'], new['epoch'], hb_boot, goal_ok)
            cur = self.cur
            cur_key = None if cur is None else (cur['name'], cur['revision'], cur['epoch'], cur['boot'], cur['goal_ok'])
            grants = new['name'] in MODE_CHANNEL and goal_ok
            if key == cur_key:
                cur['seen_t'] = now; self.counters['accepted'] += 1; self.pending = None; return True
            if not grants:
                self.cur, self.pending = new, None; self.counters['accepted'] += 1; return True          # narrowing: immediately
            # granting (a new mode or revision that allows a source): the same message twice, >= GRANT_CONFIRM_S apart; the old mode's permission ends now
            if self.pending is None or self.pending[0] != key:
                self.pending = (key, now); self.cur = None; self.counters['rejected'] += 1; self.last_reject = 'GRANT_AWAITING_SECOND_MESSAGE'; return False
            if now - self.pending[1] < GRANT_CONFIRM_S:
                return False
            self.cur, self.pending = new, None; self.counters['accepted'] += 1; return True

    def _narrow(self):
        """A bad message never keeps an old grant alive: the current permission is dropped."""
        self.cur = None; self.pending = None

    def permits(self, channel, hb_boot):
        """(allowed, reason) for a non-zero command arriving on `channel` ('nav' or 'manual') right now."""
        with self.lock:
            c = self.cur
            if c is None or c['boot'] != hb_boot or hb_boot is None:
                return False, 'NO_MODE_POLICY'
            if self.mono() - c['seen_t'] > FRESH_S:
                return False, 'MODE_POLICY_STALE'
            sel = MODE_CHANNEL.get(c['name'])
            if sel is None:
                return False, 'MODE_' + c['name']
            if not c['goal_ok']:
                return False, 'NO_OWNED_GOAL' if c['name'] != 'MANUAL' else 'GOAL_ACTIVE_IN_MANUAL'
            if channel != sel:
                return False, 'CHANNEL_NOT_SELECTED_IN_' + c['name']
            return True, None

    def status(self, hb_boot=None):
        with self.lock:
            c = self.cur
            return {'enabled': True, 'mode': None if c is None else c['name'], 'revision': None if c is None else c['revision'],
                    'epoch': None if c is None else c['epoch'], 'age_s': None if c is None else round(self.mono() - c['seen_t'], 3),
                    'selected_channel': None if c is None else MODE_CHANNEL.get(c['name']), 'goal_ok': None if c is None else c['goal_ok'], 'goal_owner': None if c is None else c['owner'], 'permits_nav': self.permits('nav', hb_boot)[0],
                    'permits_manual': self.permits('manual', hb_boot)[0], 'last_reject': self.last_reject, **self.counters}
