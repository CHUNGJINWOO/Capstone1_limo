"""Agent side of the Watchdog speed clamp (G2, 2026-11-23). Used when the Agent has no Arbiter (real robot).

Every 0.2 s: feeds the delivered server request to the limiter, tells the Watchdog the limits the Agent currently stands behind
(/fleet/speed_limit), and reports `enforced_by = 'WATCHDOG'` ONLY while the Watchdog says (fresh status, same Agent boot) that its clamp is
enabled, synchronised with this Agent, fault-free, and listening to exactly this publisher. A request counts as applied only after the
Watchdog's own status shows the same generation, request id and values (SpeedLimiter.confirm_fn) -- never because the Agent sent it.
Detection of the Watchdog is by its status data, not by the DDS graph; this is not authentication of the Watchdog."""
import json
import time

from . import topics

STATUS_MAX_AGE_S = 1.5


class WatchdogSpeedLink:
    def __init__(self, robot, agent, limiter, period=0.2):
        from std_msgs.msg import String
        self._String = String
        self.robot, self.agent, self.limiter = robot, agent, limiter
        self.seq = 0
        self.withdrawn = None        # (Agent boot, mode revision) whose authority was withdrawn (connection or contract lost): never stated again
        self.pub = robot.create_publisher(String, topics.SPEED_LIMIT, 10)
        limiter.confirm_fn = self.confirms
        self.timer = robot.create_timer(period, self.tick)

    def _status(self):
        w = getattr(self.robot, 'motion_watchdog', None)
        if not w or time.monotonic() - w[0] > STATUS_MAX_AGE_S or not isinstance(w[1], dict):
            return None
        return w[1]

    def capable(self):
        st = self._status()
        sc = (st or {}).get('speed_clamp')
        return bool(st and isinstance(sc, dict) and sc.get('enabled') is True and sc.get('synced') is True and sc.get('publisher_ok') is True
                    and sc.get('fault') is None and st.get('agent_boot_id') == self.agent.session_snapshot().get('agent_boot_id'))

    def evidence(self):
        """What the Watchdog itself reported, copied as data (never a bare boolean): its boot id, its output writer GID (a DDS graph
        identifier, not an authentication), the generations/values it applies and how old that status is. The server re-checks it."""
        w = getattr(self.robot, 'motion_watchdog', None)
        if not w or not isinstance(w[1], dict):
            return None
        st, sc = w[1], w[1].get('speed_clamp')
        if not isinstance(sc, dict):
            return None
        applied = {}
        for scope, a in (sc.get('applied') or {}).items():
            if isinstance(a, dict):
                applied[scope] = {k: a.get(k) for k in ('generation', 'request_id', 'linear_mps', 'angular_rps', 'stale')}
        return {'watchdog_boot_id': st.get('boot_id'), 'agent_boot_id': st.get('agent_boot_id'), 'output_gid': st.get('output_gid'),
                'synced': sc.get('synced'), 'publisher_ok': sc.get('publisher_ok'), 'fault': sc.get('fault'), 'enabled': sc.get('enabled'),
                'evidence_age_s': round(max(0.0, time.monotonic() - w[0]), 3), 'applied': applied, 'agent_verified': bool(self.capable())}

    def mode_for_watchdog(self, session):
        """The operating mode the server selected, as the Agent may state it to the Watchdog: only while the Agent itself would execute (connected,
        reconciled, valid Lease and contract); otherwise IDLE, so the Watchdog forwards nothing."""
        om = (session or {}).get('operating_mode')
        try:
            ok = (isinstance(om, dict) and type(om.get('revision')) is int and isinstance(om.get('mode'), str) and self.agent.is_execution_allowed()
                  and session.get('connected') is True and session.get('reconciliation_required') is False)
        except Exception:
            ok = False
        rev = om.get('revision') if isinstance(om, dict) and type(om.get('revision')) is int else None
        key = (session.get('agent_boot_id') if isinstance(session, dict) else None, rev)
        if not ok and rev is not None:
            self.withdrawn = key                    # authority lost for this mode revision: a later reconnect does NOT bring the motion back
        elif ok and self.withdrawn == key:
            ok = False                              # only a NEW mode revision (an explicit operator request, published after the idle check) is stated again
        return {'name': om['mode'] if ok else 'IDLE', 'revision': rev if rev is not None else 0, 'epoch': om.get('epoch') if isinstance(om, dict) else None}

    def goal_for_watchdog(self):
        """The Agent's own record of the Nav2 goal it owns (the Arbiter's owned-goal rule, now enforced where the motion is forwarded): who owns it and whether
        a cancel has been requested. Without an owned goal, or after a cancel request, the Watchdog forwards nothing from the nav channel."""
        a = getattr(self.robot, 'active', None)
        if not isinstance(a, dict):
            return {'owner': None, 'cancel_requested': False}
        return {'owner': a.get('owner', 'fleet'), 'cancel_requested': bool(a.get('cancel_requested'))}

    def confirms(self, c):
        st = self._status(); sc = (st or {}).get('speed_clamp') or {}
        a = (sc.get('applied') or {}).get(c['scope'])
        return bool(self.capable() and isinstance(a, dict) and a.get('generation') == c['generation'] and a.get('request_id') == c['request_id']
                    and a.get('linear_mps') == c['linear_mps'] and a.get('angular_rps') == c['angular_rps'] and a.get('stale') is False)

    def tick(self):
        try:
            session = self.agent.session_snapshot()
            self.limiter.enforced_by = 'WATCHDOG' if self.capable() else None
            self.limiter.watchdog_evidence = self.evidence()
            self.limiter.process(session)
            lease = (session or {}).get('lease') or {}
            self.seq += 1
            self.pub.publish(self._String(data=json.dumps({
                'agent_boot_id': session.get('agent_boot_id'), 'seq': self.seq, 't': time.time(),
                'binding': [session.get('server_boot_id'), lease.get('epoch'), session.get('contract_generation')],
                'mode': self.mode_for_watchdog(session), 'goal': self.goal_for_watchdog(), 'scopes': self.limiter.watchdog_scopes(session)})))
        except Exception:
            pass                       # no message: the Watchdog loses sync and goes to FAILSAFE (zero); nothing is raised here

    def close(self):
        self.timer.cancel()
        try:
            self.robot.destroy_publisher(self.pub)          # a second publisher would make the Watchdog refuse the topic (exactly one allowed)
        except Exception:
            pass
