#!/usr/bin/env python3
"""Restart policy with the independent motion watchdog (no ROS, no motors): real HTTP + SQLite + AgentCore.

A watchdog trip or restart changes the reported watchdog epoch, which is part of the Safety Context: every mission
admitted before it is invalidated (queued -> FAILED, in flight -> RECONCILIATION_REQUIRED) and nothing is dispatched
again automatically after the watchdog re-arms. A new mission needs a new operator request.
"""
import time
import unittest

import test_recovery as tr
from fleet_server.service import ApiError


class WdRobot(tr.Robot):
    def __init__(self, backend=None):
        super().__init__(backend)
        self.wd = {'required': True, 'state': 'MOTION_AUTHORIZED', 'age_s': 0.0, 'epoch': 'wdboot1:0',
                   'last_fault': None, 'agent_boot_matches': True}

    def snapshot(self):
        s = super().snapshot()
        s['motion_watchdog'] = dict(self.wd)
        return s


class WatchdogAdmissionTests(unittest.TestCase):
    setUp = tr.RecoveryTests.setUp
    tearDown = tr.RecoveryTests.tearDown
    start_server = tr.RecoveryTests.start_server
    stop_server = tr.RecoveryTests.stop_server
    agent = tr.RecoveryTests.agent
    stop_agent = tr.RecoveryTests.stop_agent
    mission = tr.RecoveryTests.mission
    status = tr.RecoveryTests.status

    def _lease(self):
        from fleet_server.auth import Principal
        return self.fleet.lease_view('r') or self.fleet.lease_action('r', Principal('op', 'operator'), 'acquire', 300)

    def test_context_carries_watchdog_epoch_only_when_required(self):
        a, r = self.agent(WdRobot())
        tr.wait(lambda: self.fleet.safety_context('r') is not None)
        self.assertEqual(self.fleet.safety_context('r')['motion_watchdog_epoch'], 'wdboot1:0')
        r.wd['required'] = False
        tr.wait(lambda: (self.fleet.safety_context('r') or {}).get('motion_watchdog_epoch', 'absent') == 'absent')

    def test_required_watchdog_not_authorized_or_stale_refuses_admission(self):
        a, r = self.agent(WdRobot())
        for bad in ({'state': 'LOCKED'}, {'state': 'FAILSAFE'}, {'age_s': 5.0}, {'epoch': None}, {'agent_boot_matches': False}):
            r.wd.update(bad)
            tr.wait(lambda: self.fleet.safety_context('r') is None)
            with self.assertRaises(ApiError) as refused:
                self.fleet.create_mission('r', 'navigate_to_pose', tr.GOAL, 'k' + str(time.time_ns()), created_by='op',
                                          lease_epoch=self._lease()['epoch'])
            self.assertEqual(refused.exception.status, 409)
            r.wd.update({'state': 'MOTION_AUTHORIZED', 'age_s': 0.0, 'epoch': 'wdboot1:0', 'agent_boot_matches': True})
            tr.wait(lambda: self.fleet.safety_context('r') is not None)
        self.assertEqual(r.backend['starts'], 0)

    def test_trip_invalidates_queue_and_nothing_restarts_after_rearm(self):
        a, r = self.agent(WdRobot())
        first = self.mission(); tr.wait(lambda: self.status(first) == 'EXECUTING')
        queued = self.mission(); self.assertEqual(self.status(queued), 'QUEUED')
        # Watchdog trips (FAILSAFE) and cancels the Nav2 goal; then it re-arms with a new epoch.
        r.wd.update({'state': 'FAILSAFE', 'last_fault': 'AGENT_HEARTBEAT_LOST'})
        tr.wait(lambda: self.status(queued) == 'FAILED')
        r.finish('CANCELED')
        r.wd.update({'state': 'MOTION_AUTHORIZED', 'epoch': 'wdboot1:1'})
        tr.wait(lambda: self.fleet.safety_context('r') is not None or self.status(first) == 'RECONCILIATION_REQUIRED')
        time.sleep(0.5)
        self.assertEqual(self.status(queued), 'FAILED')
        self.assertIn(self.status(first), ('RECONCILIATION_REQUIRED', 'CANCELED'))
        self.assertEqual(r.backend['starts'], 1)                     # no automatic re-departure
        self.assertEqual(self.fleet._deliver('r')['commands'], [])

    def test_watchdog_restart_changes_epoch_and_invalidates_queue(self):
        a, r = self.agent(WdRobot())
        first = self.mission(); tr.wait(lambda: self.status(first) == 'EXECUTING')
        queued = self.mission()
        r.wd.update({'epoch': 'wdboot2:0'})                          # watchdog process restarted
        tr.wait(lambda: self.status(queued) == 'FAILED')
        self.assertEqual(r.backend['starts'], 1)


if __name__ == '__main__':
    unittest.main()
