"""TEST ONLY: run the unmodified agent_node with a map/localization FIXTURE for isolated NAVIGATION-mode checks.

No real map-verification or localization producer exists yet (Phase 3). This wrapper injects
{map_verified, localization CONFIRMED} on a fixture map id so the existing Safety Context v1 can admit missions
in the isolated mock graph. It is NOT evidence of localization. The operation mode is NOT faked: it comes from the
Fleet server's operating mode (RosRobot._server_operation_context). Refuses to run outside an isolated domain.
"""
import json, os, sys

assert os.environ.get('ROS_LOCALHOST_ONLY') == '1' and os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0', '77')
assert not os.environ.get('ROS_DISCOVERY_SERVER')
from limo_robot_agent import ros_adapter, agent_node

FIXTURE = {'map': {'map_verified': True, 'map_version_id': 'e2e-fixture-map', 'map_epoch': 1},
           'localization': {'localization_status': 'CONFIRMED', 'localization_map_version_id': 'e2e-fixture-map',
                            'localization_epoch': 1}}
_init = ros_adapter.RosRobot.__init__


def _patched(self, *a, **k):
    _init(self, *a, **k)
    self.context_fn = lambda: json.loads(json.dumps(FIXTURE))     # map/localization FIXTURE only


ros_adapter.RosRobot.__init__ = _patched
sys.exit(agent_node.main())
