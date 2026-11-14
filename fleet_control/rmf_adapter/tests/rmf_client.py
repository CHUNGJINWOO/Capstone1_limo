"""Tiny rclpy helper that plays the role of an RMF operator: dispatches/cancels tasks through the standard RMF task API (task_api_requests) and observes
dispatch states, fleet states and the task state updates published by the fleet adapter."""
import json, threading, time, uuid
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from rmf_task_msgs.msg import ApiRequest, ApiResponse, DispatchStates, TaskSummary
from rmf_fleet_msgs.msg import FleetState


class RmfClient(Node):
    def __init__(self):
        super().__init__('rmf_test_client')
        q = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(ApiRequest, 'task_api_requests', q)
        self.responses = {}; self.dispatch = None; self.fleet = None; self.fleet_t = 0.0; self.task_updates = []
        self.create_subscription(ApiResponse, 'task_api_responses', self._resp, q)
        self.create_subscription(DispatchStates, 'dispatch_states', lambda m: setattr(self, 'dispatch', m), 10)
        self.summaries = {}
        self.create_subscription(TaskSummary, 'task_summaries', self._sum, 10)
        self.create_subscription(FleetState, 'fleet_states', self._fleet, 10)
        self._th = threading.Thread(target=rclpy.spin, args=(self,), daemon=True); self._th.start()

    def _resp(self, m): self.responses[m.request_id] = json.loads(m.json_msg)
    def _sum(self, m): self.summaries[m.task_id] = (m.state, m.status, time.time())
    def task_state(self, tid): return (self.summaries.get(tid) or (None,))[0]
    def _fleet(self, m): self.fleet = m; self.fleet_t = time.time()

    def request(self, payload, timeout=10):
        rid = uuid.uuid4().hex; m = ApiRequest(); m.request_id = rid; m.json_msg = json.dumps(payload); self.pub.publish(m)
        end = time.time() + timeout
        while time.time() < end:
            if rid in self.responses: return self.responses[rid]
            time.sleep(0.1)
        return None

    def dispatch_patrol(self, places, rounds=1):
        return self.request({'type': 'dispatch_task_request', 'request': {'unix_millis_earliest_start_time': 0, 'category': 'patrol', 'description': {'places': places, 'rounds': rounds}}})

    def cancel(self, task_id): return self.request({'type': 'cancel_task_request', 'task_id': task_id})

    def dispatch_state(self, task_id):
        if not self.dispatch: return None
        for d in list(self.dispatch.active) + list(self.dispatch.finished):
            if d.task_id == task_id: return d
        return None

    def robot(self, name='pcu'):
        if not self.fleet or time.time() - self.fleet_t > 5: return None
        for r in self.fleet.robots:
            if r.name == name: return r
        return None
