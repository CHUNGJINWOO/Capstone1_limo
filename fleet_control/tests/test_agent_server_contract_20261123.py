"""R1.1 Agent<->Server contract test (2026-11-22, Claude). Server-only, no ROS, no sockets, no hardware, no source edit.
Pins: protocol_version and safety_contract_version agree on both sides; a version mismatch or a missing contract field refuses movement
authorisation (safety_context None, reason AGENT_PROTOCOL, mission create / gate-reset blocked); every /agent/* path the Agent uses is served
and (except a recorded documentation gap) documented. Reuses the safety-context fixture; the literal checks are text/AST based so that R1.2
(naming the constants) keeps them green."""
import ast, re, sys, unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'agent' / 'limo_robot_agent'))
import test_safety_context_v1 as fixture
from fleet_server.service import ApiError
from fleet_server.auth import Principal

SRV = ROOT / 'server' / 'fleet_server'; AGT = ROOT / 'agent' / 'limo_robot_agent' / 'limo_robot_agent'
EXPECTED_PROTOCOL, EXPECTED_SAFETY = 2, 1
# Agent paths served by the Server but absent from docs/API.md (closed by R1.3 on 2026-11-22).
DOC_GAP = set()

def src(p): return Path(p).read_text()
def agent_paths():
    out = set()
    for f in AGT.glob('*.py'): out |= set(re.findall(r"""['"](/agent/[a-z][a-z-]*)""", src(f)))
    return out

class VersionAgreement(unittest.TestCase):
    def test_agent_constants(self):
        from limo_robot_agent import safety_protocol
        self.assertEqual(safety_protocol.SAFETY_CONTRACT_VERSION, EXPECTED_SAFETY)
        if hasattr(safety_protocol, 'PROTOCOL_VERSION'):          # after R1.2 the number has one definition
            self.assertEqual(safety_protocol.PROTOCOL_VERSION, EXPECTED_PROTOCOL); return
        kw = [k.value.value for n in ast.walk(ast.parse(src(AGT / 'core.py'))) if isinstance(n, ast.Call) for k in n.keywords if k.arg == 'protocol_version']
        self.assertEqual(kw, [EXPECTED_PROTOCOL])
    def test_server_hello_advertises_protocol(self):
        t = fixture.Tests('test_old_agent_blocked'); t.setUp()
        try: self.assertEqual(t.f.hello('r', {'boot_id': 'boot1', 'protocol_version': 2, 'safety_contract_version': 1})['protocol_version'], EXPECTED_PROTOCOL)
        finally: t.tearDown()
    def test_server_values_all_agree(self):
        from fleet_server import safety
        if hasattr(safety, 'PROTOCOL_VERSION'):                   # after R1.2: named constants, no bare literal may remain beside them
            self.assertEqual((safety.PROTOCOL_VERSION, safety.SAFETY_CONTRACT_VERSION), (EXPECTED_PROTOCOL, EXPECTED_SAFETY))
            for n in ('service.py', 'safety.py', 'gate_reset.py'):
                self.assertEqual(re.findall(r"protocol_version'(?:\)|\])?\s*(?:==|!=|:)\s*\d", src(SRV / n)), [], n)
            return
        text = {n: src(SRV / n) for n in ('service.py', 'safety.py')}
        for n, s in text.items():
            found = re.findall(r"protocol_version'(?:\)|\])?\s*(?:==|!=|:)\s*(\d+)", s); self.assertGreaterEqual(len(found), 1, n)
            for m in found: self.assertEqual(int(m), EXPECTED_PROTOCOL, n)
        self.assertIn('agent_contract_ok', src(SRV / 'gate_reset.py'))
        self.assertIn("res['safety_contract_version'] = %d" % EXPECTED_SAFETY, text['service.py'])
        self.assertIn("agent['safety_contract_version'] != %d" % EXPECTED_SAFETY, text['safety.py'])

class Refusal(unittest.TestCase):
    def setUp(self): self.t = fixture.Tests('test_old_agent_blocked'); self.t.setUp(); self.f = self.t.f; self.agent = self.f.robots['r']['agent']
    def tearDown(self): self.t.tearDown()
    def reason(self): return (getattr(self.f, 'safety_context_reasons', {}).get('r') or {}).get('reason')
    def test_baseline_is_admitted(self):
        self.agent.update(protocol_version=2, safety_contract_version=1)
        self.assertIsNotNone(self.f.safety_context('r'), self.reason()); self.t.create()
    def test_mismatch_or_malformed_version_refuses(self):
        for field, bad in [('protocol_version', 1), ('protocol_version', 3), ('protocol_version', True), ('protocol_version', '2'), ('protocol_version', 2.0),
                           ('safety_contract_version', 0), ('safety_contract_version', 2), ('safety_contract_version', True), ('safety_contract_version', '1'), ('safety_contract_version', None)]:
            with self.subTest(field=field, value=bad):
                self.agent.update(protocol_version=2, safety_contract_version=1); self.agent[field] = bad
                self.assertIsNone(self.f.safety_context('r')); self.assertEqual(self.reason(), 'AGENT_PROTOCOL')
                with self.assertRaises(ApiError): self.t.create()
    def test_missing_contract_field_refuses(self):
        for field in ('protocol_version', 'safety_contract_version', 'boot_id'):
            with self.subTest(field=field):
                self.agent.update(protocol_version=2, safety_contract_version=1, boot_id='boot1'); self.agent.pop(field)
                self.assertIsNone(self.f.safety_context('r')); self.assertEqual(self.reason(), 'AGENT_PROTOCOL')
    def test_bad_boot_id_refuses(self):
        for bad in ('', '   ', None, 5, 'x' * 257):
            with self.subTest(boot_id=bad):
                self.agent.update(protocol_version=2, safety_contract_version=1, boot_id=bad); self.assertIsNone(self.f.safety_context('r'))
    def test_gate_reset_reports_protocol(self):
        self.agent.update(protocol_version=3, safety_contract_version=1)
        self.assertIn('AGENT_PROTOCOL', self.f.gate_reset_blockers('r', Principal('adm', 'admin')))
        self.agent.update(protocol_version=2)
        self.assertNotIn('AGENT_PROTOCOL', self.f.gate_reset_blockers('r', Principal('adm', 'admin')))

class RoutesAndDocs(unittest.TestCase):
    def test_agent_paths_are_served(self):
        api = src(SRV / 'http_api.py'); used = agent_paths(); self.assertGreaterEqual(len(used), 11)
        missing = {p for p in used if p not in api and not re.search(re.escape(p) + r"(/|\b)", api)}
        self.assertEqual(missing, set())
    def test_agent_paths_documented_except_known_gap(self):
        doc = src(ROOT / 'docs' / 'API.md'); gap = {p for p in agent_paths() if p not in doc}
        self.assertEqual(gap, DOC_GAP, 'documentation gap changed: update docs/API.md and shrink DOC_GAP')
    def test_contract_documents_state_the_numbers(self):
        c = src(ROOT / 'server' / 'SAFETY_CONTRACT.md'); self.assertIn('safety_contract_version: 1', c.replace('`', '')); self.assertRegex(c, r'protocol_version":\s*2')
        self.assertRegex(src(ROOT / 'docs' / 'API.md'), r'"protocol_version":\s*2')

if __name__ == '__main__': unittest.main(verbosity=2)
