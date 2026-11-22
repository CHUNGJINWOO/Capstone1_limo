#!/usr/bin/env python3
"""Token hot-reload regression using temporary credentials only; never prints token values."""
import hashlib,json,os,sys,tempfile,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'server'))
from fleet_server.auth import Auth,init_tokens
class Tests(unittest.TestCase):
    def test_rotation_revokes_old_token_without_auth_restart(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);init_tokens(d,str(p/'secrets'),['mock'])
            with (p/'secrets/viewer.token').open() as f:old=f.read().strip()
            auth=Auth(d);self.assertEqual(auth.authenticate('Bearer '+old).role,'viewer')
            with (p/'auth.json').open() as f:j=json.load(f)
            j['tokens']=[e for e in j['tokens'] if e['role']!='viewer']
            replacement='fixture-only';j['tokens'].append({'name':'replacement','role':'viewer','sha256':hashlib.sha256(replacement.encode()).hexdigest()})
            previous=(p/'auth.json').stat()
            tmp=p/'next.json';tmp.write_text(json.dumps(j))
            os.utime(tmp,ns=(previous.st_atime_ns,previous.st_mtime_ns))  # Reproduce same-mtime atomic replacement deterministically.
            os.replace(tmp,p/'auth.json')
            self.assertIsNone(auth.authenticate('Bearer '+old));self.assertEqual(auth.authenticate('Bearer '+replacement).role,'viewer')
    def test_role_change_is_loaded_without_restart(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);init_tokens(d,str(p/'secrets'),['mock']);token=(p/'secrets/operator.token').read_text().strip();auth=Auth(d)
            self.assertTrue(auth.authenticate('Bearer '+token).can_control())
            j=json.loads((p/'auth.json').read_text())
            for e in j['tokens']:
                if e['name']=='operator':e['role']='viewer'
            t=p/'next.json';t.write_text(json.dumps(j));os.replace(t,p/'auth.json')
            self.assertFalse(auth.authenticate('Bearer '+token).can_control())
if __name__=='__main__':unittest.main(verbosity=2)
