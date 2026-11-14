"""Minimal client of the Mission Manager HTTP API v1 (stdlib only). Authentication: Bearer token read from a file (never logged)."""
import json, os, urllib.error, urllib.request

TERMINAL = {'SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED', 'EXPIRED'}


class MissionApiError(Exception):
    def __init__(self, status, code, message): super().__init__(f'{status} {code}: {message}'); self.status, self.code = status, code


class MissionApiUnavailable(Exception):
    """Server not reachable / timed out / unparsable: the result of the request is UNKNOWN."""


class MissionClient:
    def __init__(self, base_url, token_file, timeout=4.0):
        self.base = base_url.rstrip('/'); self.token_file = token_file; self.timeout = timeout
        if (os.stat(token_file).st_mode & 0o077): raise PermissionError(f'{token_file} must not be accessible by group/others')

    def _token(self):
        with open(self.token_file) as f: return f.read().strip()

    def _req(self, method, path, body=None):
        req = urllib.request.Request(self.base + '/api/v1' + path, method=method, data=json.dumps(body).encode() if body is not None else None,
                                     headers={'Content-Type': 'application/json', 'Authorization': 'Bearer ' + self._token()})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r: raw = r.read(); return r.status, (json.loads(raw) if raw else None)
        except urllib.error.HTTPError as e:
            try: err = json.loads(e.read() or b'{}').get('error', {})
            except Exception: err = {}
            raise MissionApiError(e.code, err.get('code', 'http_error'), err.get('message', '')) from None
        except (urllib.error.URLError, ConnectionError, TimeoutError, OSError, ValueError) as e:
            raise MissionApiUnavailable(type(e).__name__) from None

    def create_mission(self, robot_id, goal, idempotency_key, note=''):
        st, d = self._req('POST', '/missions', {'robot_id': robot_id, 'type': 'navigate_to_pose', 'goal': goal, 'idempotency_key': idempotency_key, 'note': note})
        return d, bool(d.get('created', st == 201))

    def get_mission(self, mission_id): return self._req('GET', f'/missions/{mission_id}')[1]
    def cancel_mission(self, mission_id): return self._req('POST', f'/missions/{mission_id}/cancel', {})[1]
    def lease(self, robot_id, action, ttl_s=None, note=''):
        body = {'action': action, 'note': note}
        if ttl_s is not None: body['ttl_s'] = ttl_s
        return self._req('POST', f'/robots/{robot_id}/lock', body)[1]

    def get_me(self): return self._req('GET', '/me')[1]
    def get_robot(self, robot_id): return self._req('GET', f'/robots/{robot_id}')[1]

    def list_missions(self, robot_id, limit=100):
        return self._req('GET', f'/missions?robot_id={robot_id}&limit={limit}')[1]['missions']
