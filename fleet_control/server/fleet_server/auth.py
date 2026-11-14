"""Bearer-token authentication with roles. Tokens are random, stored only as SHA-256 hashes in data/auth.json (0600);
the plain tokens are written once to secrets/<name>.token (0600) by `--init`.
Roles: viewer (read), operator (read + create/cancel missions), admin (everything), agent (one robot only)."""
import hashlib, hmac, json, os, secrets, threading, time
from collections import defaultdict, deque


class Principal:
    def __init__(self, name, role, robot_id=None):
        self.name, self.role, self.robot_id = name, role, robot_id

    def can_read(self): return self.role in ('viewer', 'operator', 'admin')
    def can_control(self): return self.role in ('operator', 'admin')
    def can_act_as_agent(self, robot_id): return self.role == 'agent' and self.robot_id == robot_id


def _h(token: str) -> str: return hashlib.sha256(token.encode()).hexdigest()


def init_tokens(data_dir: str, secrets_dir: str, robot_ids, force=False):
    os.makedirs(data_dir, exist_ok=True); os.makedirs(secrets_dir, exist_ok=True)
    path = os.path.join(data_dir, 'auth.json')
    if os.path.exists(path) and not force:
        raise SystemExit(f'{path} already exists (use --force to rotate all tokens)')
    entries, created = [], []
    for name, role, rid in [('admin', 'admin', None), ('operator', 'operator', None), ('viewer', 'viewer', None)] + [(f'agent-{r}', 'agent', r) for r in robot_ids]:
        tok = secrets.token_urlsafe(32)
        entries.append({'name': name, 'role': role, 'robot_id': rid, 'sha256': _h(tok)})
        sp = os.path.join(secrets_dir, f'{name}.token')
        fd = os.open(sp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(fd, 'w') as f: f.write(tok + '\n')
        created.append(sp)
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, 'w') as f: json.dump({'tokens': entries}, f, indent=1)
    return created


class Auth:
    def __init__(self, data_dir: str):
        self.path = os.path.join(data_dir, 'auth.json'); self._signature = None; self.entries = []; self._lock = threading.RLock()
        self._reload()
        self.fail = defaultdict(deque)                      # ip -> timestamps of recent failures

    def _reload(self):
        def signature(s): return (s.st_dev, s.st_ino, s.st_size, s.st_mtime_ns, s.st_ctime_ns)
        with self._lock:
            # Atomic replacements can share the same mtime. The inode/nanosecond signature
            # prevents a revoked token from staying valid until another edit occurs.
            if signature(os.stat(self.path)) != self._signature:
                with open(self.path) as f:
                    entries = json.load(f)['tokens']; loaded = signature(os.fstat(f.fileno()))
                self.entries = entries; self._signature = loaded

    def locked_out(self, ip: str, limit=10, window=60.0) -> bool:
        q = self.fail[ip]; now = time.time()
        while q and now - q[0] > window: q.popleft()
        return len(q) >= limit

    def authenticate(self, header: str, ip: str = ''):
        if not header or not header.startswith('Bearer '): return None
        self._reload()
        h = _h(header[7:].strip()); found = None
        for e in self.entries:                               # compare against every entry (constant-time per comparison)
            if hmac.compare_digest(e['sha256'], h): found = e
        if found is None:
            self.fail[ip].append(time.time()); return None
        return Principal(found['name'], found['role'], found.get('robot_id'))
