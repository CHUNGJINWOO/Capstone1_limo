"""SQLite persistence (WAL). Robots, missions, commands and events survive a server restart."""
import json, sqlite3, threading, time

SCHEMA = """
CREATE TABLE IF NOT EXISTS robots(robot_id TEXT PRIMARY KEY, first_seen REAL, last_seen REAL, agent_json TEXT, state_json TEXT, state_ts REAL,
  map_version INTEGER, map_meta_json TEXT, map_png BLOB, online INTEGER DEFAULT 0);
CREATE TABLE IF NOT EXISTS missions(mission_id TEXT PRIMARY KEY, robot_id TEXT NOT NULL, type TEXT NOT NULL, goal_json TEXT NOT NULL,
  idempotency_key TEXT, status TEXT NOT NULL, detail TEXT, progress_json TEXT, created_by TEXT, note TEXT, created_at REAL, updated_at REAL, cancel_requested INTEGER DEFAULT 0);
CREATE UNIQUE INDEX IF NOT EXISTS ux_mission_idem ON missions(robot_id, idempotency_key) WHERE idempotency_key IS NOT NULL;
CREATE TABLE IF NOT EXISTS commands(command_id TEXT PRIMARY KEY, robot_id TEXT NOT NULL, mission_id TEXT, type TEXT NOT NULL, payload_json TEXT NOT NULL,
  status TEXT NOT NULL, detail TEXT, seq INTEGER, attempts INTEGER DEFAULT 0, created_at REAL, sent_at REAL, acked_at REAL, expires_at REAL);
CREATE TABLE IF NOT EXISTS events(id INTEGER PRIMARY KEY AUTOINCREMENT, ts REAL, robot_id TEXT, mission_id TEXT, level TEXT, kind TEXT, message TEXT, data_json TEXT);
CREATE TABLE IF NOT EXISTS meta(k TEXT PRIMARY KEY, v TEXT);
CREATE TABLE IF NOT EXISTS locks(robot_id TEXT PRIMARY KEY, owner TEXT NOT NULL, epoch INTEGER NOT NULL, acquired_at REAL, expires_at REAL, ttl_s REAL, note TEXT);
"""


class DB:
    def __init__(self, path):
        self.lock = threading.RLock()
        self.c = sqlite3.connect(path, check_same_thread=False, isolation_level=None)
        self.c.row_factory = sqlite3.Row
        self.c.execute('PRAGMA journal_mode=WAL'); self.c.execute('PRAGMA synchronous=NORMAL')
        self.c.executescript(SCHEMA)
        # Additive migration; existing mission/command IDs and data are preserved.
        if 'owner_boot_id' not in {r[1] for r in self.c.execute('PRAGMA table_info(commands)')}:
            self.c.execute('ALTER TABLE commands ADD COLUMN owner_boot_id TEXT')

    def q(self, sql, args=()):
        with self.lock: return [dict(r) for r in self.c.execute(sql, args).fetchall()]

    def one(self, sql, args=()):
        r = self.q(sql, args); return r[0] if r else None

    def x(self, sql, args=()):
        with self.lock: return self.c.execute(sql, args)

    def next_seq(self, key):
        with self.lock:
            r = self.one('SELECT v FROM meta WHERE k=?', (key,)); n = int(r['v']) + 1 if r else 1
            self.x('INSERT INTO meta(k,v) VALUES(?,?) ON CONFLICT(k) DO UPDATE SET v=excluded.v', (key, str(n))); return n

    def event(self, kind, message, robot_id=None, mission_id=None, level='info', data=None):
        self.x('INSERT INTO events(ts,robot_id,mission_id,level,kind,message,data_json) VALUES(?,?,?,?,?,?,?)',
               (time.time(), robot_id, mission_id, level, kind, message, json.dumps(data) if data is not None else None))
