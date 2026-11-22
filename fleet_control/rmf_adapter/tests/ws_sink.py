"""Minimal RFC6455 WebSocket receiver (stdlib only) standing in for the RMF API server. The RMF Fleet Adapter library broadcasts its task/fleet state
updates (JSON) to the `server_uri` it is given; this sink records them so the test can read RMF's own view of every task (status completed/canceled/failed ...)."""
import base64, hashlib, json, socket, struct, threading, time

GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'


class WsSink:
    def __init__(self, port, path=None):
        self.port, self.path = port, path; self.msgs = []; self.task = {}; self.lock = threading.Lock()
        self.srv = socket.socket(); self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1); self.srv.bind(('127.0.0.1', port)); self.srv.listen(8)
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while True:
            try: c, _ = self.srv.accept()
            except OSError: return
            threading.Thread(target=self._conn, args=(c,), daemon=True).start()

    def _recvn(self, c, n):
        b = b''
        while len(b) < n:
            d = c.recv(n - len(b))
            if not d: raise EOFError
            b += d
        return b

    def _conn(self, c):
        try:
            req = b''
            while b'\r\n\r\n' not in req: req += c.recv(4096)
            key = [l.split(b':', 1)[1].strip() for l in req.split(b'\r\n') if l.lower().startswith(b'sec-websocket-key')][0]
            acc = base64.b64encode(hashlib.sha1(key + GUID.encode()).digest())
            c.sendall(b'HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: ' + acc + b'\r\n\r\n')
            buf = b''
            while True:
                h = self._recvn(c, 2); op = h[0] & 0x0f; ln = h[1] & 0x7f
                if ln == 126: ln = struct.unpack('>H', self._recvn(c, 2))[0]
                elif ln == 127: ln = struct.unpack('>Q', self._recvn(c, 8))[0]
                mask = self._recvn(c, 4) if h[1] & 0x80 else b'\0\0\0\0'
                data = bytes(b ^ mask[i % 4] for i, b in enumerate(self._recvn(c, ln)))
                if op == 8: return
                if op == 9: c.sendall(b'\x8a' + bytes([len(data)]) + data); continue
                buf += data
                if h[0] & 0x80:
                    self._on(buf); buf = b''
        except Exception: pass
        finally: c.close()

    def _on(self, raw):
        try: m = json.loads(raw)
        except Exception: return
        with self.lock:
            self.msgs.append((time.time(), m))
            if self.path:
                with open(self.path, 'a') as f: f.write(json.dumps({'t': time.time(), 'm': m}) + '\n')
            if m.get('type') == 'task_state_update':
                d = m.get('data') or {}; tid = (d.get('booking') or {}).get('id')
                if tid: self.task[tid] = d

    def status(self, tid):
        with self.lock: return (self.task.get(tid) or {}).get('status')

    def close(self):
        try: self.srv.close()
        except OSError: pass
