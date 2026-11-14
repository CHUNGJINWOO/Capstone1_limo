"""usage:
  python3 -m fleet_server init  --data-dir DIR --secrets-dir DIR --robots pcu mock1      (generate tokens, once)
  python3 -m fleet_server serve --data-dir DIR --port 8080 [--bind ADDR ...] [--tailscale]  (default bind: 127.0.0.1)"""
import argparse, os, signal, subprocess, sys
from dataclasses import dataclass
from . import __version__
from .auth import Auth, init_tokens
from .db import DB
from .http_api import Servers, check_bind_address
from .service import Fleet


@dataclass
class Cfg:
    offline_after_s: float = 6.0
    command_ttl_s: float = 120.0
    resend_after_s: float = 5.0


def main():
    ap = argparse.ArgumentParser(prog='fleet_server'); sub = ap.add_subparsers(dest='cmd', required=True)
    i = sub.add_parser('init'); i.add_argument('--data-dir', required=True); i.add_argument('--secrets-dir', required=True); i.add_argument('--robots', nargs='+', required=True); i.add_argument('--force', action='store_true')
    s = sub.add_parser('serve'); s.add_argument('--data-dir', required=True); s.add_argument('--port', type=int, default=8080); s.add_argument('--bind', action='append', default=[])
    s.add_argument('--tailscale', action='store_true', help='also bind this machine\'s Tailscale IPv4 address (read from `tailscale ip -4`)')
    s.add_argument('--offline-after', type=float, default=6.0)
    s.add_argument('--video-origin', action='append', default=[], help='origin of the separate video/AI host the dashboard may connect to (e.g. http://127.0.0.1:18172); added to the CSP connect-src. Default: none')
    a = ap.parse_args()
    if a.cmd == 'init':
        for p in init_tokens(a.data_dir, a.secrets_dir, a.robots, a.force): print('token written:', p)
        print('hashes in', os.path.join(a.data_dir, 'auth.json')); return
    addrs = list(dict.fromkeys(a.bind or ['127.0.0.1']))
    if a.tailscale:
        ip = subprocess.run(['tailscale', 'ip', '-4'], capture_output=True, text=True).stdout.split()
        if not ip: sys.exit('tailscale ip -4 returned nothing; not binding Tailscale')
        addrs.append(ip[0])
    for x in addrs: check_bind_address(x)                                   # validate BEFORE touching the database (a refused start must not log 'server started')
    db = DB(os.path.join(a.data_dir, 'fleet.db')); fleet = Fleet(db, Cfg(offline_after_s=a.offline_after)); auth = Auth(a.data_dir)
    import re
    for o in a.video_origin:
        if not re.fullmatch(r'https?://[A-Za-z0-9.\-]+(:[0-9]{1,5})?', o): sys.exit(f'--video-origin must look like http://host:port, got {o!r}')
    srv = Servers(fleet, auth, addrs, a.port, tuple(a.video_origin))
    signal.signal(signal.SIGTERM, lambda *_: srv.stop_evt.set()); signal.signal(signal.SIGINT, lambda *_: srv.stop_evt.set())
    print(f'fleet_server {__version__} boot={fleet.boot_id} listening on ' + ', '.join(f'http://{x}:{a.port}/' for x in addrs), flush=True)
    srv.serve_forever(); print('stopped', flush=True)


if __name__ == '__main__':
    main()
