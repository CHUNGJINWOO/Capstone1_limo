#!/usr/bin/env python3
"""Passive raw UDP listener (NOT a DDS participant): binds the two RTPS unicast ports of a participant ID on 127.0.0.1 and decodes whatever RTPS
packets other participants still send there. usage: dds_raw_listen.py DOMAIN ID [SECONDS]. Sends nothing. Reports source GUID prefixes, submessage kinds and
INFO_DST destination prefixes, so it shows whether live peers keep addressing a participant that no longer exists at this ID."""
import collections, select, socket, struct, sys, time
dom, pid = int(sys.argv[1]), int(sys.argv[2]); secs = float(sys.argv[3]) if len(sys.argv) > 3 else 15
meta, user = 7400 + 250 * dom + 10 + 2 * pid, 7400 + 250 * dom + 11 + 2 * pid
KIND = {0x01: 'PAD', 0x06: 'ACKNACK', 0x07: 'HEARTBEAT', 0x08: 'GAP', 0x09: 'INFO_TS', 0x0c: 'INFO_SRC', 0x0d: 'INFO_REPLY_IP4', 0x0e: 'INFO_DST', 0x0f: 'INFO_REPLY', 0x12: 'NACK_FRAG', 0x13: 'HEARTBEAT_FRAG', 0x15: 'DATA', 0x16: 'DATA_FRAG'}
socks = {}
for name, port in (('meta', meta), ('user', user)):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('127.0.0.1', port)); socks[s] = (name, port)
src = collections.Counter(); kinds = collections.Counter(); dst = collections.Counter(); n = collections.Counter(); nonrtps = 0
end = time.time() + secs
while time.time() < end:
    r, _, _ = select.select(list(socks), [], [], 0.5)
    for s in r:
        data, addr = s.recvfrom(65536); nm = socks[s][0]; n[nm] += 1
        if data[:4] != b'RTPS' or len(data) < 20: nonrtps += 1; continue
        src[(nm, data[8:20].hex())] += 1
        i = 20
        while i + 4 <= len(data):
            sid, fl, ln = data[i], data[i + 1], struct.unpack('<H' if fl & 1 else '>H', data[i + 2:i + 4])[0] if True else 0
            fl = data[i + 1]; ln = struct.unpack('<H' if fl & 1 else '>H', data[i + 2:i + 4])[0]
            kinds[(nm, KIND.get(sid, hex(sid)))] += 1
            if sid == 0x0e and ln >= 12: dst[(nm, data[i + 4:i + 16].hex())] += 1
            i += 4 + (ln if ln else len(data) - i - 4)
print(f'domain {dom} ID {pid}: meta {meta} user {user}; packets {dict(n)} non-RTPS {nonrtps} in {secs}s')
for k, v in sorted(kinds.items()): print('  submessage', k, v)
for k, v in src.most_common(12): print('  src prefix', k, v)
for k, v in dst.most_common(12): print('  INFO_DST prefix', k, v)
