#!/usr/bin/env python3
"""Summarize RTPS packets a traced participant RECEIVED. usage: dds_strace_rtps.py STRACE_FILE [OWN_PREFIX_HEX24]
Input: output of `strace -f -xx -s 400 -e trace=recvfrom` of one of our own probes (ptrace of our own child, no root). Counts submessage kinds,
source GUID prefixes and INFO_DST destination prefixes among received packets (first 400 bytes of each packet are visible)."""
import collections, re, struct, sys
KIND = {0x06: 'ACKNACK', 0x07: 'HEARTBEAT', 0x08: 'GAP', 0x09: 'INFO_TS', 0x0e: 'INFO_DST', 0x15: 'DATA', 0x16: 'DATA_FRAG', 0x13: 'HB_FRAG'}
pat = re.compile(r'"(\\x52\\x54\\x50\\x53(?:\\x[0-9a-f]{2})*)"[^\n]*\) = (\d+)\s*$')
kinds, src, dst, total = collections.Counter(), collections.Counter(), collections.Counter(), 0
data_kind, hb_kind, ack_kind = collections.Counter(), collections.Counter(), collections.Counter()
builtin = lambda eid: eid[3] in (0xc1, 0xc2, 0xc3, 0xc4, 0xc7) # builtin entity kinds; user: 0x01/0x02/0x03/0x04/0x07
for line in open(sys.argv[1], errors='replace'):
    m = pat.search(line)
    if not m or int(m.group(2)) <= 0: continue
    b = bytes(int(x, 16) for x in re.findall(r'\\x([0-9a-f]{2})', m.group(1)))
    if b[:4] != b'RTPS' or len(b) < 24: continue
    total += 1; src[b[8:20].hex()] += 1; i = 20
    while i + 4 <= len(b):
        sid, fl = b[i], b[i + 1]; ln = struct.unpack('<H' if fl & 1 else '>H', b[i + 2:i + 4])[0]
        kinds[KIND.get(sid, hex(sid))] += 1
        if sid == 0x15 and i + 16 <= len(b): wid = b[i + 12:i + 16]; data_kind['builtin' if builtin(wid) else 'USER'] += 1
        if sid == 0x07 and i + 12 <= len(b): wid = b[i + 8:i + 12]; hb_kind['builtin' if builtin(wid) else 'USER'] += 1
        if sid == 0x06 and i + 12 <= len(b): wid = b[i + 8:i + 12]; ack_kind['builtin' if builtin(wid) else 'USER'] += 1
        if sid == 0x0e and i + 16 <= len(b): dst[b[i + 4:i + 16].hex()] += 1
        i += 4 + (ln if ln else len(b) - i - 4)
own = sys.argv[2] if len(sys.argv) > 2 else None
print(f'received RTPS packets: {total}; submessages: {dict(kinds)}')
print('DATA by writer kind:', dict(data_kind), '| HEARTBEAT:', dict(hb_kind), '| ACKNACK (writerId):', dict(ack_kind))
print('source prefixes:', len(src), '| INFO_DST prefixes:', {k: v for k, v in dst.most_common(8)})
if own: print('INFO_DST == own prefix:', dst.get(own, 0), '| INFO_DST != own:', sum(v for k, v in dst.items() if k != own))
