import json,sys,re,collections
D,mode=sys.argv[1],sys.argv[2]; r=json.load(open(f'{D}/result_{mode}.json'))
print(f"[{mode}] scans sent {r['scans_sent']} (skipped {r['scans_skipped']}), tf sent {r['tf_sent']}, recorded scan lengths {r['recorded_scan_lengths']}")
print(f"[{mode}] /map msgs {r['map_msgs']}; known cells first {r['known_first']} -> last {r['known_last']} (max {r['known_max']}); occupied last {r['occupied_last']}; map size {r['size_last']}")
print(f"[{mode}] known trace (sim s, cells):", r['known_trace'][:16]); print(f"[{mode}] graph markers:", r['graph_markers'])
log=open(f'{D}/slam_{mode}.log',errors='ignore').read()
c=collections.Counter(re.findall(r'contains (\d+) range readings, expected (\d+)',log)); print(f"[{mode}] mismatch warnings: {sum(c.values())}", dict(c.most_common(5)))
