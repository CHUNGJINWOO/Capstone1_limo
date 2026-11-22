"""qr_encode.js produces QR codes a real decoder reads back (OpenCV QRCodeDetector with QUIRC, run with `python3 -s` to use the system NumPy).
Covers versions 1..10 (short text up to ~200 bytes) and the visitor-link shape. SKIP if OpenCV is unavailable."""
import json, os, subprocess, sys, tempfile, unittest
from pathlib import Path

JS = Path(__file__).resolve().parents[1] / 'server' / 'fleet_server' / 'static' / 'qr_encode.js'
DECODE = r'''
import sys, json, numpy as np, cv2
out = []
for item in json.load(open(sys.argv[1])):
    m = np.array(item['m'], dtype=np.uint8); n = m.shape[0]; q = 4; s = 8
    img = np.full(((n + 2 * q) * s, (n + 2 * q) * s), 255, np.uint8)
    for y in range(n):
        for x in range(n):
            if m[y, x]: img[(y + q) * s:(y + q + 1) * s, (x + q) * s:(x + q + 1) * s] = 0
    txt, pts, _ = cv2.QRCodeDetector().detectAndDecode(img)
    out.append(txt)
print(json.dumps(out))
'''


class QrEncode(unittest.TestCase):
    def test_roundtrip(self):
        texts = ['A', 'hello', 'http://127.0.0.1:8080/visitor.html#t=' + 'x' * 32,
                 'https://fleet.example.local:18190/visitor.html#t=Zq3-_' + 'k' * 27, 'B' * 70, 'C' * 106, 'D' * 152, 'E' * 200, '안내 로봇 방문객 링크']
        js = 'const q=require(%r); console.log(JSON.stringify(%s.map(t=>({t, n:q.matrix(t).length, m:q.matrix(t).map(r=>r.map(v=>v?1:0))}))))' % (str(JS), json.dumps(texts))
        enc = json.loads(subprocess.run(['node', '-e', js], capture_output=True, text=True, check=True).stdout)
        with tempfile.NamedTemporaryFile('w', suffix='.json', delete=False) as f: json.dump(enc, f)
        p = subprocess.run(['python3', '-s', '-c', DECODE, f.name], capture_output=True, text=True); os.unlink(f.name)
        if p.returncode != 0 and 'cv2' in p.stderr: self.skipTest('OpenCV unavailable: ' + p.stderr[-120:])
        if 'QUIRC is not linked' in p.stdout + p.stderr:
            self.skipTest('OpenCV here is built without its QR decoder (QUIRC); no other decoder installed - QR output NOT verified')
        got = json.loads(p.stdout.strip().splitlines()[-1])
        sizes = sorted({e['n'] for e in enc})
        self.assertEqual(got, texts, f'module sizes {sizes}')
        self.assertIn(57, sizes)                                   # version 10 exercised (57x57)


if __name__ == '__main__':
    unittest.main()
