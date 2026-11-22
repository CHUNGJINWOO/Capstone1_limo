"""Decode snapshot QR matrices with the existing system libzbar; no installation or real tokens."""
import ctypes,json,os,subprocess,unittest
from pathlib import Path

class QR(unittest.TestCase):
 def test_real_decode(self):
  js=Path(os.environ['CAPSTONE45_SNAPSHOT'])/'fleet_control/server/fleet_server/static/qr_encode.js'
  texts=['A','hello','http://127.0.0.1:8080/visitor.html#t='+'x'*32,
         'https://fleet.example.local:18190/visitor.html#t='+'k'*32,'B'*70,'C'*106,'D'*152,'E'*200,'안내 로봇 방문객 링크']
  script='const q=require(%s);console.log(JSON.stringify(%s.map(t=>q.matrix(t))))' % (json.dumps(str(js)),json.dumps(texts,ensure_ascii=False))
  matrices=json.loads(subprocess.run(['node','-e',script],capture_output=True,text=True,check=True).stdout)
  z=ctypes.CDLL('libzbar.so.0');P=ctypes.c_void_p;I=ctypes.c_int;U=ctypes.c_uint
  for name,restype,args in [('zbar_image_scanner_create',P,[]),('zbar_image_create',P,[]),('zbar_image_set_format',None,[P,U]),('zbar_image_set_size',None,[P,U,U]),('zbar_image_set_data',None,[P,P,ctypes.c_ulong,P]),('zbar_scan_image',I,[P,P]),('zbar_image_first_symbol',P,[P]),('zbar_symbol_get_data',P,[P]),('zbar_symbol_get_data_length',U,[P]),('zbar_image_destroy',None,[P]),('zbar_image_scanner_destroy',None,[P])]:
   fn=getattr(z,name);fn.restype=restype;fn.argtypes=args
  scanner=z.zbar_image_scanner_create();self.assertTrue(scanner)
  try:
   for idx,(matrix,want) in enumerate(zip(matrices,texts)):
    side=len(matrix);quiet=4;scale=8;width=(side+2*quiet)*scale
    empty=bytes([255])*width;rows=[empty]*(quiet*scale)
    for row in matrix:
     line=bytes([255])*(quiet*scale)+b''.join(bytes([0 if cell else 255])*scale for cell in row)+bytes([255])*(quiet*scale)
     rows.extend([line]*scale)
    rows.extend([empty]*(quiet*scale));pixels=b''.join(rows);buf=ctypes.create_string_buffer(pixels);image=z.zbar_image_create()
    z.zbar_image_set_format(image,0x30303859);z.zbar_image_set_size(image,width,width);z.zbar_image_set_data(image,buf,len(pixels),None)
    count=z.zbar_scan_image(scanner,image);symbol=z.zbar_image_first_symbol(image)
    got=ctypes.string_at(z.zbar_symbol_get_data(symbol),z.zbar_symbol_get_data_length(symbol)).decode('utf8') if symbol else ''
    z.zbar_image_destroy(image)
    with self.subTest(case=idx):self.assertGreater(count,0);self.assertEqual(got,want)
   print('Decoded 9 synthetic QR cases with libzbar; no real token used.',flush=True)
  finally:z.zbar_image_scanner_destroy(scanner)
if __name__=='__main__':unittest.main(verbosity=2)
