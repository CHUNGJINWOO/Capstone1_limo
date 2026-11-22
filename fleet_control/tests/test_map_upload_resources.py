"""Loopback-only slow-body and upload concurrency checks."""
import contextlib, http.client, io, json, socket, struct, threading, time, unittest
from unittest.mock import patch
import test_map_registry_http_upload as base
from fleet_server import http_api

class Tests(unittest.TestCase):
 def setUp(self):self.t=base.RegistryUploadHttpTests('test_upload_unverified_preserves_existing_and_audits');self.t.setUp()
 def tearDown(self):self.t.tearDown()
 def test_slow_upload_slots_timeout_and_release(self):
  with patch.object(http_api,'MAP_UPLOAD_WAIT_S',.15),patch.object(http_api,'MAP_UPLOAD_READ_S',1.5):
   sockets=[]
   try:
    for _ in range(2):
     s=socket.create_connection(('127.0.0.1',self.t.port),timeout=3);s.settimeout(3)
     request=f'POST /api/v1/map-registry/maps HTTP/1.1\r\nHost: localhost\r\nAuthorization: Bearer {self.t.op}\r\nContent-Length: 1000000\r\n\r\n{{'.encode()
     s.sendall(request);sockets.append(s)
    time.sleep(.25)
    c=http.client.HTTPConnection('127.0.0.1',self.t.port,timeout=3)
    c.request('POST','/api/v1/map-registry/maps',body=b'{}',headers={'Authorization':'Bearer '+self.t.op,'Content-Type':'application/json'})
    resp=c.getresponse();self.assertEqual(resp.status,503);resp.read();c.close()
    for s in sockets:self.assertIn(b' 408 ',s.recv(1024))
    self.assertEqual(self.t.req('POST','/api/v1/map-registry/maps',self.t.op,{})[0],400)
   finally:
    for s in sockets:s.close()
 def test_two_simultaneous_megabyte_bodies(self):
  results=[];errors=[];start=threading.Barrier(3)
  def worker(i):
   try:
    data=json.dumps({'facility_id':'large'+str(i),'name':'large','padding':'x'*(1024*1024)}).encode()
    start.wait();results.append(self.t.req('POST','/api/v1/map-registry/facilities',self.t.op,raw=data)[0])
   except Exception as exc:errors.append(exc)
  threads=[threading.Thread(target=worker,args=(i,)) for i in range(2)]
  for t in threads:t.start()
  start.wait()
  for t in threads:t.join(10)
  self.assertFalse(errors);self.assertFalse(any(t.is_alive() for t in threads));self.assertEqual(sorted(results),[201,201])
 def test_disconnected_client_is_logged_and_slot_released(self):
  with self.assertLogs('fleet_server.http_api',level='INFO') as logs:
   s=socket.create_connection(('127.0.0.1',self.t.port),timeout=3)
   s.sendall(f'POST /api/v1/map-registry/maps HTTP/1.1\r\nHost: localhost\r\nAuthorization: Bearer {self.t.op}\r\nContent-Length: 1000\r\n\r\n{{'.encode())
   s.setsockopt(socket.SOL_SOCKET,socket.SO_LINGER,struct.pack('ii',1,0));s.close()
   deadline=time.monotonic()+3
   while not logs.output and time.monotonic()<deadline:time.sleep(.02)
  self.assertTrue(any('client connection closed' in line for line in logs.output))
  self.assertEqual(self.t.req('POST','/api/v1/map-registry/maps',self.t.op,{})[0],400)
 def test_unexpected_server_error_remains_visible(self):
  errors=io.StringIO()
  with contextlib.redirect_stderr(errors),patch.object(self.t.reg,'dispatch_write',side_effect=RuntimeError('injected internal failure')):
   self.assertEqual(self.t.req('POST','/api/v1/map-registry/maps',self.t.op,{})[0],500)
  self.assertIn('injected internal failure',errors.getvalue())
  self.assertEqual(self.t.req('POST','/api/v1/map-registry/maps',self.t.op,{})[0],400)

if __name__=='__main__':unittest.main(verbosity=2)
