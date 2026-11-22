"""Isolated local test: large uploads and server-process resource sampling."""
import http.client,json,multiprocessing,threading,time,unittest
import psutil
import test_map_registry_http_upload as base

def server(pipe,stop):
 t=base.RegistryUploadHttpTests('test_upload_unverified_preserves_existing_and_audits');t.setUp()
 try:pipe.send((t.port,t.op));stop.wait(90)
 finally:t.tearDown()

class Tests(unittest.TestCase):
 def test_two_near_limit_and_disconnect(self):
  parent,child=multiprocessing.Pipe();stop=multiprocessing.Event();proc=multiprocessing.Process(target=server,args=(child,stop));proc.start()
  try:
   self.assertTrue(parent.poll(10));port,token=parent.recv();ps=psutil.Process(proc.pid)
   rss_before=ps.memory_info().rss;peak=[rss_before];threads_peak=[ps.num_threads()];monitor_stop=threading.Event()
   def monitor():
    while not monitor_stop.wait(.01):
     try:peak[0]=max(peak[0],ps.memory_info().rss);threads_peak[0]=max(threads_peak[0],ps.num_threads())
     except psutil.Error:return
   watch=threading.Thread(target=monitor);watch.start()
   # A complete 48 MiB minus 1 KiB JSON request. Both have identical size.
   prefix=b'{"facility_id":"loadx","name":"load","padding":"';suffix=b'"}'
   size=48*1024*1024-1024;body=prefix+b'x'*(size-len(prefix)-len(suffix))+suffix
   barrier=threading.Barrier(3);results=[];durations=[];errors=[]
   def send():
    try:
     c=http.client.HTTPConnection('127.0.0.1',port,timeout=40);barrier.wait();start=time.monotonic()
     c.request('POST','/api/v1/map-registry/facilities',body=body,headers={'Authorization':'Bearer '+token,'Content-Type':'application/json'})
     r=c.getresponse();r.read();durations.append(time.monotonic()-start);results.append(r.status);c.close()
    except Exception as e:errors.append(repr(e))
   workers=[threading.Thread(target=send) for _ in range(2)]
   for w in workers:w.start()
   barrier.wait()
   for w in workers:w.join(50)
   monitor_stop.set();watch.join(2)
   self.assertFalse(errors);self.assertFalse(any(w.is_alive() for w in workers));self.assertEqual(sorted(results),[201,201])
   print('LOAD_METRICS',json.dumps({'body_bytes_each':len(body),'concurrency':2,'server_baseline_rss_bytes':rss_before,'server_peak_rss_bytes':peak[0],'server_peak_threads':threads_peak[0],'request_seconds':durations}),flush=True)
   c=http.client.HTTPConnection('127.0.0.1',port,timeout=5);c.connect()
   c.sock.sendall(('POST /api/v1/map-registry/maps HTTP/1.1\r\nHost: localhost\r\nAuthorization: Bearer '+token+'\r\nContent-Length: 1000\r\n\r\n{').encode());c.close()
   time.sleep(.15)
   c=http.client.HTTPConnection('127.0.0.1',port,timeout=5);c.request('POST','/api/v1/map-registry/maps',body='{}',headers={'Authorization':'Bearer '+token});r=c.getresponse();self.assertEqual(r.status,400);r.read();c.close()
  finally:stop.set();proc.join(5);self.assertFalse(proc.is_alive())

if __name__=='__main__':unittest.main(verbosity=2)
