"""Read-only RGB adapter derived from the existing project sender. Subscribes to RGB and reuses existing vlink/YOLO TCP protocol. No ROS publishers, actions, services or sensor/control configuration writes. Existing token is read from file; never printed."""
import collections, json, os, socket, sys, threading, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import numpy as np, rclpy
try:
    import cv2
except ImportError:
    cv2 = None
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
try:
    from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
    HAVE_VISION = True
except ImportError:
    HAVE_VISION = False
from vlink import protocol as P

def jpeg_size(b: bytes):
    """(width, height) from the first SOF marker, or None. Lets compressed passthrough keep w/h without decoding."""
    i = 2
    while i + 9 < len(b):
        if b[i] != 255:
            i += 1
            continue
        m = b[i + 1]
        if m in (192, 193, 194):
            return (b[i + 7] << 8 | b[i + 8], b[i + 5] << 8 | b[i + 6])
        if m in (216, 1) or 208 <= m <= 215:
            i += 2
            continue
        i += 2 + (b[i + 2] << 8 | b[i + 3])
    return None

class Sender(Node):

    def __init__(self):
        super().__init__('ai_video_sender', start_parameter_services=False, enable_rosout=False)
        d = self.declare_parameter
        d('robot_id', 'sim1')
        d('stream_id', 'rgb')
        d('image_topic', '/camera/color/image_raw')
        d('server_host', '127.0.0.1')
        d('server_port', 18171)
        d('token_file', '')
        d('fps', 10.0)
        d('jpeg_quality', 80)
        d('scale', 1.0)
        d('max_frame_age_s', 0.5)
        d('max_result_age_s', 1.5)
        d('result_timeout_s', 2.0)
        d('compressed', False)
        d('detections_topic', '/ai/detections')
        d('status_topic', '/ai/link_status')
        d('send_timeout_s', 2.0)
        g = lambda n: self.get_parameter(n).value
        self.cfg = {k: g(k) for k in ('robot_id', 'stream_id', 'image_topic', 'server_host', 'server_port', 'fps', 'jpeg_quality', 'scale', 'max_frame_age_s', 'max_result_age_s', 'result_timeout_s', 'send_timeout_s', 'compressed')}
        self.token = open(g('token_file')).read().strip()
        self.lock = threading.Lock()
        self.cv = threading.Condition(self.lock)
        self.send_lock = threading.Lock()
        self.clock_samples = collections.deque(maxlen=20)
        self.cpu_prev = None
        self.proc_prev = None
        self.slot = None
        self.src_n = 0
        self.src_t = collections.deque(maxlen=200)
        self.stats = collections.defaultdict(int)
        self.rtt = collections.deque(maxlen=300)
        self.e2e = collections.deque(maxlen=300)
        self.sz = collections.deque(maxlen=300)
        self.sent_t = collections.deque(maxlen=300)
        self.res_t = collections.deque(maxlen=300)
        self.sent_bytes_t = collections.deque(maxlen=300)
        self.inflight = {}
        self.seq = 0
        self.last_result_wall = None
        self.connected = False
        self.sock = None
        self.stop = threading.Event()
        self.state = 'CONNECTING'
        self.pub_det = None if HAVE_VISION else None
        self.pub_det_json = None
        self.pub_status = None
        self.create_subscription(CompressedImage if g('compressed') else Image, str(g('image_topic')), self.on_image, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST))
        self.create_timer(1.0, self.publish_status)
        threading.Thread(target=self.link_loop, daemon=True).start()
        self.get_logger().info(f"sender up: topic={g('image_topic')} -> {g('server_host')}:{g('server_port')} fps<={g('fps')} q={g('jpeg_quality')} scale={g('scale')}")

    def on_image(self, msg: Image):
        now = time.time()
        with self.cv:
            if self.slot is not None:
                self.stats['src_overwritten'] += 1
            self.slot = (msg, now)
            self.src_n += 1
            self.src_t.append(now)
            self.cv.notify()

    def to_bgr(self, msg):
        (w, h, step) = (msg.width, msg.height, msg.step)
        raw = np.frombuffer(msg.data, np.uint8)
        if msg.encoding in ('rgb8', 'bgr8'):
            if len(raw) < h * step or step < w * 3:
                return None
            a = raw[:h * step].reshape(h, step)[:, :w * 3].reshape(h, w, 3)
            return np.ascontiguousarray(a[:, :, ::-1]) if msg.encoding == 'rgb8' else a
        if msg.encoding == 'mono8':
            if len(raw) < h * step:
                return None
            return raw[:h * step].reshape(h, step)[:, :w]
        return None

    def _send(self, s, data):
        with self.send_lock:
            s.sendall(data)

    def connect(self):
        s = socket.create_connection((self.cfg['server_host'], int(self.cfg['server_port'])), timeout=3.0)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.sendall(P.pack({'type': 'hello', 'robot_id': self.cfg['robot_id'], 'stream_id': self.cfg['stream_id'], 'proto': P.PROTO, 'token': self.token, 'codec': 'jpeg'}))
        s.settimeout(5.0)
        (h, _) = P.read_message(s)
        if h.get('type') != 'hello_ack':
            raise ConnectionError(f"server refused: {h.get('reason', h.get('type'))}")
        s.settimeout(float(self.cfg['send_timeout_s']))
        return s

    def link_loop(self):
        backoff = 1.0
        while not self.stop.is_set():
            try:
                self.state = 'CONNECTING'
                s = self.connect()
                self.sock = s
                self.connected = True
                backoff = 1.0
                self.inflight.clear()
                if self.stats['connects']:
                    self.stats['reconnects'] += 1
                self.stats['connects'] += 1
                self.get_logger().info('connected to the AI host')
                rd = threading.Thread(target=self.reader, args=(s,), daemon=True)
                rd.start()
                self.send_loop(s)
            except (OSError, ConnectionError, P.ProtocolError, ValueError) as e:
                self.stats['link_errors'] += 1
                self.get_logger().warn(f'link down ({type(e).__name__}: {e}); retry in {backoff:.0f}s', throttle_duration_sec=5.0)
            finally:
                self.connected = False
                try:
                    self.sock and self.sock.close()
                except OSError:
                    pass
                self.sock = None
            self.stop.wait(backoff)
            backoff = min(backoff * 2, 5.0)

    def send_loop(self, s):
        """Event driven: a frame is sent as soon as it arrives, but never faster than the fps parameter; while the link is slower than the source the NEWEST frame wins (old ones are dropped)."""
        period = 1.0 / max(float(self.cfg['fps']), 0.1)
        last_send = 0.0
        last_ping = time.time()
        while not self.stop.is_set() and self.connected:
            now = time.time()
            if now - last_ping > 2.0:
                self._send(s, P.pack({'type': 'ping', 't_ns': time.time_ns()}))
                last_ping = now
            with self.cv:
                if self.slot is None:
                    self.cv.wait(timeout=0.2)
                have = self.slot is not None
            if not have:
                continue
            wait = last_send + period - time.time()
            if wait > 0:
                time.sleep(wait)
            with self.lock:
                item = self.slot
                self.slot = None
            last_send = time.time()
            if item is not None:
                (msg, t_in) = item
                if time.time() - t_in > float(self.cfg['max_frame_age_s']):
                    self.stats['stale_before_send'] += 1
                elif self.cfg['compressed']:
                    data = bytes(msg.data)
                    wh = jpeg_size(data) if 'jpeg' in msg.format.lower() else None
                    if wh is None:
                        self.stats['unsupported_image'] += 1
                    else:
                        self.seq += 1
                        sent = time.time()
                        hdr = {'type': 'frame', 'seq': self.seq, 'frame_id': msg.header.frame_id, 'stamp_ns': msg.header.stamp.sec * 10 ** 9 + msg.header.stamp.nanosec, 'capture_wall_ns': int(t_in * 1000000000.0), 'sent_wall_ns': int(sent * 1000000000.0), 'w': wh[0], 'h': wh[1], 'quality': -1, 'enc': 'jpeg'}
                        self.inflight[self.seq] = (t_in, sent, hdr['stamp_ns'], hdr['frame_id'])
                        if len(self.inflight) > 600:
                            self.inflight.pop(min(self.inflight))
                        self._send(s, P.pack(hdr, data))
                        self.stats['sent'] += 1
                        self.sz.append(len(data))
                        self.sent_t.append(sent)
                        self.sent_bytes_t.append((sent, len(data) + 200))
                else:
                    bgr = self.to_bgr(msg) if cv2 is not None else None
                    if cv2 is None:
                        self.stats['no_cv2_for_raw_image'] += 1
                    elif bgr is None:
                        self.stats['unsupported_image'] += 1
                    else:
                        sc = float(self.cfg['scale'])
                        if sc != 1.0:
                            bgr = cv2.resize(bgr, None, fx=sc, fy=sc, interpolation=cv2.INTER_AREA)
                        (ok, jpg) = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, int(self.cfg['jpeg_quality'])])
                        if ok:
                            self.seq += 1
                            sent = time.time()
                            data = jpg.tobytes()
                            hdr = {'type': 'frame', 'seq': self.seq, 'frame_id': msg.header.frame_id, 'stamp_ns': msg.header.stamp.sec * 10 ** 9 + msg.header.stamp.nanosec, 'capture_wall_ns': int(t_in * 1000000000.0), 'sent_wall_ns': int(sent * 1000000000.0), 'w': int(bgr.shape[1]), 'h': int(bgr.shape[0]), 'quality': int(self.cfg['jpeg_quality']), 'enc': 'jpeg'}
                            self.inflight[self.seq] = (t_in, sent, hdr['stamp_ns'], hdr['frame_id'])
                            if len(self.inflight) > 600:
                                self.inflight.pop(min(self.inflight))
                            self._send(s, P.pack(hdr, data))
                            self.stats['sent'] += 1
                            self.sz.append(len(data))
                            self.sent_t.append(sent)
                            self.sent_bytes_t.append((sent, len(data) + 200))

    def reader(self, s):
        try:
            while not self.stop.is_set():
                s.settimeout(None)
                (h, _) = P.read_message(s)
                now = time.time()
                if h.get('type') == 'pong':
                    (t0, t1, ts) = (h.get('t_ns'), time.time_ns(), h.get('server_wall_ns'))
                    if t0 and ts:
                        self.clock_samples.append((t1 - t0, ts - (t0 + t1) // 2))
                    continue
                if h.get('type') != 'result':
                    continue
                self.stats['dropped_before_total'] += int(h.get('dropped_before') or 0)
                inf = self.inflight.pop(h.get('seq'), None)
                if inf is None:
                    self.stats['unmatched_results'] += 1
                    continue
                (t_in, sent, stamp_ns, frame_id) = inf
                if h.get('stamp_ns') == stamp_ns and h.get('frame_id') == frame_id:
                    self.stats['stamp_match'] += 1
                else:
                    self.stats['stamp_mismatch'] += 1
                st = h.get('status')
                self.last_result_wall = now
                if st == 'stale_drop':
                    self.stats['server_stale_drop'] += 1
                    continue
                if st != 'ok':
                    self.stats['server_error'] += 1
                    continue
                self.rtt.append((now - sent) * 1000)
                self.e2e.append((now - t_in) * 1000)
                self.res_t.append(now)
                if now - t_in > float(self.cfg['max_result_age_s']):
                    self.stats['stale_results_blocked'] += 1
                    continue
                self.stats['results_ok'] += 1
                dets = h.get('detections', [])
                if HAVE_VISION:
                    out = Detection2DArray()
                    out.header.stamp.sec = stamp_ns // 10 ** 9
                    out.header.stamp.nanosec = stamp_ns % 10 ** 9
                    out.header.frame_id = frame_id
                    for (i, dt) in enumerate(dets):
                        d = Detection2D()
                        d.header = out.header
                        hyp = ObjectHypothesisWithPose()
                        hyp.hypothesis.class_id = str(dt['cls'])
                        hyp.hypothesis.score = float(dt['conf'])
                        d.results.append(hyp)
                        d.bbox.center.position.x = (dt['x1'] + dt['x2']) / 2
                        d.bbox.center.position.y = (dt['y1'] + dt['y2']) / 2
                        d.bbox.size_x = dt['x2'] - dt['x1']
                        d.bbox.size_y = dt['y2'] - dt['y1']
                        d.id = f"{h.get('seq')}_{i}"
                        out.detections.append(d)
                    pass
                pass
                self.stats['last_n'] = len(dets)
        except (OSError, ConnectionError, P.ProtocolError, ValueError):
            pass
        finally:
            self.connected = False
            try:
                s.close()
            except OSError:
                pass

    def sample_system(self):
        """Robot-side load from /proc (read only): total CPU %, memory, and this process' CPU %/RSS."""
        now = time.time()
        out = {}
        try:
            f = open('/proc/stat').readline().split()[1:]
            tot = sum((int(x) for x in f))
            idle = int(f[3]) + int(f[4])
            if self.cpu_prev and tot > self.cpu_prev[0]:
                out['cpu_pct_total'] = round(100 * (1 - (idle - self.cpu_prev[1]) / (tot - self.cpu_prev[0])), 1)
            self.cpu_prev = (tot, idle)
            mem = {}
            for l in open('/proc/meminfo'):
                (k, v) = l.split(':')
                if k in ('MemTotal', 'MemAvailable'):
                    mem[k] = int(v.split()[0])
            out['mem_used_mb'] = round((mem['MemTotal'] - mem['MemAvailable']) / 1024)
            out['mem_total_mb'] = round(mem['MemTotal'] / 1024)
            st = open('/proc/self/stat').read().rsplit(')', 1)[1].split()
            pt = int(st[11]) + int(st[12])
            if self.proc_prev:
                out['proc_cpu_pct_of_1core'] = round(100 * (pt - self.proc_prev[0]) / os.sysconf('SC_CLK_TCK') / (now - self.proc_prev[1]), 1)
            self.proc_prev = (pt, now)
            out['proc_rss_mb'] = round(int([l for l in open('/proc/self/status') if l.startswith('VmRSS')][0].split()[1]) / 1024, 1)
            out['load1'] = round(os.getloadavg()[0], 2)
        except (OSError, ValueError, IndexError, KeyError):
            out['error'] = 'could not read /proc'
        return out

    def clock_estimate(self):
        """NTP-style estimate from ping/pong: server_clock - robot_clock at the midpoint of the best (lowest RTT) exchange. It is an ESTIMATE bounded by rtt/2, not a synchronisation."""
        if not self.clock_samples:
            return None
        (rtt, off) = min(self.clock_samples, key=lambda x: x[0])
        return {'offset_ms_est': round(off / 1000000.0, 1), 'uncertainty_ms': round(rtt / 2000000.0, 1), 'samples': len(self.clock_samples)}

    def publish_status(self):
        now = time.time()
        rate = lambda dq, w=5.0: round(sum((1 for t in dq if now - t <= w)) / w, 2)
        age = None if self.last_result_wall is None else now - self.last_result_wall
        if not self.connected:
            st = 'DISCONNECTED' if self.stats['connects'] else 'CONNECTING'
        elif age is None or age > float(self.cfg['result_timeout_s']):
            st = 'NO_RESULT'
        else:
            st = 'OK'
        self.state = st

        def pct(dq, q):
            v = sorted(dq)
            return round(v[int(q * (len(v) - 1))], 1) if v else None
        kbps = round(sum((b for (t, b) in self.sent_bytes_t if now - t <= 5.0)) * 8 / 5.0 / 1000, 1)
        j = {'ai_link': st, 'connected': self.connected, 'robot_id': self.cfg['robot_id'], 'topic': self.cfg['image_topic'], 'src_fps': rate(self.src_t), 'sent_fps': rate(self.sent_t), 'result_fps': rate(self.res_t), 'rtt_ms_avg': round(sum(self.rtt) / len(self.rtt), 1) if self.rtt else None, 'rtt_ms_p95': pct(self.rtt, 0.95), 'e2e_ms_avg': round(sum(self.e2e) / len(self.e2e), 1) if self.e2e else None, 'e2e_ms_p95': pct(self.e2e, 0.95), 'jpeg_bytes_avg': round(sum(self.sz) / len(self.sz)) if self.sz else None, 'out_kbps': kbps, 'last_result_age_s': None if age is None else round(age, 2), 'last_detection_count': self.stats.get('last_n'), 'counters': dict(self.stats), 'cfg': {k: self.cfg[k] for k in ('fps', 'jpeg_quality', 'scale', 'max_frame_age_s', 'max_result_age_s')}, 't': now, 'note': 'last_detection_count=0 with ai_link=OK means nothing was found; NO_RESULT/DISCONNECTED means the AI result path is down.'}
        j['system'] = self.sample_system()
        j['clock'] = self.clock_estimate()
        j['vision_msgs'] = HAVE_VISION
        j['opencv'] = cv2 is not None
        pass
        sk = self.sock
        if sk is not None and self.connected:
            try:
                self._send(sk, P.pack({'type': 'telemetry', 'system': j['system'], 'clock': j['clock'], 'counters': j['counters'], 'src_fps': j['src_fps'], 'sent_fps': j['sent_fps'], 'result_fps': j['result_fps'], 'rtt_ms_avg': j['rtt_ms_avg'], 'e2e_ms_avg': j['e2e_ms_avg'], 'ai_link': st}))
            except OSError:
                pass

    def destroy_node(self):
        self.stop.set()
        super().destroy_node()

def main():
    rclpy.init()
    n = Sender()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == '__main__':
    main()
