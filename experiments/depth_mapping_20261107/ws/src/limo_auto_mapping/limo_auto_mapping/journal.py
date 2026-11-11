"""Action 한 회차의 사건·최종 요약을 지도와 같은 prefix로 보관한다."""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import threading
import time
import uuid


class SessionJournal:
    def __init__(self, prefix):
        self.prefix = str(prefix)
        self.event_path = self.prefix + '.events.jsonl'
        self.summary_path = self.prefix + '.session.json'
        self.explorer_path = self.prefix + '.explorer.log'
        self.lock = threading.Lock()  # 센서·Action·rosout 콜백의 동시 쓰기를 직렬화한다.
        self.started = time.monotonic()
        self.closed = False
        self.finishing = False
        Path(prefix).parent.mkdir(parents=True, exist_ok=True)
        created = []
        self.stream = None
        try:
            for path in (self.event_path, self.summary_path, self.explorer_path):
                with open(path, 'x', encoding='utf-8'):
                    pass  # 기존 지도뿐 아니라 기존 기록도 덮어쓰지 않는다.
                created.append(path)
            self.stream = open(self.event_path, 'a', encoding='utf-8')
            Path(self.summary_path).write_text(json.dumps(
                {'state': 'running', 'output_prefix': self.prefix,
                 'note': '최종 요약이 없으면 강제 종료·전원 차단 등도 조사해야 함'},
                ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        except Exception:
            if self.stream is not None:
                self.stream.close()
            for path in created:
                Path(path).unlink(missing_ok=True)
            raise

    def event(self, kind, **fields):
        with self.lock:
            if self.closed or self.finishing:
                return
            item = {'timestamp_utc': datetime.now(timezone.utc).isoformat(),
                    'elapsed_sec': time.monotonic()-self.started, 'event': kind, **fields}
            self.stream.write(json.dumps(item, ensure_ascii=False, allow_nan=False)+'\n')
            self.stream.flush()  # 정상 종료 전이라도 지금까지의 사건을 읽을 수 있다.
            if kind == 'session_finished':
                self.finishing = True  # 늦게 도착한 rosout이 최종 사건 뒤에 끼어들지 않게 한다.

    def finish(self, summary):
        with self.lock:
            temporary = Path(self.summary_path + '.' + uuid.uuid4().hex + '.tmp')
            try:
                self.stream.flush()
                os.fsync(self.stream.fileno())
                with temporary.open('x', encoding='utf-8') as stream:
                    json.dump(summary, stream, ensure_ascii=False, indent=2, allow_nan=False)
                    stream.write('\n')
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(temporary, self.summary_path)
            finally:
                temporary.unlink(missing_ok=True)
                self.closed = True
                self.stream.close()
