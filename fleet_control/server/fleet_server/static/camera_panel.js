'use strict';
(() => {
  let robot = '', generation = 0, busy = false;
  const kinds = {rgb: 'Rgb', depth: 'Depth'};
  function show(label, url, state) {
    const img = $('camera' + label);
    if (img.dataset.url) URL.revokeObjectURL(img.dataset.url);
    img.dataset.url = url || '';
    if (url) img.src = url;
    else img.removeAttribute('src');
    img.hidden = !url;
    $('camera' + label + 'State').textContent = state;
  }
  window.cameraRobot = rid => {
    robot = rid; generation += 1;
    for (const label of Object.values(kinds)) show(label, null, '수신 대기');
  };
  setInterval(async () => {
    if (!robot || !token || busy) return;
    const selected = robot, stamp = generation;
    busy = true;
    try {
      await Promise.all(Object.entries(kinds).map(async ([kind, label]) => {
        try {
          const response = await api('/robots/' + encodeURIComponent(selected) + '/camera/' + kind + '.jpg',
            {signal: AbortSignal.timeout(1000)});
          if (!response.ok) throw Error('frame unavailable');
          const blob = await response.blob();
          if (stamp === generation) show(label, URL.createObjectURL(blob),
            response.headers.get('X-Camera-Source') === 'SIMULATED' ? '격리·모의 ROS' : 'ROS 수신 중');
        } catch {
          if (stamp === generation) show(label, null, '영상 없음 / 만료');
        }
      }));
    } finally { busy = false; }
  }, 650);
})();
