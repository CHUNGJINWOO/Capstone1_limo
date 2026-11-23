'use strict';
(() => {
  let selected = '', busy = false, lease = null, models = {}, refreshing = false;
  let retry = null;
  const detail = message => { $('opsDetail').textContent = message; };
  const path = rid => '/robots/' + encodeURIComponent(rid);
  function oldRobotIdle(r) {
    const execution = r.state?.execution;
    const nav = execution?.nav2_goals;
    const preview = r.state?.manual_web_bench;
    return r.online && r.last_seen_age_s < 2 && execution?.active === null &&
      nav?.known === true && nav?.publishers === 1 && nav?.active_count === 0 &&
      (!r.agent?.manual_web_bench || (preview?.motor_output === false && preview.requested_linear === 0 && preview.requested_turn === 0));
  }
  window.selectOperationRobot = async rid => {
    if (rid === selected) return true;
    if (!(data?.robots || []).some(robot => robot.robot_id === rid)) return false;
    if (busy) return false;
    busy = true;
    try {
      if (selected) await window.stopManualPreview?.('로봇 전환 · 입력 중단');
      const oldLock = selected && await api(path(selected) + '/lock', {signal: AbortSignal.timeout(700)});
      const oldLease = oldLock?.ok ? (await oldLock.json()).lease : null;
      if (selected && oldLock && !oldLock.ok) throw Error('이전 Lease 조회 실패');
      if (selected && oldLease && oldLease.owner === principalName) {
        await new Promise(resolve => setTimeout(resolve, 300));
        const until = performance.now() + 2200;
        let idle = false;
        while (performance.now() < until) {
          const prefix = path(selected);
          const [r, m, manual] = await Promise.all(['', '/mapping', '/manual-bench'].map(
            part => api(prefix + part, {signal: AbortSignal.timeout(700)})));
          if (r.ok && m.ok && manual.ok && !(await manual.json()).active && oldRobotIdle(await r.json()) &&
              ['IDLE', 'STOPPED'].includes((await m.json()).state)) { idle = true; break; }
          await new Promise(resolve => setTimeout(resolve, 150));
        }
        if (!idle) throw Error('이전 로봇의 영속도와 Mapping 종료를 확인하지 못했습니다.');
        const released = await api(path(selected) + '/lock', {
          method: 'POST', body: JSON.stringify({action: 'release'})
        });
        if (!released.ok) throw Error('이전 로봇의 Lease 반환을 확인하지 못했습니다.');
      }
      selected = rid; lease = null; models = {}; retry = null;
      $('opsLease').textContent = 'Lease 조회 중';
      $('opsSafety').textContent = '모드·Gate·모터 경로 조회 중';
      $('opsMap').textContent = 'Mapping 조회 중';
      $('opsBag').textContent = 'Bag 조회 중';
      $('gpRobot').value = rid;
      $('opsSelect').value = rid;
      $('mrobot').value = rid;
      window.cameraRobot?.(rid);
      if ([...$('mapRobot').options].some(option => option.value === rid)) {
        $('mapRobot').value = rid; drawMap();
      }
      $('opsRobot').textContent = rid;
      detail('제어권을 확인한 뒤 명령을 보내세요.');
      return true;
    } catch (error) { detail(error.message); $('gpRobot').value = selected; $('opsSelect').value = selected; $('mapRobot').value = selected; drawMap(); return false; }
    finally { busy = false; }
  };
  async function refresh() {
    const ids = (data?.robots || []).map(r => r.robot_id);
    $('gpRobots').replaceChildren(...ids.map(id => new Option(id, id)));
    fillSelect($('opsSelect'), ids);
    if (!ids.length || refreshing) return;
    if (!selected) await window.selectOperationRobot(ids[0]);
    if (!selected) return;
    refreshing = true;
    const rid = selected;
    try {
      const results = await Promise.all(['lock', 'mapping', 'bag'].map(async part => {
        const response = await api(path(rid) + '/' + part, {signal: AbortSignal.timeout(1500)});
        if (!response.ok) throw Error('상태 조회 실패');
        return response.json();
      }));
      if (rid !== selected) return;
      lease = results[0].lease;
      models = {mapping: results[1], bag: results[2]};
      const robot = data?.robots?.find(r => r.robot_id === rid);
      const fresh = robot?.online && robot.last_seen_age_s < 2;
      $('opsRobot').textContent = rid + ' · ' + (fresh ? (robot.agent?.mode || 'ONLINE') : '오프라인 / 오래됨');
      $('opsLease').textContent = lease ? `Lease ${lease.owner} · epoch ${lease.epoch} · ${lease.remaining_s}초` : 'Lease 없음';
      const m = models.mapping, b = models.bag;
      const status = fresh ? robot.state || {} : {}, agent = robot?.agent || {};
      const source = kind => agent['isolated_ros_' + kind] ? '격리 ROS' : agent.mapping_fixture ? 'Mock' : '비활성';
      const arbiter = status.velocity_arbiter?.mode;
      const mode = arbiter || (status.manual_web_bench?.session_id ? 'MANUAL 미리보기' :
        m.state === 'MAPPING' ? 'AUTO Mapping' : '대기');
      $('opsSafety').textContent = '모드 ' + (fresh ? mode : 'UNKNOWN') + ' · Gate ' + (status.safety_gate?.status || 'UNKNOWN') +
        ' · 모터 경로 ' + (status.safety_gate?.motor_path?.status || 'UNKNOWN');
      $('opsMap').textContent = 'Mapping [' + source('mapping') + '] ' + m.state + ' · ' + (m.command?.status || '—');
      $('opsBag').textContent = 'Bag [' + source('bag') + '] ' + b.state + ' · ' + (b.command?.status || '—');
      if (m.result?.detail) $('opsMap').textContent += ' · ' + m.result.detail;
      if (b.result?.detail) $('opsBag').textContent += ' · ' + b.result.detail;
      $('gpMode').textContent = agent.locked_velocity_arbiter ? '격리 ROS · 모터 잠금' :
        agent.manual_web_bench && agent.mode === 'real' ? '실차 관측 전용' : '비활성';
      if (m.last_save) {
        const e = m.last_save.evidence;
        $('opsMap').textContent += ` · YAML ${e.yaml_ok} · PGM ${e.pgm_ok} · posegraph ${e.posegraph_ok}`;
      }
      if (m.detail) $('opsMap').textContent += ' · ' + m.detail;
      if (b.detail) $('opsBag').textContent += ' · ' + b.detail;
      if (status.bag_worker?.reason) $('opsBag').textContent += ' · ' + status.bag_worker.reason;
    } catch {
      if (selected === rid) {
        lease = null; models = {};
        $('opsMap').textContent = 'Mapping 상태 오래됨';
        $('opsBag').textContent = 'Bag 상태 오래됨';
        $('opsLease').textContent = 'Lease 상태 오래됨';
        $('opsSafety').textContent = '모드·Gate·모터 경로 UNKNOWN';
      }
    } finally { refreshing = false; }
  }
  window.sendOperation = async function (operation) {
    if (operation === 'video') { $('robotVideo').scrollIntoView(); return; }
    if (operation === 'software') {
      await window.stopManualPreview?.('소프트웨어 정지 요청');
      detail('수동 입력 중단 요청. 물리 비상정지 상태는 확인되지 않습니다.');
      if (!['MAPPING', 'PAUSED'].includes(models.mapping?.state)) return;
      operation = 'mapping:STOP';
    }
    if (!selected || !lease || busy) { detail('선택 로봇과 Fleet Lease를 확인하세요.'); return; }
    const [kind, requested] = operation.split(':');
    let action = requested;
    if (action === 'TOGGLE') action = kind === 'bag' ? (models.bag?.state === 'RECORDING' ? 'STOP' : 'START') :
      (models.mapping?.state === 'PAUSED' ? 'RESUME' : 'PAUSE');
    const sid = models[kind]?.session_id;
    const key = [selected, lease.epoch, kind, action, sid].join('/');
    const id = retry?.key === key ? retry.id : uuid();
    busy = true;
    try {
      const response = await api(path(selected) + '/' + kind, {method: 'POST',
        body: JSON.stringify({action, epoch: lease.epoch, session_id: sid, idempotency_key: id}),
        signal: AbortSignal.timeout(2000)});
      const result = await response.json();
      if (!response.ok) throw Error(result.error?.message || '명령 거부');
      retry = null;
      detail('요청 등록 · ' + result.command_id);
      await refresh();
    } catch (error) {
      retry = {key, id};
      detail(error.message);
    } finally { busy = false; }
  };
  $('opsCommands').onclick = event => {
    if (event.target.dataset.operation) window.sendOperation(event.target.dataset.operation);
  };
  $('gpRobot').addEventListener('change', () => window.selectOperationRobot($('gpRobot').value));
  $('opsSelect').onchange = () => window.selectOperationRobot($('opsSelect').value);
  $('mapRobot').onchange = () => window.selectOperationRobot($('mapRobot').value);
  setInterval(refresh, 750);
})();
