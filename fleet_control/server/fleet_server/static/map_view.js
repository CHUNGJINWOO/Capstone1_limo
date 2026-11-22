'use strict';
(function (root) {
  const H = root.FleetHelpers;

  class FleetMapView {
    constructor(canvas, opts) {
      this.cv = canvas;
      this.ctx = canvas.getContext('2d');
      this.opts = opts || {};
      this.cache = {};       // robot_id -> { version, img }
      this.trails = {};      // robot_id -> [[x, y], ...]
      this.sensors = {};     // robot_id -> { data, costmapImg }
      this.previewGoal = null;
      this.robot = null;
      this.missions = [];
      this.fitKey = null;
      this.layers = {
        map: true,
        trail: true,
        goals: true,
        costmap: true,
        plan: true,
      };
      this.view = {
        x: 0,
        y: 0,
        zoom: 45,
        rotation: 0,
        panX: 0,
        panY: 0,
        width: 640,
        height: 480,
      };
      this._pending = false;
      this.autoFit = true;
      this._bindEvents();
      this.resizeObserver = new ResizeObserver(() => this.resize());
      this.resizeObserver.observe(canvas.parentElement);
      this.resize();
    }

    _bindEvents() {
      let drag = null;
      let moved = false;
      this.cv.addEventListener('pointerdown', (e) => {
        drag = { x: e.clientX, y: e.clientY, px: this.view.panX, py: this.view.panY };
        moved = false;
        this.cv.setPointerCapture(e.pointerId);
      });
      this.cv.addEventListener('pointermove', (e) => {
        const rect = this.cv.getBoundingClientRect();
        const p = { x: e.clientX - rect.left, y: e.clientY - rect.top };
        if (drag) {
          const dx = e.clientX - drag.x;
          const dy = e.clientY - drag.y;
          if (Math.hypot(dx, dy) > 4) { moved = true; this.autoFit = false; }
          this.view.panX = drag.px + dx;
          this.view.panY = drag.py + dy;
          this.request();
        }
        const w = H.screenToWorld(this.view, p);
        if (this.opts.onCursor) this.opts.onCursor(w);
      });
      this.cv.addEventListener('pointerup', (e) => {
        const wasClick = drag && !moved;
        drag = null;
        if (wasClick) {
          if (this.readOnly) return;
          const rect = this.cv.getBoundingClientRect();
          const p = { x: e.clientX - rect.left, y: e.clientY - rect.top };
          const w = H.screenToWorld(this.view, p);
          if (this.registryPreview) {
            this.previewGoal = null;
            if (this.opts.onPreviewInspect) this.opts.onPreviewInspect(w);
            this.request();
            return;
          }
          this.previewGoal = { x: w.x, y: w.y };
          if (this.opts.onPickPoint) this.opts.onPickPoint(w);
          this.request();
        }
      });
      this.cv.addEventListener('pointercancel', () => { drag = null; });
      this.cv.addEventListener('wheel', (e) => {
        e.preventDefault();
        const rect = this.cv.getBoundingClientRect();
        this.zoomBy(Math.exp(-e.deltaY * 0.0012), {
          x: e.clientX - rect.left,
          y: e.clientY - rect.top,
        });
      }, { passive: false });
    }

    resize() {
      const parent = this.cv.parentElement;
      if (!parent) return;
      const rect = parent.getBoundingClientRect();
      if (!rect.width || !rect.height) return; // Hidden detail tabs must not reset the view.
      const dpr = window.devicePixelRatio || 1;
      const w = Math.max(1, Math.round(parent.clientWidth || 640));
      const h = Math.max(1, Math.round(parent.clientHeight || 460));
      if (this.view.width === w && this.view.height === h && this.cv.width === Math.round(w * dpr) && this.cv.height === Math.round(h * dpr)) return;
      this.view.width = w;
      this.view.height = h;
      this.cv.width = Math.round(w * dpr);
      this.cv.height = Math.round(h * dpr);
      if (this.autoFit) this.fit();
      this.draw();
    }

    zoomBy(k, anchor) {
      this.autoFit = false;
      const p = anchor || { x: this.view.width / 2, y: this.view.height / 2 };
      const before = H.screenToWorld(this.view, p);
      this.view.zoom = Math.max(5, Math.min(1200, this.view.zoom * k));
      const after = H.worldToScreen(this.view, before);
      this.view.panX += p.x - after.x;
      this.view.panY += p.y - after.y;
      this.request();
    }

    setRotationDeg(deg) {
      this.view.rotation = (Number(deg) || 0) * Math.PI / 180;
      this.request();
    }

    setLayer(name, visible) {
      this.layers[name] = Boolean(visible);
      this.request();
    }

    setPreviewGoal(x, y) {
      if (this.registryPreview) {
        this.previewGoal = null;
        this.request();
        return;
      }
      if (Number.isFinite(x) && Number.isFinite(y)) {
        this.previewGoal = { x, y };
      } else {
        this.previewGoal = null;
      }
      this.request();
    }

    fitRobot() {
      const r = this.robot, p = r?.state?.pose;
      const st = r?.state || {};
      if (this.registryPreview || !r?.online || r.state_age_s == null || r.state_age_s > 2 || st.localization_status !== 'CONFIRMED' || st.map_verified !== true || !st.map_version_id || st.localization_map_version_id !== st.map_version_id || p?.status !== 'OK' || p.frame_id !== 'map' || !Number.isFinite(p.x) || !Number.isFinite(p.y)) return;
      this.autoFit = false; this.view.x = p.x; this.view.y = p.y; this.view.panX = 0; this.view.panY = 0; this.view.zoom = Math.max(this.view.zoom, 60); this.request();
    }

    fit() {
      this.autoFit = true;
      const m = this.registryPreview && this.registryPreview.meta
        ? this.registryPreview.meta
        : (this.robot && this.robot.map_meta ? this.robot.map_meta : null);
      if (!m) return;
      const pts = [
        [0, 0],
        [m.width, 0],
        [0, m.height],
        [m.width, m.height],
      ].map(([u, v]) => H.gridToWorld(m, u, v));
      const xs = pts.map((p) => p.x);
      const ys = pts.map((p) => p.y);
      const minX = Math.min(...xs), maxX = Math.max(...xs);
      const minY = Math.min(...ys), maxY = Math.max(...ys);
      this.view.x = (minX + maxX) / 2;
      this.view.y = (minY + maxY) / 2;
      this.view.panX = 0;
      this.view.panY = 0;
      const spanX = Math.max(0.5, maxX - minX);
      const spanY = Math.max(0.5, maxY - minY);
      this.view.zoom = Math.min((this.view.width - 60) / spanX, (this.view.height - 60) / spanY);
      this.request();
    }

    setMapImage(robotId, version, img) {
      this.cache[robotId] = { version, img };
      if (!this.registryPreview && this.robot && this.robot.robot_id === robotId && this.fitKey !== robotId + ':' + version) {
        this.fitKey = robotId + ':' + version;
        this.fit();
      } else {
        this.request();
      }
    }

    setRegistryPreview(preview) {
      this.registryPreview = preview || null;
      if (this.registryPreview) {
        this.previewGoal = null;
        if (this.registryPreview.meta && this.registryPreview.img && this.previewFitKey !== this.registryPreview.fitKey) {
          this.previewFitKey = this.registryPreview.fitKey;
          this.fit();
          return;
        }
      } else {
        this.previewFitKey = null;
        if (this.robot && this.robot.map_meta) {
          this.fit();
          return;
        }
      }
      this.request();
    }

    hasMapVersion(robotId, version) {
      return Boolean(this.cache[robotId] && this.cache[robotId].version === version);
    }

    update(robot, missions) {
      this.robot = robot || null;
      this.missions = missions || [];
      if (robot && robot.state && robot.state.pose) {
        const p = robot.state.pose;
        const st=robot.state;
        const trailVersion=st.map_version_id || null;
        this.trailVersions=this.trailVersions || {};
        if(this.trailVersions[robot.robot_id]!==trailVersion){this.trails[robot.robot_id]=[];this.trailVersions[robot.robot_id]=trailVersion;}
        if (robot.online && robot.state_age_s!=null && robot.state_age_s<=2 && st.map_verified===true && st.localization_status==='CONFIRMED' && trailVersion && st.localization_map_version_id===trailVersion && p.frame_id === 'map' && p.status === 'OK' && Number.isFinite(p.x) && Number.isFinite(p.y)) {
          const list = this.trails[robot.robot_id] || (this.trails[robot.robot_id] = []);
          const last = list[list.length - 1];
          if (!last || Math.hypot(p.x - last[0], p.y - last[1]) > 0.04) {
            list.push([p.x, p.y]);
            if (list.length > 600) list.shift();
          }
        }
      }
      if (!this.registryPreview && robot && robot.map_meta && this.cache[robot.robot_id] && this.fitKey !== robot.robot_id + ':' + this.cache[robot.robot_id].version) {
        this.fitKey = robot.robot_id + ':' + this.cache[robot.robot_id].version;
        this.fit();
      } else {
        this.request();
      }
    }

    setSensorData(robotId, data, costmapImg) {
      if (!robotId) return;
      const prev = this.sensors[robotId] || {};
      this.sensors[robotId] = {
        data: data || prev.data || null,
        costmapImg: costmapImg !== undefined ? costmapImg : prev.costmapImg || null,
      };
      this.request();
    }

    request() {
      if (this._pending) return;
      this._pending = true;
      requestAnimationFrame(() => {
        this._pending = false;
        this.draw();
      });
    }

    draw() {
      const c = this.ctx;
      const dpr = window.devicePixelRatio || 1;
      const v = this.view;
      c.setTransform(dpr, 0, 0, dpr, 0, 0);
      const palette = getComputedStyle(document.documentElement);
      c.fillStyle = palette.getPropertyValue('--map-canvas').trim() || '#090d12';
      c.fillRect(0, 0, v.width, v.height);

      // Subtle background grid (screen space)
      c.strokeStyle = palette.getPropertyValue('--map-grid').trim() || 'rgba(42, 51, 64, 0.45)';
      c.lineWidth = 1;
      for (let x = 0; x < v.width; x += 40) {
        c.beginPath(); c.moveTo(x, 0); c.lineTo(x, v.height); c.stroke();
      }
      for (let y = 0; y < v.height; y += 40) {
        c.beginPath(); c.moveTo(0, y); c.lineTo(v.width, y); c.stroke();
      }

      const prev = this.registryPreview;
      const r = this.robot;
      const cached = (!prev && r) ? this.cache[r.robot_id] : null;
      const meta = prev ? prev.meta : (r ? r.map_meta : null);
      const gridImg = prev ? prev.img : (cached ? cached.img : null);

      c.save();
      c.translate(v.width / 2 + v.panX, v.height / 2 + v.panY);
      c.scale(v.zoom, -v.zoom);
      c.rotate(v.rotation);
      c.translate(-v.x, -v.y);

      // Origin axes (map frame 0,0)
      c.strokeStyle = 'rgba(255, 80, 80, 0.35)';
      c.lineWidth = 1.5 / v.zoom;
      c.beginPath(); c.moveTo(0, 0); c.lineTo(0.4, 0); c.stroke();
      c.strokeStyle = 'rgba(80, 220, 120, 0.35)';
      c.beginPath(); c.moveTo(0, 0); c.lineTo(0, 0.4); c.stroke();

      // OccupancyGrid map PNG (registry preview or live robot map)
      if (this.layers.map && gridImg && meta) {
        const o = H.gridOrigin(meta);
        c.save();
        c.globalAlpha = prev ? (prev.verified ? 0.96 : 0.78) : (r && r.online ? 1.0 : 0.55);
        c.translate(o.x, o.y);
        c.rotate(o.yaw);
        c.scale(meta.resolution, -meta.resolution);
        c.translate(0, -meta.height);
        c.imageSmoothingEnabled = false;
        c.drawImage(gridImg, 0, 0, meta.width, meta.height);
        c.restore();
      }

      if (!prev) {
        const sensRec = r ? this.sensors[r.robot_id] : null;
        const sens = sensRec && sensRec.data && sensRec.data.available ? sensRec.data : null;

        // Live Costmap overlay
        if (this.layers.costmap && sens && sensRec.costmapImg && sens.costmap_meta) {
          const cm = sens.costmap_meta;
          const co = H.gridOrigin(cm);
          c.save();
          c.globalAlpha = 0.52;
          c.translate(co.x, co.y);
          c.rotate(co.yaw);
          c.scale(cm.resolution, -cm.resolution);
          c.translate(0, -cm.height);
          c.imageSmoothingEnabled = false;
          c.drawImage(sensRec.costmapImg, 0, 0, cm.width, cm.height);
          c.restore();
        }

        // Observed pose trail
        if (this.layers.trail && r && this.trails[r.robot_id] && this.trails[r.robot_id].length > 1) {
          const pts = this.trails[r.robot_id];
          c.save();
          c.strokeStyle = '#68a6e0';
          c.lineWidth = 2 / v.zoom;
          c.setLineDash([4 / v.zoom, 4 / v.zoom]);
          c.beginPath();
          pts.forEach(([x, y], i) => (i === 0 ? c.moveTo(x, y) : c.lineTo(x, y)));
          c.stroke();
          c.restore();
        }

        // Live Frontier candidates, Nav2 Global Plan, and LiDAR scan points
        if (this.layers.plan && sens) {
          if (Array.isArray(sens.frontiers) && sens.frontiers.length) {
            c.save();
            c.fillStyle = '#facc15';
            const rad = 3.2 / v.zoom;
            for (const pt of sens.frontiers) {
              if (pt && pt.length >= 2) {
                c.beginPath(); c.arc(pt[0], pt[1], rad, 0, Math.PI * 2); c.fill();
              }
            }
            c.restore();
          }
          if (Array.isArray(sens.global_plan) && sens.global_plan.length > 1) {
            c.save();
            c.strokeStyle = '#22d3ee';
            c.lineWidth = 2.6 / v.zoom;
            c.beginPath();
            sens.global_plan.forEach((pt, i) => (i === 0 ? c.moveTo(pt[0], pt[1]) : c.lineTo(pt[0], pt[1])));
            c.stroke();
            c.restore();
          }
          if (Array.isArray(sens.lidar_points) && sens.lidar_points.length) {
            c.save();
            c.fillStyle = '#f43f5e';
            const s = 3.0 / v.zoom;
            for (const pt of sens.lidar_points) {
              if (pt && pt.length >= 2) c.fillRect(pt[0] - s / 2, pt[1] - s / 2, s, s);
            }
            c.restore();
          }
        }

        // Active (non-terminal) mission goals for selected robot
        if (this.layers.goals && r) {
          for (const m of this.missions) {
            if (m.robot_id === r.robot_id && !m.terminal && m.goal && m.goal.frame_id === 'map') {
              c.save();
              c.translate(m.goal.x, m.goal.y);
              c.strokeStyle = '#34c759';
              c.lineWidth = 2.2 / v.zoom;
              c.beginPath();
              c.arc(0, 0, 9 / v.zoom, 0, Math.PI * 2);
              c.stroke();
              if (Number.isFinite(m.goal.yaw_rad)) {
                c.rotate(m.goal.yaw_rad);
                c.beginPath(); c.moveTo(0, 0); c.lineTo(14 / v.zoom, 0); c.stroke();
              }
              c.restore();
            }
          }
        }

        // Preview goal (clicked point)
        if (this.previewGoal && Number.isFinite(this.previewGoal.x) && Number.isFinite(this.previewGoal.y)) {
          c.save();
          c.translate(this.previewGoal.x, this.previewGoal.y);
          c.strokeStyle = '#ffcc00';
          c.lineWidth = 2 / v.zoom;
          c.setLineDash([3 / v.zoom, 3 / v.zoom]);
          c.beginPath();
          c.arc(0, 0, 10 / v.zoom, 0, Math.PI * 2);
          c.stroke();
          c.restore();
        }

        // Robot current pose arrow (verified localization OR live SLAM sensor TF)
        const pose = r && r.state && r.state.pose;
        const st = r?.state || {};
        const spatialPoseKnown = (r?.online && r.state_age_s != null && r.state_age_s <= 2 && st.localization_status === 'CONFIRMED' && st.map_verified === true && st.map_version_id && st.localization_map_version_id === st.map_version_id && pose?.status === 'OK');
        const liveSlamPose = (this.layers.plan && !spatialPoseKnown && sens && sens.age_s <= 5 && sens.pose && Number.isFinite(sens.pose.x) && Number.isFinite(sens.pose.y)) ? sens.pose : null;
        const drawPose = spatialPoseKnown ? pose : liveSlamPose;
        if (drawPose && Number.isFinite(drawPose.x) && Number.isFinite(drawPose.y)) {
          c.save();
          c.translate(drawPose.x, drawPose.y);
          c.rotate(drawPose.yaw_rad || 0);
          const fault = H.classifySafetyGate(r).category === 'FAULT';
          const fill = !r.online ? '#8795a6' : fault ? '#ff4d4f' : (spatialPoseKnown ? '#459dff' : '#38bdf8');
          c.fillStyle = fill;
          c.strokeStyle = '#ffffff';
          c.lineWidth = 1.6 / v.zoom;
          const sz = 12 / v.zoom;
          c.beginPath();
          c.moveTo(sz * 1.2, 0);
          c.lineTo(-sz * 0.75, sz * 0.7);
          c.lineTo(-sz * 0.4, 0);
          c.lineTo(-sz * 0.75, -sz * 0.7);
          c.closePath();
          c.fill();
          c.stroke();
          c.restore();
        }
      }

      c.restore();

      // Overlay warnings (screen space, wrapped to stay inside canvas width)
      c.font = '12px system-ui, sans-serif';
      const msgs = [];
      if (prev) {
        msgs.push({ text: '레지스트리 미리보기 — 로봇 탑재 지도 아님 (목표 클릭·전송 차단)', color: '#ffcc00' });
        if (prev.loading) {
          msgs.push({ text: `지도 불러오는 중 (${prev.mapVersionId})...`, color: '#a5d6ff' });
        } else if (prev.error) {
          msgs.push({ text: `미리보기 오류: ${prev.error}`, color: '#ff7b72' });
        } else {
          if (prev.verification_status === 'UNVERIFIED') {
            msgs.push({ text: 'UNVERIFIED — 미검증 지도 (로봇 운용 활성화 불가)', color: '#ffcc00' });
          } else if (prev.verification_status === 'REJECTED') {
            msgs.push({ text: 'REJECTED — 검증 거부된 지도 버전', color: '#ff7b72' });
          }
          if (meta) {
            const o = H.gridOrigin(meta);
            msgs.push({
              text: `${prev.mapVersionId} · ${meta.width}x${meta.height} · ${meta.resolution}m/px · origin=(${o.x.toFixed(2)}, ${o.y.toFixed(2)}, yaw=${o.yaw.toFixed(3)})`,
              color: '#a5d6ff',
            });
          }
        }
      } else {
        const st = (r && r.state) || {};
        const mStat = (st.map && st.map.status) || '';
        const mapMismatch = Boolean(
          r && (
            (st.map_version_id && st.localization_map_version_id && st.map_version_id !== st.localization_map_version_id) ||
            st.map_verified === false
          )
        );
        if (!r) {
          msgs.push({ text: '등록된 로봇이 없습니다.', color: '#8b98a5' });
        } else if (!cached || !meta) {
          msgs.push({ text: '지도 수신 대기 (NO_MAP) — 로봇 Agent가 지도를 업로드하면 표시됩니다.', color: '#ffcc00' });
        } else {
          if (!r.online) msgs.push({ text: 'OFFLINE — 과거 수신 지도 스냅샷 (실시간 아님)', color: '#ff7b72' });
          if (mStat === 'STALE') msgs.push({ text: 'STALE — 오래된 지도 스냅샷 (갱신 지연)', color: '#ffcc00' });
          if (mapMismatch) {
            msgs.push({
              text: `지도 버전 불일치/미검증 (map=${st.map_version_id || 'NONE'} · loc=${st.localization_map_version_id || 'NONE'} · verified=${st.map_verified})`,
              color: '#ff9b71',
            });
          }
          const pose = r && r.state && r.state.pose;
          if (pose && pose.frame_id && pose.frame_id !== 'map') {
            msgs.push({ text: '위치 프레임 ' + pose.frame_id + ' (map 아님): 지도에 표시하지 않음', color: '#ffcc00' });
          } else if (!pose || pose.status === 'NO_DATA') {
            msgs.push({ text: '로봇 위치 NO_DATA — map->base_footprint TF 미수신', color: '#8b98a5' });
          }
        }
      }

      let curY = 12;
      const maxW = Math.max(120, v.width - 28);
      for (const item of msgs) {
        const lines = [];
        let line = '';
        for (const ch of item.text) {
          if (c.measureText(line + ch).width > maxW && line) {
            lines.push(line);
            line = ch;
          } else {
            line += ch;
          }
        }
        if (line) lines.push(line);
        const boxH = lines.length * 16 + 8;
        c.fillStyle = 'rgba(9, 13, 18, 0.84)';
        c.fillRect(10, curY, Math.min(maxW + 12, v.width - 20), boxH);
        c.fillStyle = item.color;
        lines.forEach((ln, idx) => c.fillText(ln, 16, curY + 16 + idx * 16));
        curY += boxH + 6;
      }

      if (this.opts.onScale) {
        this.opts.onScale((100 / v.zoom).toFixed(2) + ' m / 100 px');
      }
    }
  }

  root.FleetMapView = FleetMapView;
})(typeof globalThis !== 'undefined' ? globalThis : this);
