"""Saved-map navigation (NAVIGATION mode): Map Registry selection -> Agent map_server + AMCL -> goal validation.

One record per robot (meta `navigation:<robot>`):
  {request_id, map_version_id, yaml_sha256, pgm_sha256, verification_status, map_verified, map_meta,
   initial_pose: {x, y, yaw, seq} | None, loaded_by, epoch}
* LOAD needs the current Lease, operating mode NAVIGATION (ACTIVE), a registered, non-quarantined version whose bundle
  passes `verify_bundle_integrity` on disk. UNVERIFIED versions may be loaded (localization tests), but the Safety
  Context then reports map_verified=false and NO mission is admitted; nothing is promoted here.
* INITIAL_POSE must lie on a free cell with footprint clearance on the loaded raster.
* Goals (mission creation and the read-only goal check) are validated on the same raster: inside the map, FREE (not
  OCCUPIED, not UNKNOWN) with GOAL_CLEARANCE_M clearance, and the Agent must report localization CONFIRMED on this
  exact request (map version, request id) before a mission is accepted.
* The record is delivered to the Agent only while NAVIGATION is active (or while a NAVIGATION mission is being
  stopped); leaving NAVIGATION clears it, so re-entering requires a new LOAD and initial pose.
"""
import base64, json, math, time

GOAL_CLEARANCE_M = 0.20          # local costmap robot_radius 0.18 m (integrated_nav2_lidar_only_slow_params.yaml) + 2 cm
RECORD = 'navigation:'


class NavigationMixin:
    def _nav_load(self, robot_id):
        row = self.db.one('SELECT v FROM meta WHERE k=?', (RECORD + robot_id,))
        return json.loads(row['v']) if row else None

    def _nav_save(self, robot_id, rec):
        with self.db.transaction():
            if rec is None:
                self.db.x('DELETE FROM meta WHERE k=?', (RECORD + robot_id,))
            else:
                self.db.x('INSERT OR REPLACE INTO meta(k,v) VALUES(?,?)', (RECORD + robot_id, json.dumps(rec)))

    def navigation_clear(self, robot_id, why):
        if self._nav_load(robot_id) is not None:
            self._nav_save(robot_id, None)
            self.db.event('navigation_cleared', f'saved-map navigation cleared: {why}', robot_id)

    def navigation_view(self, robot_id):
        rec = self._nav_load(robot_id)
        loc = (((self.robots.get(robot_id) or {}).get('state') or {}).get('localization')) or {}
        if rec:
            rec = {k: v for k, v in rec.items() if k != 'map_meta'} | {'map_meta': rec.get('map_meta')}
        return {'robot_id': robot_id, 'request': rec, 'localization': loc,
                'localized': self._nav_localized(robot_id, rec) if rec else False}

    def navigation_for_agent(self, robot_id):
        """Desired localization state for the Agent (None outside NAVIGATION or on contract mismatch)."""
        if not self.agent_contract_ok(robot_id):
            return None
        rec = self.mode_view(robot_id, advance=False)
        if rec is None:
            return None
        t = rec.get('transition') or {}
        active = rec['mode'] == 'NAVIGATION' or (rec['mode'] == 'TRANSITIONING' and t.get('from') == 'NAVIGATION'
                                                 and t.get('state') == 'STOPPING_PREVIOUS')
        nav = self._nav_load(robot_id) if active else None
        if not nav:
            return None
        return {k: nav[k] for k in ('request_id', 'map_version_id', 'yaml_sha256', 'pgm_sha256', 'map_verified', 'initial_pose')}

    def _nav_localized(self, robot_id, rec):
        loc = (((self.robots.get(robot_id) or {}).get('state') or {}).get('localization')) or {}
        pose = rec.get('initial_pose') or {}
        return (loc.get('status') == 'CONFIRMED' and loc.get('request_id') == rec['request_id']
                and loc.get('map_version_id') == rec['map_version_id'] and loc.get('initial_pose_seq') == pose.get('seq'))

    # ------------------------------------------------------------ raster
    def _nav_raster(self, rec):
        cache = getattr(self, '_nav_raster_cache', None)
        if cache and cache[0] == (rec['map_version_id'], rec['pgm_sha256']):
            return cache[1]
        from .map_registry import parse_pgm_p5, parse_ros_map_yaml
        reg = self.map_registry
        meta = parse_ros_map_yaml(reg.read_bundle_file(rec['map_version_id'], 'map.yaml'))
        pgm = parse_pgm_p5(reg.read_bundle_file(rec['map_version_id'], 'map.pgm'))
        raster = {'w': pgm['width'], 'h': pgm['height'], 'canon': pgm['canonical_bytes'], 'res': float(meta['resolution']),
                  'origin': [float(v) for v in meta['origin']]}
        self._nav_raster_cache = ((rec['map_version_id'], rec['pgm_sha256']), raster)
        return raster

    def check_point(self, rec, x, y, clearance=GOAL_CLEARANCE_M):
        """(ok, reason, detail) for a map-frame point on the loaded raster (canonical FREE/OCCUPIED/UNKNOWN cells)."""
        from .map_registry import CANON_FREE, CANON_OCCUPIED
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in (x, y)):
            return False, 'NOT_FINITE', {}
        r = self._nav_raster(rec)
        ox, oy, oyaw = r['origin'][0], r['origin'][1], (r['origin'][2] if len(r['origin']) > 2 else 0.0)
        dx, dy = x - ox, y - oy
        lx, ly = math.cos(oyaw) * dx + math.sin(oyaw) * dy, -math.sin(oyaw) * dx + math.cos(oyaw) * dy
        cf, rf = lx / r['res'], ly / r['res']
        if not (0 <= cf < r['w'] and 0 <= rf < r['h']):
            return False, 'OUTSIDE_MAP', {}
        col, row = int(math.floor(cf)), r['h'] - 1 - int(math.floor(rf))
        center = r['canon'][row * r['w'] + col]
        detail = {'cell': [col, row]}
        if center == CANON_OCCUPIED:
            return False, 'OCCUPIED', detail
        if center != CANON_FREE:
            return False, 'UNKNOWN_AREA', detail
        n = int(math.ceil(clearance / r['res']))
        for dr in range(-n, n + 1):
            for dc in range(-n, n + 1):
                if (dr * dr + dc * dc) * r['res'] ** 2 > clearance ** 2:
                    continue
                rr, cc = row + dr, col + dc
                if not (0 <= rr < r['h'] and 0 <= cc < r['w']):
                    return False, 'FOOTPRINT_OUTSIDE_MAP', detail
                v = r['canon'][rr * r['w'] + cc]
                if v != CANON_FREE:
                    return False, 'FOOTPRINT_OCCUPIED' if v == CANON_OCCUPIED else 'FOOTPRINT_UNKNOWN', dict(detail, blocked_cell=[cc, rr])
        return True, None, detail

    def goal_check(self, robot_id, x, y):
        rec = self._nav_load(robot_id)
        if not rec:
            return {'ok': False, 'reason': 'NO_MAP_LOADED'}
        ok, why, d = self.check_point(rec, x, y)
        return {'ok': ok, 'reason': why, 'map_version_id': rec['map_version_id'], 'localized': self._nav_localized(robot_id, rec),
                'clearance_m': GOAL_CLEARANCE_M, **d}

    def require_valid_goal(self, robot_id, x, y):
        """Mission admission on a saved map: localization confirmed on this request and a valid goal cell."""
        from .service import ApiError
        rec = self._nav_load(robot_id)
        if rec is None:
            if self.mode_view(robot_id, advance=False) is not None:
                raise ApiError(409, 'no_map_loaded', 'load a registered map and set the initial pose first')
            return
        if not self._nav_localized(robot_id, rec):
            raise ApiError(409, 'localization_not_confirmed', 'localization on the loaded map is not CONFIRMED')
        ok, why, _ = self.check_point(rec, x, y)
        if not ok:
            raise ApiError(409, 'goal_invalid', f'goal rejected on {rec["map_version_id"]}: {why}')

    # ------------------------------------------------------------ operator requests
    def request_navigation(self, robot_id, actor, body):
        from .service import ApiError
        if not isinstance(body, dict):
            raise ApiError(400, 'bad_request', 'JSON object required')
        with self.lock:
            if robot_id not in self.robots:
                raise ApiError(404, 'unknown_robot', 'robot not registered')
            lease = self.lease_view(robot_id)
            if not lease or lease['owner'] != actor or type(body.get('epoch')) is not int or lease['epoch'] != body['epoch']:
                raise ApiError(409, 'lease_required', 'current owner and epoch required')
            self.require_mode(robot_id, 'NAVIGATION', 'mode_not_navigation', motion=False)      # LOAD / INITIAL_POSE / UNLOAD move nothing: the mode record is enough
            if (self.robots[robot_id].get('agent') or {}).get('navigation_localization') is not True:
                raise ApiError(409, 'localization_unavailable', 'agent has no saved-map localization worker')
            action = body.get('action')
            rec = self._nav_load(robot_id)
            if action == 'LOAD':
                self.require_agent_contract(robot_id, 'navigation LOAD')
                return self._nav_request_load(robot_id, actor, body, lease)
            if action == 'INITIAL_POSE':
                self.require_agent_contract(robot_id, 'navigation INITIAL_POSE')
                if not rec:
                    raise ApiError(409, 'no_map_loaded', 'load a map first')
                try:
                    x, y, yaw = (float(body[k]) for k in ('x', 'y', 'yaw'))
                except (KeyError, TypeError, ValueError):
                    raise ApiError(400, 'bad_pose', 'numeric x, y, yaw required')
                ok, why, _ = self.check_point(rec, x, y, clearance=0.0)
                if not ok or not math.isfinite(yaw):
                    raise ApiError(409, 'pose_invalid', f'initial pose rejected: {why or "yaw"}')
                seq = self._nav_counter('initial_pose_seq:' + robot_id)
                rec['initial_pose'] = {'x': x, 'y': y, 'yaw': yaw, 'seq': seq, 'by': actor, 'at': time.time()}
                self._nav_save(robot_id, rec)
                self.db.event('navigation_initial_pose', f'{actor}: initial pose ({x:.2f}, {y:.2f}, {yaw:.2f}) seq {seq}', robot_id)
                return self.navigation_view(robot_id)
            if action == 'UNLOAD':
                self.navigation_clear(robot_id, f'unloaded by {actor}')
                return self.navigation_view(robot_id)
            raise ApiError(400, 'bad_action', 'action must be LOAD, INITIAL_POSE or UNLOAD')

    def _nav_counter(self, key):
        row = self.db.one('SELECT v FROM meta WHERE k=?', (key,))
        n = (int(row['v']) if row else 0) + 1
        self.db.x('INSERT OR REPLACE INTO meta(k,v) VALUES(?,?)', (key, str(n)))
        return n

    def _nav_request_load(self, robot_id, actor, body, lease):
        from .service import ApiError
        reg = self.map_registry
        mvid = body.get('map_version_id')
        if reg is None or not isinstance(mvid, str):
            raise ApiError(400, 'map_version_required', 'map_version_id of a registered map is required')
        try:
            mv = reg.get_map_version(mvid)
        except Exception:
            raise ApiError(404, 'map_not_found', 'map version is not registered')
        expected = body.get('expected_content_sha256')
        if expected is not None and expected != mv.get('content_sha256') and expected != (mv.get('sha256') or {}).get('content_sha256'):
            raise ApiError(409, 'map_version_mismatch', 'the registered version differs from the one shown to the operator')
        if (mv.get('quarantine') or {}).get('is_quarantined') or (mv.get('quarantine') or {}).get('status') == 'ACTIVE':
            raise ApiError(409, 'map_quarantined', 'map version is quarantined')
        integrity = reg.verify_bundle_integrity(mvid)
        if not integrity.get('ok'):
            self.db.event('navigation_load_refused', f'{mvid}: bundle integrity failed', robot_id, level='warning', data=integrity)
            raise ApiError(409, 'map_integrity_failed', f"bundle integrity check failed: {integrity.get('reason')}")
        sha = mv.get('sha256') or {}
        status = mv.get('verification_status')
        rec = {'request_id': self._nav_counter('navigation_request_seq:' + robot_id), 'map_version_id': mvid,
               'yaml_sha256': sha.get('yaml_sha256') or mv.get('yaml_sha256'), 'pgm_sha256': sha.get('pgm_sha256') or mv.get('pgm_sha256'),
               'verification_status': status, 'map_verified': status == 'VERIFIED', 'initial_pose': None,
               'loaded_by': actor, 'epoch': lease['epoch'], 'at': time.time()}
        if not rec['yaml_sha256'] or not rec['pgm_sha256']:
            raise ApiError(409, 'map_integrity_failed', 'registry record lacks file checksums')
        try:
            raster = self._nav_raster(rec)
        except Exception as exc:
            raise ApiError(409, 'map_integrity_failed', f'map files unreadable: {type(exc).__name__}')
        rec['map_meta'] = {'width': raster['w'], 'height': raster['h'], 'resolution': raster['res'], 'origin': raster['origin']}
        self._nav_save(robot_id, rec)
        self.db.event('navigation_load', f'{actor}: load {mvid} ({status}) request {rec["request_id"]}', robot_id)
        return self.navigation_view(robot_id)

    def agent_map_bundle(self, robot_id, boot, body):
        """Agent download of the CURRENTLY requested map version only (checksums verified by the Registry on read)."""
        from .service import ApiError
        with self.lock:
            self.check_session(robot_id, boot)
            want = self.navigation_for_agent(robot_id)
            if not want or body.get('map_version_id') != want['map_version_id'] or body.get('file') not in ('map.yaml', 'map.pgm'):
                raise ApiError(409, 'not_requested', 'only the requested map version may be downloaded')
            data = self.map_registry.read_bundle_file(want['map_version_id'], body['file'])
            return {'map_version_id': want['map_version_id'], 'file': body['file'], 'data_b64': base64.b64encode(data).decode('ascii')}
