#!/usr/bin/env python3
"""
render_complex_indoor_videos.py:
Generates auxiliary log-reconstructed 1080p split-screen comparison videos (1.0x real-time and 2.0x summary)
for Stages A through F of the Complex Indoor Environment (16.0m x 12.0m).

Visual Requirements Enforced:
  - Top Badge: '시뮬레이션 · 로그 재구성 영상' + Explicit Playback Speed ('1.0x (실시간)' or '2.0x (요약본)')
  - Left Panel ('Gazebo 상단 시점'): Overhead camera frames (/demo/overhead) cropped to 16m x 12m
  - Right Panel ('로그 기반 시각화'): Static map + live /global_costmap/costmap + /scan LiDAR hits +
    Start (S) & Goal (G) + Robot footprint & heading arrow +
    Initial Global Path (Blue Dashed Line, kept visible until the end) +
    Current Global Path (Orange Solid Line) +
    Actual Driven Trajectory (Green Solid Line) +
    Unmapped Static Obstacles & Moving Obstacles (from obstacle_pose_history.jsonl)
"""

import os
import sys
import json
import math
import shutil
import pathlib
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
Gst.init(None)

BASE_DIR = pathlib.Path(__file__).resolve().parents[2]
ENV_DIR = BASE_DIR / 'output/simulation/complex_indoor_env'
RUNS_DIR = BASE_DIR / 'output/simulation/2026-10-24_complex_indoor_runs'
PUBLIC_VIDEO_DIR = BASE_DIR / 'output/video'
PUBLIC_VIDEO_DIR.mkdir(parents=True, exist_ok=True)
MAP_PGM = ENV_DIR / 'complex_indoor_map.pgm'

FONT_BOLD_PATH = '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'
FONT_MED_PATH = '/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc'


def load_font(size, bold=True):
    path = FONT_BOLD_PATH if bold else FONT_MED_PATH
    if not os.path.exists(path):
        path = FONT_BOLD_PATH
    return ImageFont.truetype(path, size=size)


FONTS = {
    'title': load_font(22, bold=True),
    'badge': load_font(17, bold=True),
    'clock': load_font(19, bold=True),
    'panel_hdr': load_font(19, bold=True),
    'panel_sub': load_font(14, bold=False),
    'map_label': load_font(15, bold=True),
    'map_small': load_font(12, bold=True),
    'sg_marker': load_font(16, bold=True),
    'legend': load_font(15, bold=True),
    'sub_main': load_font(21, bold=True),
    'sub_detail': load_font(16, bold=False),
}


def draw_dashed_polyline(draw, pts, color, width=5, dash_len=14, gap_len=10, halo_color=None, halo_width=8):
    if len(pts) < 2:
        return
    segs = []
    for i in range(len(pts) - 1):
        x0, y0 = pts[i]
        x1, y1 = pts[i + 1]
        L = math.hypot(x1 - x0, y1 - y0)
        if L > 1e-4:
            segs.append((x0, y0, x1, y1, L))

    cycle = dash_len + gap_len
    dist_acc = 0.0
    dash_segments = []

    for x0, y0, x1, y1, L in segs:
        ux, uy = (x1 - x0) / L, (y1 - y0) / L
        cur = 0.0
        while cur < L:
            phase = (dist_acc + cur) % cycle
            if phase < dash_len:
                rem_dash = dash_len - phase
                step = min(rem_dash, L - cur)
                sx0, sy0 = x0 + ux * cur, y0 + uy * cur
                sx1, sy1 = x0 + ux * (cur + step), y0 + uy * (cur + step)
                dash_segments.append(((sx0, sy0), (sx1, sy1)))
                cur += step
            else:
                rem_gap = cycle - phase
                step = min(rem_gap, L - cur)
                cur += step
        dist_acc += L

    if halo_color is not None:
        for p0, p1 in dash_segments:
            draw.line([p0, p1], fill=halo_color, width=halo_width)
    for p0, p1 in dash_segments:
        draw.line([p0, p1], fill=color, width=width)


def draw_solid_polyline(draw, pts, color, width=5, halo_color=None, halo_width=8):
    if len(pts) < 2:
        return
    flat = [(float(p[0]), float(p[1])) for p in pts]
    if halo_color is not None:
        draw.line(flat, fill=halo_color, width=halo_width, joint='curve')
    draw.line(flat, fill=color, width=width, joint='curve')


def draw_robot_pose(draw, px, py, yaw, scale_px_per_m, rw=0.22, rl=0.32):
    vis_l = rl * 1.45 * scale_px_per_m
    vis_w = rw * 1.45 * scale_px_per_m
    cos_t = math.cos(yaw)
    sin_t = math.sin(yaw)

    corners = []
    for dx, dy in [(-vis_l / 2, -vis_w / 2), (vis_l / 2, -vis_w / 2), (vis_l / 2, vis_w / 2), (-vis_l / 2, vis_w / 2)]:
        wx = dx * cos_t - dy * sin_t
        wy = dx * sin_t + dy * cos_t
        corners.append((px + wx, py - wy))

    draw.polygon(corners, fill=(0, 229, 255), outline=(10, 20, 40))
    draw.line(corners + [corners[0]], fill=(10, 20, 40), width=3)

    arrow_len = 0.55 * scale_px_per_m
    ax = px + arrow_len * cos_t
    ay = py - arrow_len * sin_t
    draw.line([(px, py), (ax, ay)], fill=(10, 20, 40), width=6)
    draw.line([(px, py), (ax, ay)], fill=(255, 234, 0), width=3)

    head_len = 11.0
    head_ang = 0.55
    p_left = (ax - head_len * math.cos(yaw - head_ang), ay + head_len * math.sin(yaw - head_ang))
    p_right = (ax - head_len * math.cos(yaw + head_ang), ay + head_len * math.sin(yaw + head_ang))
    draw.polygon([(ax, ay), p_left, p_right], fill=(255, 234, 0), outline=(10, 20, 40))


def build_base_map_canvas(map_pgm_path, target_w=888, target_h=666):
    """Pre-renders the 16m x 12m static map (320x240) at target_w x target_h (4:3 aspect ratio)."""
    im_pgm = Image.open(map_pgm_path)
    grid = np.array(im_pgm.getdata(), dtype=np.uint8).reshape(im_pgm.size[1], im_pgm.size[0])
    H, W = grid.shape  # 240 x 320

    rgb = np.zeros((H, W, 3), dtype=np.uint8)
    free_mask = (grid > 200)
    wall_mask = (grid == 0)
    unk_mask = (~free_mask) & (~wall_mask)

    rgb[free_mask] = [240, 244, 249]
    rgb[unk_mask] = [200, 208, 218]
    rgb[wall_mask] = [45, 55, 72]

    pil_map = Image.fromarray(rgb, mode='RGB').resize((target_w, target_h), resample=Image.NEAREST)
    draw = ImageDraw.Draw(pil_map)

    ox, oy = -8.0, -6.0
    sx = target_w / 16.0
    sy = target_h / 12.0
    for xm in range(-7, 8):
        px = int(round((xm - ox) * sx))
        draw.line([(px, 0), (px, target_h)], fill=(218, 225, 235), width=1)
        if xm % 2 == 0:
            draw.text((px + 3, target_h - 17), f"{xm}m", fill=(130, 145, 165), font=FONTS['map_small'])
    for ym in range(-5, 6):
        py = int(round(target_h - (ym - oy) * sy))
        draw.line([(0, py), (target_w, py)], fill=(218, 225, 235), width=1)
        if ym % 2 == 0:
            draw.text((5, py - 15), f"{ym}m", fill=(130, 145, 165), font=FONTS['map_small'])

    return pil_map, grid


def render_costmap_overlay(base_map_pil, static_grid, costmap_grid, target_w=888, target_h=666):
    if costmap_grid is None:
        return base_map_pil.copy()

    cm_flipped = np.flipud(costmap_grid)
    H, W = cm_flipped.shape

    base_arr = np.array(base_map_pil.getdata(), dtype=np.uint8).reshape(target_h, target_w, 3)
    r_idx = (np.arange(target_h) * H // target_h).clip(0, H - 1)
    c_idx = (np.arange(target_w) * W // target_w).clip(0, W - 1)
    cm_up = cm_flipped[r_idx[:, None], c_idx[None, :]]
    st_up = static_grid[r_idx[:, None], c_idx[None, :]]

    static_free = (st_up > 200)

    inf_mask = static_free & (cm_up >= 10) & (cm_up < 99)
    base_arr[inf_mask] = (base_arr[inf_mask].astype(np.float32) * 0.55 + np.array([155, 205, 255], dtype=np.float32) * 0.45).astype(np.uint8)

    ins_mask = static_free & (cm_up == 99)
    base_arr[ins_mask] = (base_arr[ins_mask].astype(np.float32) * 0.35 + np.array([245, 158, 11], dtype=np.float32) * 0.65).astype(np.uint8)

    new_lethal_mask = static_free & (cm_up == 100)
    base_arr[new_lethal_mask] = [225, 29, 72]

    wall_mask = (st_up == 0)
    base_arr[wall_mask] = [45, 55, 72]

    return Image.fromarray(base_arr, mode='RGB')


def get_gazebo_cropped_panel(frame_path, target_w=888, target_h=666):
    """
    Crops the 1280x960 Gazebo overhead image (already North-Up +Y, East-Right +X)
    around the 16.0m x 12.0m arena [110..1170, 82..878].
    """
    if not frame_path.exists():
        return Image.new('RGB', (target_w, target_h), (166, 166, 166))
    im = Image.open(frame_path).convert('RGB')
    crop_im = im.crop((110, 82, 1170, 878))
    return crop_im.resize((target_w, target_h), resample=Image.BILINEAR)


def world_to_gazebo_panel_px(wx, wy, target_w=888, target_h=666):
    px = ((wx - (-8.0)) / 16.0) * target_w
    py = target_h - ((wy - (-6.0)) / 12.0) * target_h
    return int(round(px)), int(round(py))


def world_to_map_panel_px(wx, wy, target_w=888, target_h=666):
    px = ((wx - (-8.0)) / 16.0) * target_w
    py = target_h - ((wy - (-6.0)) / 12.0) * target_h
    return px, py


def render_stage_log_videos(run_dir: pathlib.Path):
    summary_path = run_dir / 'summary.json'
    events_path = run_dir / 'events.json'
    scen_path = run_dir / 'obstacle_scenario.json'
    if not summary_path.exists() or not events_path.exists():
        print(f"[Skip] {run_dir} missing summary.json or events.json")
        return

    summary = json.loads(summary_path.read_text())
    events = json.loads(events_path.read_text())
    scen = json.loads(scen_path.read_text()) if scen_path.exists() else {}
    stage_id = events.get('stage_id', scen.get('stage_id', 'X'))
    stage_title = scen.get('title', f'Stage {stage_id}')

    with open(run_dir / 'telemetry.jsonl') as f:
        telemetry = [json.loads(line) for line in f if line.strip()]
    with open(run_dir / 'plan_history.jsonl') as f:
        plans = [json.loads(line) for line in f if line.strip()]
    with open(run_dir / 'scan_history.jsonl') as f:
        scans = [json.loads(line) for line in f if line.strip()]
    with open(run_dir / 'gt_pose_history.jsonl') as f:
        gt_poses = [json.loads(line) for line in f if line.strip()]

    obs_poses = []
    obs_hist_path = run_dir / 'obstacle_pose_history.jsonl'
    if obs_hist_path.exists():
        with open(obs_hist_path) as f:
            obs_poses = [json.loads(line) for line in f if line.strip()]

    cm_archive = np.load(run_dir / 'global_costmap_snapshots.npz')
    cm_times = cm_archive['sim_times']
    cm_grids = cm_archive['grids']

    start_pose = summary['initial_pose']
    goal_pose = summary['goal_pose']
    nav_start_sim = events.get('nav_start_sim_time') or (telemetry[0]['sim_time'] if telemetry else 0.0)
    nav_end_sim = events.get('action_end_sim_time') or (telemetry[-1]['sim_time'] if telemetry else nav_start_sim + 10.0)

    initial_plan = plans[0] if plans else None
    PANEL_W, PANEL_H = 888, 666
    base_map_pil, static_grid = build_base_map_canvas(MAP_PGM, PANEL_W, PANEL_H)
    scale_px_per_m = PANEL_W / 16.0  # 55.5 px/m

    active_telem = [t for t in telemetry if (nav_start_sim - 0.5) <= t['sim_time'] <= (nav_end_sim + 0.5)]
    if not active_telem:
        active_telem = telemetry

    def build_synced_telem_at(target_sim_t: float) -> dict:
        best_t = active_telem[0]
        for pt in active_telem:
            if pt['sim_time'] <= target_sim_t + 0.05:
                best_t = pt
            else:
                break
        rec = dict(best_t)
        rec['sim_time'] = float(target_sim_t)
        best_gt = None
        for g in gt_poses:
            if g['recv_sim_time'] <= target_sim_t + 0.02:
                best_gt = g
            else:
                break
        if best_gt is not None:
            rec['truth'] = [best_gt['x'], best_gt['y'], best_gt['yaw']]
        return rec

    def render_single_composite(telem_rec, speed_label="1.0x (실시간)"):
        sim_t = telem_rec['sim_time']
        elapsed_nav = max(0.0, min(nav_end_sim - nav_start_sim, sim_t - nav_start_sim))

        canvas = Image.new('RGB', (1920, 1080), (15, 23, 42))
        draw = ImageDraw.Draw(canvas)

        # 1. Header Bar
        draw.rectangle([(0, 0), (1920, 68)], fill=(30, 41, 59))
        draw.line([(0, 68), (1920, 68)], fill=(71, 85, 105), width=2)

        draw.rounded_rectangle([(18, 14), (295, 54)], radius=8, fill=(37, 99, 235))
        draw.text((30, 20), "시뮬레이션 · 로그 재구성 영상", fill=(255, 255, 255), font=FONTS['badge'])
        draw.text((310, 19), stage_title, fill=(248, 250, 252), font=FONTS['title'])

        clock_txt = f"주행 경과: {elapsed_nav:04.1f}s (Sim Clock: {sim_t:04.1f}s) | 배속: {speed_label}"
        draw.rounded_rectangle([(1180, 12), (1902, 56)], radius=8, fill=(15, 23, 42), outline=(100, 116, 139), width=2)
        draw.text((1198, 20), clock_txt, fill=(56, 189, 248), font=FONTS['clock'])

        # 2. Left Panel: Gazebo Overhead View (x: 24..928, y: 78..794)
        lx0, ly0 = 24, 78
        draw.rounded_rectangle([(lx0, ly0), (lx0 + PANEL_W + 16, ly0 + PANEL_H + 50)], radius=10, fill=(30, 41, 59), outline=(71, 85, 105), width=2)
        draw.rounded_rectangle([(lx0 + 12, ly0 + 8), (lx0 + 170, ly0 + 38)], radius=6, fill=(2, 132, 199))
        draw.text((lx0 + 26, ly0 + 11), "Gazebo 화면", fill=(255, 255, 255), font=FONTS['panel_hdr'])
        draw.text((lx0 + 184, ly0 + 13), "복잡 실내 공간(16m×12m) 상단 카메라 (/demo/overhead · 시점/배율 고정)", fill=(203, 213, 225), font=FONTS['panel_sub'])

        img_rel = telem_rec.get('image')
        gz_panel = get_gazebo_cropped_panel(run_dir / img_rel if img_rel else run_dir / 'nonexistent.png', PANEL_W, PANEL_H)
        gz_draw = ImageDraw.Draw(gz_panel)

        s_gx, s_gy = world_to_gazebo_panel_px(start_pose[0], start_pose[1], PANEL_W, PANEL_H)
        g_gx, g_gy = world_to_gazebo_panel_px(goal_pose[0], goal_pose[1], PANEL_W, PANEL_H)
        gz_draw.ellipse([(s_gx - 12, s_gy - 12), (s_gx + 12, s_gy + 12)], outline=(59, 130, 246), width=3)
        gz_draw.text((s_gx + 15, s_gy - 10), "S", fill=(255, 255, 255), font=FONTS['map_label'])
        gz_draw.ellipse([(g_gx - 13, g_gy - 13), (g_gx + 13, g_gy + 13)], outline=(239, 68, 68), width=3)
        gz_draw.text((g_gx + 15, g_gy - 10), "G", fill=(255, 255, 255), font=FONTS['map_label'])

        canvas.paste(gz_panel, (lx0 + 8, ly0 + 44))

        # 3. Right Panel: Log-based Visualization (x: 968..1872, y: 78..794)
        rx0, ry0 = 968, 78
        draw.rounded_rectangle([(rx0, ry0), (rx0 + PANEL_W + 16, ry0 + PANEL_H + 50)], radius=10, fill=(30, 41, 59), outline=(71, 85, 105), width=2)
        draw.rounded_rectangle([(rx0 + 12, ry0 + 8), (rx0 + 195, ry0 + 38)], radius=6, fill=(124, 58, 237))
        draw.text((rx0 + 24, ry0 + 11), "로그 기반 시각화", fill=(255, 255, 255), font=FONTS['panel_hdr'])
        draw.text((rx0 + 208, ry0 + 13), "사전 지도 · 실시간 Costmap · LiDAR(/scan) · 최초/현재 경로 · 실제 궤적", fill=(203, 213, 225), font=FONTS['panel_sub'])

        cm_idx = int(np.searchsorted(cm_times, sim_t, side='right')) - 1
        cm_grid = cm_grids[max(0, cm_idx)] if len(cm_times) > 0 else None
        map_panel = render_costmap_overlay(base_map_pil, static_grid, cm_grid, PANEL_W, PANEL_H)
        mp_draw = ImageDraw.Draw(map_panel)

        # 3a. LiDAR points
        active_scan = None
        for s in scans:
            if s['sim_time'] <= sim_t + 0.05:
                active_scan = s
            else:
                break
        if active_scan is not None:
            for spx, spy in active_scan.get('scan_points_map', []):
                u, v = world_to_map_panel_px(spx, spy, PANEL_W, PANEL_H)
                mp_draw.ellipse([(u - 2, v - 2), (u + 2, v + 2)], fill=(14, 165, 233))

        # 3b. Unmapped static obstacles
        det_events = events.get('obstacle_detections', {})
        for uo in scen.get('unmapped_obstacles', []):
            ox, oy = uo['pose'][0], uo['pose'][1]
            sx_o, sy_o = uo['size'][0], uo['size'][1]
            u0, v0 = world_to_map_panel_px(ox - sx_o / 2, oy + sy_o / 2, PANEL_W, PANEL_H)
            u1, v1 = world_to_map_panel_px(ox + sx_o / 2, oy - sy_o / 2, PANEL_W, PANEL_H)
            det_t = det_events.get(uo['name'], {}).get('sim_time')
            is_det = (det_t is not None and sim_t >= det_t)
            col = (225, 29, 72) if is_det else (244, 63, 94)
            out_col = (250, 204, 21) if is_det else (15, 23, 42)
            mp_draw.rectangle([(u0, v0), (u1, v1)], fill=col, outline=out_col, width=3)
            mp_draw.rounded_rectangle([(u1 + 4, (v0 + v1) / 2 - 10), (u1 + 114, (v0 + v1) / 2 + 10)], radius=4, fill=(136, 19, 55), outline=out_col, width=1)
            mp_draw.text((u1 + 8, (v0 + v1) / 2 - 8), uo['name'], fill=(255, 255, 255), font=FONTS['map_small'])

        # 3c. Moving obstacles at sim_t
        for mo in scen.get('moving_obstacles', []):
            mname = mo['name']
            sx_m, sy_m = mo['size'][0], mo['size'][1]
            best_mo = None
            for op in obs_poses:
                if (op.get('model_name') or op.get('name')) == mname and op['recv_sim_time'] <= sim_t + 0.05:
                    best_mo = op
            if best_mo is not None:
                mx, my = best_mo['x'], best_mo['y']
                u0, v0 = world_to_map_panel_px(mx - sx_m / 2, my + sy_m / 2, PANEL_W, PANEL_H)
                u1, v1 = world_to_map_panel_px(mx + sx_m / 2, my - sy_m / 2, PANEL_W, PANEL_H)
                mp_draw.rectangle([(u0, v0), (u1, v1)], fill=(245, 158, 11), outline=(15, 23, 42), width=3)
                mp_draw.rounded_rectangle([(u1 + 4, (v0 + v1) / 2 - 10), (u1 + 104, (v0 + v1) / 2 + 10)], radius=4, fill=(180, 83, 9), outline=(254, 240, 138), width=1)
                mp_draw.text((u1 + 8, (v0 + v1) / 2 - 8), mname, fill=(255, 255, 255), font=FONTS['map_small'])

        # 3d. Initial Global Path (Blue Dashed Line - Kept Visible!)
        if initial_plan is not None:
            init_px_pts = [world_to_map_panel_px(p[0], p[1], PANEL_W, PANEL_H) for p in initial_plan['poses']]
            draw_dashed_polyline(mp_draw, init_px_pts, color=(0, 90, 255), width=4, dash_len=12, gap_len=8, halo_color=(255, 255, 255), halo_width=7)

        # 3e. Current Global Path (Orange Solid Line)
        cur_plan = initial_plan
        for p in plans:
            if p['sim_time'] <= sim_t + 0.05:
                cur_plan = p
            else:
                break
        if cur_plan is not None:
            cur_px_pts = [world_to_map_panel_px(p[0], p[1], PANEL_W, PANEL_H) for p in cur_plan['poses']]
            draw_solid_polyline(mp_draw, cur_px_pts, color=(255, 109, 0), width=4, halo_color=(255, 255, 255), halo_width=7)
            if initial_plan is not None and cur_plan['plan_id'] != initial_plan['plan_id']:
                draw_dashed_polyline(mp_draw, init_px_pts, color=(0, 90, 255), width=4, dash_len=12, gap_len=8, halo_color=(255, 255, 255), halo_width=7)

        # 3f. Actual Driven Trajectory (Green Solid Line)
        traj_pts = [
            world_to_map_panel_px(g['x'], g['y'], PANEL_W, PANEL_H)
            for g in gt_poses
            if nav_start_sim <= g['recv_sim_time'] <= sim_t + 0.02
        ]
        if len(traj_pts) >= 2:
            draw_solid_polyline(mp_draw, traj_pts, color=(0, 210, 80), width=4, halo_color=(15, 23, 42), halo_width=7)

        # 3g. Start S & Goal G
        sx_m, sy_m = world_to_map_panel_px(start_pose[0], start_pose[1], PANEL_W, PANEL_H)
        gx_m, gy_m = world_to_map_panel_px(goal_pose[0], goal_pose[1], PANEL_W, PANEL_H)
        mp_draw.ellipse([(sx_m - 13, sy_m - 13), (sx_m + 13, sy_m + 13)], fill=(29, 78, 216), outline=(255, 255, 255), width=3)
        mp_draw.text((sx_m - 5, sy_m - 10), "S", fill=(255, 255, 255), font=FONTS['sg_marker'])
        mp_draw.ellipse([(gx_m - 14, gy_m - 14), (gx_m + 14, gy_m + 14)], fill=(220, 38, 38), outline=(255, 255, 255), width=3)
        mp_draw.text((gx_m - 6, gy_m - 10), "G", fill=(255, 255, 255), font=FONTS['sg_marker'])

        # 3h. Robot Pose
        if telem_rec.get('truth'):
            rx, ry, ryaw = telem_rec['truth'][0], telem_rec['truth'][1], telem_rec['truth'][-1]
            rpx, rpy = world_to_map_panel_px(rx, ry, PANEL_W, PANEL_H)
            draw_robot_pose(mp_draw, rpx, rpy, ryaw, scale_px_per_m)

        canvas.paste(map_panel, (rx0 + 8, ry0 + 44))

        # 4. Bottom Legend Row (y: 804..856)
        draw.rounded_rectangle([(24, 804), (1896, 856)], radius=8, fill=(30, 41, 59), outline=(71, 85, 105), width=2)
        draw.text((40, 818), "범례:", fill=(248, 250, 252), font=FONTS['legend'])
        draw_dashed_polyline(draw, [(110, 830), (175, 830)], color=(0, 90, 255), width=4, dash_len=10, gap_len=6, halo_color=(255, 255, 255), halo_width=6)
        draw.text((185, 818), "최초 전역 경로 (파란 점선·유지)", fill=(147, 197, 253), font=FONTS['legend'])
        draw_solid_polyline(draw, [(455, 830), (520, 830)], color=(255, 109, 0), width=4, halo_color=(255, 255, 255), halo_width=6)
        draw.text((530, 818), "현재 전역 경로 (주황 실선)", fill=(253, 186, 116), font=FONTS['legend'])
        draw_solid_polyline(draw, [(775, 830), (840, 830)], color=(0, 210, 80), width=4, halo_color=(15, 23, 42), halo_width=6)
        draw.text((850, 818), "실제 이동 궤적 (초록 실선)", fill=(134, 239, 172), font=FONTS['legend'])
        draw.rectangle([(1090, 818), (1118, 842)], fill=(225, 29, 72), outline=(250, 204, 21), width=2)
        draw.text((1128, 818), "미등록 고정 장애물", fill=(253, 164, 175), font=FONTS['legend'])
        draw.rectangle([(1310, 818), (1338, 842)], fill=(245, 158, 11), outline=(255, 255, 255), width=2)
        draw.text((1348, 818), "실시간 이동 장애물", fill=(253, 224, 71), font=FONTS['legend'])
        draw.rectangle([(1535, 820), (1565, 840)], fill=(0, 229, 255), outline=(15, 23, 42), width=2)
        draw.text((1575, 818), "로봇 현재 위치·방향 / LiDAR(/scan)", fill=(103, 232, 249), font=FONTS['legend'])

        # 5. Bottom Event Summary Box (y: 866..1062)
        rx_v, ry_v = (telem_rec['truth'][0], telem_rec['truth'][1]) if telem_rec.get('truth') else (0.0, 0.0)
        min_clr = (
            telem_rec.get('min_all_clearance')
            or summary.get('measurements', {}).get('minimum_all_objects_clearance_meters')
            or summary.get('safety_metrics', {}).get('min_clearance_meters')
            or 0.0
        )
        n_replans = len([r for r in events.get('path_replans', []) if r['sim_time'] <= sim_t])
        n_dets = len([k for k, v in det_events.items() if v['sim_time'] <= sim_t])
        n_waits = len([w for w in events.get('decelerate_or_wait_events', []) if w['start_sim_time'] <= sim_t])

        draw.rounded_rectangle([(24, 866), (1896, 1062)], radius=10, fill=(30, 41, 59), outline=(59, 130, 246), width=3)
        sub_main = (
            f"[{stage_title}]  로봇 위치: ({rx_v:.2f}, {ry_v:.2f})  |  "
            f"감지된 추가 장애물: {n_dets}개  |  누적 경로 재계획: {n_replans}회  |  "
            f"감속/대기: {n_waits}회  |  최소 안전간격: {min_clr*100:.1f}cm"
        )
        sub_detail = scen.get('description', '')
        verdict_str = f"최종 판정: {summary['verdict']['primary_status']} ({summary['verdict']['rationale']})"
        draw.text((42, 885), sub_main, fill=(255, 255, 255), font=FONTS['sub_main'])
        draw.text((42, 935), sub_detail, fill=(203, 213, 225), font=FONTS['sub_detail'])
        draw.text((42, 978), verdict_str, fill=(134, 239, 172), font=FONTS['sub_detail'])
        draw.text(
            (42, 1018),
            f"원본 실행 폴더: {run_dir.name}  |  실시간 GUI 녹화본: live_gui_stage{stage_id}_recording.mp4  |  Nav2에 GT 장애물 정보 미제공(gt_obstacle_fed_to_nav2=False)",
            fill=(148, 163, 184),
            font=FONTS['panel_sub']
        )

        return canvas

    def encode_speed_variant(out_mp4: pathlib.Path, speed_mult: float, speed_label: str, save_rep_png: pathlib.Path = None):
        fps = 10
        W_vid, H_vid = 1920, 1080
        pipeline_str = (
            f'appsrc name=src is-live=false format=time block=true '
            f'caps=video/x-raw,format=RGB,width={W_vid},height={H_vid},framerate={fps}/1 ! '
            f'queue ! videoconvert ! video/x-raw,format=I420 ! '
            f'x264enc speed-preset=ultrafast tune=zerolatency bitrate=4500 key-int-max=20 ! '
            f'video/x-h264,stream-format=avc,alignment=au ! mp4mux faststart=true ! filesink location="{out_mp4}"'
        )
        pipe = Gst.parse_launch(pipeline_str)
        src = pipe.get_by_name('src')
        pipe.set_state(Gst.State.PLAYING)

        out_idx = 0
        dt_sim = (1.0 / fps) * speed_mult
        sim_times_seq = np.arange(nav_start_sim, nav_end_sim + 0.05, dt_sim)
        mid_idx = len(sim_times_seq) // 2

        for i, sim_t_val in enumerate(sim_times_seq):
            pt = build_synced_telem_at(float(sim_t_val))
            img = render_single_composite(pt, speed_label=speed_label)
            if save_rep_png is not None and i == mid_idx:
                img.save(save_rep_png)
            raw_bytes = img.tobytes()
            buf = Gst.Buffer.new_allocate(None, len(raw_bytes), None)
            buf.fill(0, raw_bytes)
            buf.pts = Gst.util_uint64_scale(out_idx, Gst.SECOND, fps)
            buf.duration = Gst.util_uint64_scale(1, Gst.SECOND, fps)
            src.emit('push-buffer', buf)
            out_idx += 1

        src.emit('end-of-stream')
        pipe.get_bus().timed_pop_filtered(60 * Gst.SECOND, Gst.MessageType.EOS | Gst.MessageType.ERROR)
        pipe.set_state(Gst.State.NULL)
        shutil.copy2(out_mp4, PUBLIC_VIDEO_DIR / out_mp4.name)
        print(f"[ComplexRenderer] Saved {speed_label} video -> {out_mp4} ({out_mp4.stat().st_size / (1024*1024):.2f} MB)")

    mp4_1x = run_dir / f'stage{stage_id}_log_reconstructed_1.0x.mp4'
    mp4_2x = run_dir / f'stage{stage_id}_log_reconstructed_2.0x.mp4'
    rep_png = run_dir / f'stage{stage_id}_representative.png'

    encode_speed_variant(mp4_1x, speed_mult=1.0, speed_label="1.0x (실시간)", save_rep_png=rep_png)
    encode_speed_variant(mp4_2x, speed_mult=2.0, speed_label="2.0x (요약본)", save_rep_png=None)
    if rep_png.exists():
        shutil.copy2(rep_png, PUBLIC_VIDEO_DIR / rep_png.name)
    live_mp4 = run_dir / f'live_gui_stage{stage_id}_recording.mp4'
    if live_mp4.exists():
        shutil.copy2(live_mp4, PUBLIC_VIDEO_DIR / live_mp4.name)


if __name__ == '__main__':
    if len(sys.argv) > 1:
        for arg in sys.argv[1:]:
            render_stage_log_videos(pathlib.Path(arg))
    else:
        for d in sorted(RUNS_DIR.iterdir()):
            if d.is_dir() and (d / 'summary.json').exists():
                render_stage_log_videos(d)
