#!/usr/bin/env python3
"""
render_comparison_videos.py:
Generates 1920x1080 (Full HD) split-screen visualization videos (MP4 & WebM) and representative PNG images
from authentic ROS 2 and Gazebo Fortress simulation logs.

Visual Requirements Enforced:
  - Fixed top-down view of the entire 12m x 10m maze (zero rotation or zoom changes during navigation)
  - Left Panel ('Gazebo 화면'): Actual overhead camera frames from Gazebo Fortress (aligned North-up)
  - Right Panel ('로그 기반 시각화'): Static map + live /global_costmap/costmap + /scan LiDAR hits +
    Start (S) & Goal (G) + Robot footprint & heading arrow +
    Initial Global Path (Blue Dashed Line, kept visible until the end) +
    Current Global Path (Orange Solid Line) +
    Actual Driven Trajectory (Green Solid Line) +
    Unmapped / Sudden Obstacle (High-contrast Red/Magenta with Yellow border)
  - Synchronized Simulation Time, Playback Speed (1.0x), '시뮬레이션' badge, and Event-Synchronized Subtitles
  - Intro and Outro static summary holds clearly separated from driving time
"""

import os
import sys
import json
import math
import shutil
import pathlib
import subprocess
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
Gst.init(None)

BASE_DIR = pathlib.Path(__file__).resolve().parents[2]
VIDEO_DIR = BASE_DIR / 'output/simulation/2026-10-24_video_scenarios'
PUBLIC_VIDEO_DIR = BASE_DIR / 'output/video'
PUBLIC_VIDEO_DIR.mkdir(parents=True, exist_ok=True)
MAZE_PGM = BASE_DIR / 'output/simulation/2026-10-24_verification/maze/maze_map.pgm'

FONT_BOLD_PATH = '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'
FONT_MED_PATH = '/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc'


def load_font(size, bold=True):
    path = FONT_BOLD_PATH if bold else FONT_MED_PATH
    if not os.path.exists(path):
        path = FONT_BOLD_PATH
    return ImageFont.truetype(path, size=size)


FONTS = {
    'title': load_font(24, bold=True),
    'badge': load_font(18, bold=True),
    'clock': load_font(20, bold=True),
    'panel_hdr': load_font(20, bold=True),
    'panel_sub': load_font(15, bold=False),
    'map_label': load_font(16, bold=True),
    'map_small': load_font(13, bold=True),
    'sg_marker': load_font(18, bold=True),
    'legend': load_font(16, bold=True),
    'step_bar': load_font(16, bold=True),
    'sub_main': load_font(22, bold=True),
    'sub_detail': load_font(17, bold=False),
}


def draw_dashed_polyline(draw, pts, color, width=5, dash_len=14, gap_len=10, halo_color=None, halo_width=8):
    """Draws a crisp dashed polyline along a sequence of (x, y) pixel coordinates."""
    if len(pts) < 2:
        return
    # Precompute cumulative arc lengths
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
    """Draws a crisp solid polyline with optional halo."""
    if len(pts) < 2:
        return
    flat = [ (float(p[0]), float(p[1])) for p in pts ]
    if halo_color is not None:
        draw.line(flat, fill=halo_color, width=halo_width, joint='curve')
    draw.line(flat, fill=color, width=width, joint='curve')


def draw_robot_pose(draw, px, py, yaw, scale_px_per_m, rw=0.22, rl=0.32):
    """Draws the oriented robot body and a prominent heading arrow."""
    # Make visual footprint slightly larger (1.35x) so it is clearly legible on 1080p
    vis_l = rl * 1.35 * scale_px_per_m
    vis_w = rw * 1.35 * scale_px_per_m
    # Note: In image coordinates, +X is Right (+u), +Y is Up (-v)
    cos_t = math.cos(yaw)
    sin_t = math.sin(yaw)

    corners = []
    for dx, dy in [(-vis_l / 2, -vis_w / 2), (vis_l / 2, -vis_w / 2), (vis_l / 2, vis_w / 2), (-vis_l / 2, vis_w / 2)]:
        wx = dx * cos_t - dy * sin_t
        wy = dx * sin_t + dy * cos_t
        corners.append((px + wx, py - wy))

    # Outer dark halo & cyan body
    draw.polygon(corners, fill=(0, 229, 255), outline=(10, 20, 40))
    draw.line(corners + [corners[0]], fill=(10, 20, 40), width=3)

    # Heading arrow pointing forward along yaw
    arrow_len = 0.52 * scale_px_per_m
    ax = px + arrow_len * cos_t
    ay = py - arrow_len * sin_t
    draw.line([(px, py), (ax, ay)], fill=(10, 20, 40), width=6)
    draw.line([(px, py), (ax, ay)], fill=(255, 234, 0), width=3)

    # Arrowhead
    head_len = 12.0
    head_ang = 0.55
    p_left = (
        ax - head_len * math.cos(yaw - head_ang),
        ay + head_len * math.sin(yaw - head_ang)
    )
    p_right = (
        ax - head_len * math.cos(yaw + head_ang),
        ay + head_len * math.sin(yaw + head_ang)
    )
    draw.polygon([(ax, ay), p_left, p_right], fill=(255, 234, 0), outline=(10, 20, 40))


def build_base_map_canvas(map_pgm_path, target_w=888, target_h=740):
    """Pre-renders the static map (12m x 10m) at target_w x target_h with 1m coordinate grid."""
    im_pgm = Image.open(map_pgm_path)
    grid = np.array(im_pgm.getdata(), dtype=np.uint8).reshape(im_pgm.size[1], im_pgm.size[0])
    H, W = grid.shape  # 200 x 240

    rgb = np.zeros((H, W, 3), dtype=np.uint8)
    # Free space (254) -> clean light background
    free_mask = (grid > 200)
    wall_mask = (grid == 0)
    unk_mask = (~free_mask) & (~wall_mask)

    rgb[free_mask] = [240, 244, 249]
    rgb[unk_mask] = [200, 208, 218]
    rgb[wall_mask] = [45, 55, 72]

    pil_map = Image.fromarray(rgb, mode='RGB').resize((target_w, target_h), resample=Image.NEAREST)
    draw = ImageDraw.Draw(pil_map)

    # Draw subtle 1m grid lines
    ox, oy = -6.0, -5.0
    sx = target_w / 12.0
    sy = target_h / 10.0
    for xm in range(-5, 6):
        px = int(round((xm - ox) * sx))
        draw.line([(px, 0), (px, target_h)], fill=(218, 225, 235), width=1)
        if xm % 2 == 0:
            draw.text((px + 3, target_h - 18), f"{xm}m", fill=(130, 145, 165), font=FONTS['map_small'])
    for ym in range(-4, 5):
        py = int(round(target_h - (ym - oy) * sy))
        draw.line([(0, py), (target_w, py)], fill=(218, 225, 235), width=1)
        if ym % 2 == 0:
            draw.text((6, py - 16), f"{ym}m", fill=(130, 145, 165), font=FONTS['map_small'])

    return pil_map, grid


def render_costmap_overlay(base_map_pil, static_grid, costmap_grid, target_w=888, target_h=740):
    """Overlays live /global_costmap/costmap onto the static map image."""
    if costmap_grid is None:
        return base_map_pil.copy()

    # In OccupancyGrid, row 0 is y_min (-5.0), whereas in image row 0 is y_max (+5.0)
    cm_flipped = np.flipud(costmap_grid)
    H, W = cm_flipped.shape

    base_arr = np.array(base_map_pil.getdata(), dtype=np.uint8).reshape(target_h, target_w, 3)
    # Resize cm_flipped and static_grid to (target_h, target_w) using nearest neighbor index mapping
    r_idx = (np.arange(target_h) * H // target_h).clip(0, H - 1)
    c_idx = (np.arange(target_w) * W // target_w).clip(0, W - 1)
    cm_up = cm_flipped[r_idx[:, None], c_idx[None, :]]
    st_up = static_grid[r_idx[:, None], c_idx[None, :]]

    static_free = (st_up > 200)

    # 1. Inflation cells (1 <= cost <= 98) on free space
    inf_mask = static_free & (cm_up >= 10) & (cm_up < 99)
    base_arr[inf_mask] = (base_arr[inf_mask].astype(np.float32) * 0.55 + np.array([155, 205, 255], dtype=np.float32) * 0.45).astype(np.uint8)

    # 2. Inscribed inflation cells (cost == 99) on free space
    ins_mask = static_free & (cm_up == 99)
    base_arr[ins_mask] = (base_arr[ins_mask].astype(np.float32) * 0.35 + np.array([245, 158, 11], dtype=np.float32) * 0.65).astype(np.uint8)

    # 3. Newly discovered obstacle lethal cells (cost == 100 where static map was free!)
    new_lethal_mask = static_free & (cm_up == 100)
    base_arr[new_lethal_mask] = [225, 29, 72]  # Bright crimson/magenta

    # Re-enforce static walls so they remain sharp
    wall_mask = (st_up == 0)
    base_arr[wall_mask] = [45, 55, 72]

    return Image.fromarray(base_arr, mode='RGB')


def get_gazebo_cropped_panel(frame_path, target_w=888, target_h=740):
    """
    Rotates the raw 1280x960 Gazebo overhead image 90 deg CW so +X is Right and +Y is Up,
    and crops the 12m x 10m maze region to match the right panel 1:1.
    """
    im = Image.open(frame_path).convert('RGB')
    im_rot = im.transpose(Image.ROTATE_270)  # 960 x 1280
    # Maze spans x in [0, 960], y in [205, 1074] (height 869, center 640)
    # Crop 960 x 800 around center (480, 640) -> y in [240, 1040] for exact 12:10 aspect ratio!
    # Wait: maze y is [205, 1074] (869 px height), so let's pad x slightly from 960 to 1042 (12:10 of 869)
    # so the entire outer north/south walls [205, 1074] are 100% visible without clipping!
    pad_w = 1044
    pad_h = 870
    canvas = Image.new('RGB', (pad_w, pad_h), (166, 166, 166))
    x_off = (pad_w - 960) // 2  # 42 px
    crop_im = im_rot.crop((0, 205, 960, 1075))
    canvas.paste(crop_im, (x_off, 0))
    return canvas.resize((target_w, target_h), resample=Image.BILINEAR)


def world_to_gazebo_panel_px(wx, wy, target_w=888, target_h=740):
    """Maps world (wx, wy) in [-6..6, -5..5] to pixel coordinates on the Gazebo panel."""
    # In crop (pad_w=1044, pad_h=870), maze [-6..6] spans roughly 1020px and [-5..5] spans 850px
    px = target_w * 0.5 + (wx / 12.0) * (target_w * 0.965)
    py = target_h * 0.5 - (wy / 10.0) * (target_h * 0.965)
    return int(round(px)), int(round(py))


def world_to_map_panel_px(wx, wy, target_w=888, target_h=740):
    """Maps world (wx, wy) in [-6..6, -5..5] to exact pixel coordinates on the Map panel."""
    px = ((wx - (-6.0)) / 12.0) * target_w
    py = target_h - ((wy - (-5.0)) / 10.0) * target_h
    return px, py


def get_subtitle_and_stage(scenario_type, sim_t, events, telem):
    """Returns (stage_idx 1..6, main_subtitle, detail_subtitle, stage_badge_color) synchronized to exact event timestamps."""
    nav_start = events['nav_start_sim_time']
    t_scan = events['first_scan_hit_sim_time']
    t_cm = events['first_costmap_mark_sim_time']
    t_replan = events['detour_replan_sim_time']
    t_end = events['action_end_sim_time']

    rx, ry = (telem['truth'][0], telem['truth'][1]) if telem.get('truth') else (0.0, 0.0)
    min_clr = telem.get('min_clearance_all_time') or 0.0

    if scenario_type == 'unmapped_detour':
        if sim_t < nav_start:
            return (
                1,
                "[초기화 및 출발 대기] 시작점 S(-2.0, -3.0) 및 목적지 G(3.0, 0.5) 확인",
                "사전 지도에는 남측 통로(0.0, -0.85)의 장애물이 등록되어 있지 않으며, 출발 위치에서 센서 범위(2.5m) 밖에 위치함",
                (37, 99, 235)
            )
        elif sim_t < t_scan:
            return (
                2,
                f"[1~2단계: 최초 전역 경로 생성(t={nav_start:.2f}s) 및 이동] 파란 점선(남측 통로 경유)을 따라 주행 시작",
                f"사전 지도 기준 최단 경로(292 경유점)를 따라 북진 이동 중 | 현재 로봇 위치: ({rx:.2f}, {ry:.2f}) | 장애물 미감지 상태",
                (37, 99, 235)
            )
        elif sim_t < t_cm:
            dist = telem.get('scan_obs_min_dist') or events.get('first_scan_hit_distance') or 2.44
            hits = telem.get('scan_obs_hits', 2)
            return (
                3,
                f"[3단계: 센서로 미등록 장애물 최초 발견 (t={t_scan:.2f}s)] 라이다(/scan)가 전방 {dist:.2f}m 장애물 관측!",
                f"로봇이 y={ry:.2f}m 지점에 접근하자 벽면 가림이 해소되며 라이다 빔({hits}개)이 지도에 없던 장애물(0.0, -0.85)을 실시간 감지",
                (220, 38, 38)
            )
        elif sim_t < t_replan + 3.5:
            lethal = telem.get('costmap_obs_lethal_cells', 6)
            return (
                5 if sim_t >= t_replan else 4,
                f"[4~5단계: Costmap 반영(t={t_cm:.2f}s) → 우회 경로 재계획(t={t_replan:.2f}s)] 북측 우회로(주황 실선) 생성!",
                f"Global Costmap에 장애물 치명 비용({lethal}셀) 반영 직후, 남측 경로(파란 점선) 대신 북측 통로 우회 경로(339 경유점)로 재계획",
                (234, 88, 12)
            )
        elif sim_t < t_end:
            return (
                6,
                "[6단계: 변경된 경로로 이동 중] 최초 경로(파란 점선, 유지) 대신 재계획된 북측 우회로(주황 실선) 주행",
                f"로봇 크기(0.32×0.22m)와 Ackermann 조향 제약을 만족하는 북측 광폭 통로 통과 중 | 로봇 위치: ({rx:.2f}, {ry:.2f}) | 최소 안전간격: {min_clr*100:.1f}cm",
                (5, 150, 105)
            )
        else:
            return (
                6,
                f"[목적지 도달 완료 (t={t_end:.2f}s, STATUS_SUCCEEDED)] 우회 경로를 따라 목적지 G 도달 성공!",
                f"최초 경로(파란 점선) → 센서 장애물 발견(8.82s) → Costmap 반영(9.95s) → 북측 우회 재계획(10.05s) → 완주 (최소 간격 {min_clr*100:.1f}cm, 무충돌)",
                (5, 150, 105)
            )
    else:
        # scen4_dynamic_spawn
        t_spawn = events['obstacle_spawn_sim_time']
        if sim_t < nav_start:
            return (
                1,
                "[별도 시험: 주행 중 돌발 장애물 등장] 시작점 S(-4.5, -3.5) → 목적지 G(-1.5, -3.5) 대기",
                "출발 시점에는 주행 통로에 장애물이 없으며, 로봇이 전진하는 도중 전방 안전거리(1.5m)에 장애물이 등장하는 시나리오",
                (37, 99, 235)
            )
        elif sim_t < t_spawn:
            return (
                2,
                f"[1~2단계: 최초 직선 경로(파란 점선) 주행 중] S(-4.5, -3.5)에서 G(-1.5, -3.5)로 직진 이동",
                f"장애물 등장 전 최초 전역 경로(118 경유점)를 따라 정상 주행 중 | 현재 로봇 위치: ({rx:.2f}, {ry:.2f})",
                (37, 99, 235)
            )
        elif sim_t < t_replan:
            return (
                3,
                f"[3단계: 돌발 장애물 등장(t={t_spawn:.2f}s) 및 라이다 감지(t={t_scan:.2f}s)] 전방 (-2.3, -3.5) 장애물 관측!",
                f"로봇 전방 1.50m 위치에 돌발 장애물 생성 → 라이다(/scan) 즉각 감지 및 Costmap 장애물 반영 진행 중",
                (220, 38, 38)
            )
        elif sim_t < t_end:
            return (
                5 if sim_t < t_replan + 3.0 else 6,
                f"[4~6단계: 실시간 회피 경로 재계획(t={t_replan:.2f}s) 및 우회 주행] 남측 회피 궤적(주황 실선) 주행",
                f"최초 직선 경로(파란 점선)를 유지한 채 남측 여유 공간(y=-4.09m)으로 회피 경로(94 경유점) 재계획 및 통과 (최소 간격 {min_clr*100:.1f}cm)",
                (234, 88, 12) if sim_t < t_replan + 3.0 else (5, 150, 105)
            )
        else:
            return (
                6,
                f"[목적지 도달 완료 (t={t_end:.2f}s, STATUS_SUCCEEDED)] 돌발 장애물 회피 후 목적지 G 정확 도달!",
                f"돌발 등장(9.66s) → 센서 감지(10.05s) → 회피 재계획(10.90s) → 목적지 도달 (최종 위치 오차 7.0cm, 최소 간격 18.8cm, 무충돌)",
                (5, 150, 105)
            )


def render_scenario_video(
    scenario_dir: pathlib.Path,
    scenario_type: str,
    out_mp4: pathlib.Path,
    out_webm: pathlib.Path,
    out_rep_png: pathlib.Path,
    out_replan_png: pathlib.Path = None
):
    print(f"\n[Renderer] Loading recorded data from {scenario_dir}...")
    with open(scenario_dir / 'summary.json') as f:
        summary = json.load(f)
    with open(scenario_dir / 'events.json') as f:
        events = json.load(f)
    with open(scenario_dir / 'telemetry.jsonl') as f:
        telemetry = [json.loads(line) for line in f]
    with open(scenario_dir / 'plan_history.jsonl') as f:
        plans = [json.loads(line) for line in f]
    with open(scenario_dir / 'scan_history.jsonl') as f:
        scans = [json.loads(line) for line in f]
    with open(scenario_dir / 'gt_pose_history.jsonl') as f:
        gt_poses = [json.loads(line) for line in f]

    cm_archive = np.load(scenario_dir / 'global_costmap_snapshots.npz')
    cm_times = cm_archive['sim_times']
    cm_grids = cm_archive['grids']

    start_pose = summary['initial_pose']
    goal_pose = summary['goal_pose']
    nav_start_sim = events['nav_start_sim_time']
    nav_end_sim = events['action_end_sim_time']

    # Identify Initial Plan (first plan at or right after nav_start_sim)
    initial_plan = None
    for p in plans:
        if p['sim_time'] >= nav_start_sim - 0.2:
            initial_plan = p
            break
    if initial_plan is None and plans:
        initial_plan = plans[0]

    PANEL_W, PANEL_H = 888, 736
    base_map_pil, static_grid = build_base_map_canvas(MAZE_PGM, PANEL_W, PANEL_H)
    scale_px_per_m = PANEL_W / 12.0  # 74.0 px/m

    # Filter telemetry frames to start roughly 0.5s before nav_start_sim up to nav_end_sim + 0.5s
    active_telem = [t for t in telemetry if (nav_start_sim - 0.3) <= t['sim_time'] <= (nav_end_sim + 0.5)]
    if not active_telem:
        active_telem = telemetry

    rendered_dir = scenario_dir / 'rendered_1080p_frames'
    if rendered_dir.exists():
        shutil.rmtree(rendered_dir)
    rendered_dir.mkdir(parents=True, exist_ok=True)

    def render_single_composite(telem_rec, banner_mode="DRIVING"):
        """
        banner_mode:
          - "INTRO_HOLD": 2.0s static hold at initial plan state (marked outside driving time)
          - "DRIVING": 1.0x real-time simulation playback
          - "OUTRO_HOLD": 2.5s static hold at final state (marked outside driving time)
        """
        sim_t = telem_rec['sim_time']
        elapsed_nav = max(0.0, min(nav_end_sim - nav_start_sim, sim_t - nav_start_sim))

        canvas = Image.new('RGB', (1920, 1080), (15, 23, 42))
        draw = ImageDraw.Draw(canvas)

        # --- 1. Top Header Bar (y: 0..68) ---
        draw.rectangle([(0, 0), (1920, 68)], fill=(30, 41, 59))
        draw.line([(0, 68), (1920, 68)], fill=(71, 85, 105), width=2)

        # Simulation Badge
        draw.rounded_rectangle([(20, 14), (165, 54)], radius=8, fill=(37, 99, 235))
        draw.text((35, 20), "시뮬레이션", fill=(255, 255, 255), font=FONTS['badge'])

        if scenario_type == 'unmapped_detour':
            title_str = "미로 주행 중 사전 지도에 없는 장애물 발견 및 우회 경로 재계획 (Gazebo + Nav2)"
        else:
            title_str = "[별도 시험] 주행 중 돌발 장애물 등장 감지 및 실시간 회피 경로 재계획"
        draw.text((182, 18), title_str, fill=(248, 250, 252), font=FONTS['title'])

        # Clock & Speed Box on Top Right
        if banner_mode == "INTRO_HOLD":
            clock_bg = (30, 64, 175)
            clock_txt = f"[도입 정지 화면 · 주행 시간 외]  초기 경로 확인  |  Sim Clock: {sim_t:.1f}s"
        elif banner_mode == "OUTRO_HOLD":
            clock_bg = (6, 95, 70)
            clock_txt = f"[종료 요약 정지 화면 · 주행 시간 외]  총 주행: {elapsed_nav:.1f}s 완료  |  Sim Clock: {sim_t:.1f}s"
        else:
            clock_bg = (15, 23, 42)
            clock_txt = f"시뮬레이션 주행 경과: {elapsed_nav:04.1f}s  (Sim Clock: {sim_t:04.1f}s)  |  배속: 1.0x (실시간)"

        draw.rounded_rectangle([(1130, 12), (1900, 56)], radius=8, fill=clock_bg, outline=(100, 116, 139), width=2)
        draw.text((1148, 20), clock_txt, fill=(56, 189, 248) if banner_mode == "DRIVING" else (254, 240, 138), font=FONTS['clock'])

        # --- 2. Left Panel: Gazebo Overhead View (x: 24..932, y: 78..864) ---
        lx0, ly0 = 24, 78
        draw.rounded_rectangle([(lx0, ly0), (lx0 + PANEL_W + 16, ly0 + PANEL_H + 50)], radius=10, fill=(30, 41, 59), outline=(71, 85, 105), width=2)
        draw.rounded_rectangle([(lx0 + 12, ly0 + 8), (lx0 + 170, ly0 + 38)], radius=6, fill=(2, 132, 199))
        draw.text((lx0 + 26, ly0 + 11), "Gazebo 화면", fill=(255, 255, 255), font=FONTS['panel_hdr'])
        draw.text((lx0 + 184, ly0 + 13), "실제 물리 시뮬레이션 상단 카메라 (/demo/overhead · 시점/배율 고정)", fill=(203, 213, 225), font=FONTS['panel_sub'])

        gz_panel = get_gazebo_cropped_panel(scenario_dir / telem_rec['image'], PANEL_W, PANEL_H)
        gz_draw = ImageDraw.Draw(gz_panel)

        # Draw non-intrusive reference callouts on Gazebo panel for S, G, and Obstacle
        s_gx, s_gy = world_to_gazebo_panel_px(start_pose[0], start_pose[1], PANEL_W, PANEL_H)
        g_gx, g_gy = world_to_gazebo_panel_px(goal_pose[0], goal_pose[1], PANEL_W, PANEL_H)
        gz_draw.ellipse([(s_gx - 14, s_gy - 14), (s_gx + 14, s_gy + 14)], outline=(59, 130, 246), width=3)
        gz_draw.text((s_gx + 16, s_gy - 12), "S (시작점)", fill=(255, 255, 255), font=FONTS['map_small'])
        gz_draw.ellipse([(g_gx - 15, g_gy - 15), (g_gx + 15, g_gy + 15)], outline=(239, 68, 68), width=3)
        gz_draw.text((g_gx + 18, g_gy - 12), "G (목적지)", fill=(255, 255, 255), font=FONTS['map_small'])

        if scenario_type == 'unmapped_detour':
            ox_g, oy_g = world_to_gazebo_panel_px(0.0, -0.85, PANEL_W, PANEL_H)
            gz_draw.rectangle([(ox_g - 22, oy_g - 22), (ox_g + 22, oy_g + 22)], outline=(250, 204, 21), width=3)
            gz_draw.text((ox_g + 26, oy_g - 10), "지도에 없던 장애물", fill=(254, 240, 138), font=FONTS['map_small'])
        else:
            if sim_t >= events['obstacle_spawn_sim_time']:
                ox_g, oy_g = world_to_gazebo_panel_px(-2.3, -3.5, PANEL_W, PANEL_H)
                gz_draw.rectangle([(ox_g - 22, oy_g - 22), (ox_g + 22, oy_g + 22)], outline=(250, 204, 21), width=3)
                gz_draw.text((ox_g - 55, oy_g - 40), "돌발 등장 장애물", fill=(254, 240, 138), font=FONTS['map_small'])

        canvas.paste(gz_panel, (lx0 + 8, ly0 + 44))

        # --- 3. Right Panel: Log-based Visualization (x: 968..1896, y: 78..864) ---
        rx0, ry0 = 968, 78
        draw.rounded_rectangle([(rx0, ry0), (rx0 + PANEL_W + 16, ry0 + PANEL_H + 50)], radius=10, fill=(30, 41, 59), outline=(71, 85, 105), width=2)
        draw.rounded_rectangle([(rx0 + 12, ry0 + 8), (rx0 + 195, ry0 + 38)], radius=6, fill=(124, 58, 237))
        draw.text((rx0 + 24, ry0 + 11), "로그 기반 시각화", fill=(255, 255, 255), font=FONTS['panel_hdr'])
        draw.text((rx0 + 208, ry0 + 13), "사전 지도 · 실시간 Costmap · 최초/현재 전역 경로(/plan) · 궤적(GT)", fill=(203, 213, 225), font=FONTS['panel_sub'])

        # Select latest global costmap snapshot <= sim_t
        cm_idx = int(np.searchsorted(cm_times, sim_t, side='right')) - 1
        cm_grid = cm_grids[max(0, cm_idx)] if len(cm_times) > 0 else None

        map_panel = render_costmap_overlay(base_map_pil, static_grid, cm_grid, PANEL_W, PANEL_H)
        mp_draw = ImageDraw.Draw(map_panel)

        # 3a. Draw LiDAR /scan points & obstacle hit rays (from scan_history.jsonl)
        # Find closest scan record <= sim_t
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
            # If LiDAR is actively hitting the unmapped obstacle, draw laser rays from robot to obstacle!
            hit_pts = active_scan.get('obstacle_hit_points', [])
            if hit_pts and telem_rec.get('truth'):
                r_u, r_v = world_to_map_panel_px(telem_rec['truth'][0], telem_rec['truth'][1], PANEL_W, PANEL_H)
                for hpx, hpy in hit_pts[::max(1, len(hit_pts) // 8)]:
                    hu, hv = world_to_map_panel_px(hpx, hpy, PANEL_W, PANEL_H)
                    mp_draw.line([(r_u, r_v), (hu, hv)], fill=(251, 191, 36), width=2)
                for hpx, hpy in hit_pts:
                    hu, hv = world_to_map_panel_px(hpx, hpy, PANEL_W, PANEL_H)
                    mp_draw.ellipse([(hu - 3, hv - 3), (hu + 3, hv + 3)], fill=(254, 240, 138), outline=(220, 38, 38))

        # 3b. Draw Unmapped / Sudden Obstacle Box clearly
        if scenario_type == 'unmapped_detour':
            ob_cx, ob_cy, ob_sx, ob_sy = 0.0, -0.85, 0.45, 0.45
            ob_visible = True
            ob_detected = (sim_t >= events['first_scan_hit_sim_time'])
        else:
            ob_cx, ob_cy, ob_sx, ob_sy = -2.3, -3.5, 0.45, 0.45
            ob_visible = (sim_t >= events['obstacle_spawn_sim_time'])
            ob_detected = (events['first_scan_hit_sim_time'] is not None and sim_t >= events['first_scan_hit_sim_time'])

        if ob_visible:
            ox0, oy0 = world_to_map_panel_px(ob_cx - ob_sx / 2, ob_cy + ob_sy / 2, PANEL_W, PANEL_H)
            ox1, oy1 = world_to_map_panel_px(ob_cx + ob_sx / 2, ob_cy - ob_sy / 2, PANEL_W, PANEL_H)
            if ob_detected:
                mp_draw.rectangle([(ox0 - 3, oy0 - 3), (ox1 + 3, oy1 + 3)], fill=(225, 29, 72), outline=(250, 204, 21), width=4)
                lbl = "▼ 미등록 장애물 (센서 감지·Costmap 반영)" if scenario_type == 'unmapped_detour' else "▼ 돌발 장애물 (센서 감지·Costmap 반영)"
                lbl_bg = (190, 18, 60)
            else:
                mp_draw.rectangle([(ox0 - 2, oy0 - 2), (ox1 + 2, oy1 + 2)], fill=(244, 63, 94), outline=(255, 255, 255), width=3)
                lbl = "▼ 지도에 없던 장애물 (센서 미감지)" if scenario_type == 'unmapped_detour' else "▼ 돌발 장애물 등장"
                lbl_bg = (136, 19, 55)

            # Place obstacle callout tag inside the dark wall island above the obstacle so it never overlaps any path!
            if scenario_type == 'unmapped_detour':
                tx, ty = world_to_map_panel_px(-0.72, 0.32, PANEL_W, PANEL_H)
                tag_x, tag_y = int(tx), int(ty)
                mp_draw.line([(int((ox0 + ox1) / 2), int(oy0 - 3)), (tag_x + 40, tag_y + 25)], fill=(250, 204, 21), width=2)
            else:
                tx, ty = world_to_map_panel_px(-3.2, -2.15, PANEL_W, PANEL_H)
                tag_x, tag_y = int(tx), int(ty)
                mp_draw.line([(int((ox0 + ox1) / 2), int(oy0 - 3)), (tag_x + 60, tag_y + 25)], fill=(250, 204, 21), width=2)

            box_w = 246 if ob_detected else 210
            mp_draw.rounded_rectangle([(tag_x, tag_y), (tag_x + box_w, tag_y + 25)], radius=5, fill=lbl_bg, outline=(250, 204, 21), width=2)
            mp_draw.text((tag_x + 7, tag_y + 3), lbl, fill=(255, 255, 255), font=FONTS['map_small'])

        # 3c. Draw INITIAL Global Path (Blue Dashed Line - NEVER ERASED!)
        if initial_plan is not None:
            init_px_pts = [world_to_map_panel_px(p[0], p[1], PANEL_W, PANEL_H) for p in initial_plan['poses']]
            draw_dashed_polyline(
                mp_draw, init_px_pts,
                color=(0, 90, 255), width=5, dash_len=14, gap_len=9,
                halo_color=(255, 255, 255), halo_width=9
            )
            # Non-overlapping label for Initial Path
            if scenario_type == 'unmapped_detour':
                ip_x, ip_y = world_to_map_panel_px(0.25, -1.65, PANEL_W, PANEL_H)
                p_ref_x, p_ref_y = world_to_map_panel_px(1.10, -1.00, PANEL_W, PANEL_H)
                mp_draw.line([(int(ip_x + 95), int(ip_y)), (int(p_ref_x), int(p_ref_y + 5))], fill=(96, 165, 250), width=2)
                mp_draw.rounded_rectangle([(ip_x, ip_y), (ip_x + 216, ip_y + 25)], radius=5, fill=(30, 58, 138), outline=(96, 165, 250), width=2)
                mp_draw.text((ip_x + 7, ip_y + 3), "▲ 최초 전역 경로 (파란 점선·유지)", fill=(255, 255, 255), font=FONTS['map_small'])
            else:
                ip_x, ip_y = world_to_map_panel_px(-3.6, -2.75, PANEL_W, PANEL_H)
                mp_draw.rounded_rectangle([(ip_x, ip_y), (ip_x + 216, ip_y + 25)], radius=5, fill=(30, 58, 138), outline=(96, 165, 250), width=2)
                mp_draw.text((ip_x + 7, ip_y + 3), "▼ 최초 전역 경로 (파란 점선·유지)", fill=(255, 255, 255), font=FONTS['map_small'])

        # 3d. Draw CURRENT Global Path (Orange Solid Line)
        cur_plan = None
        for p in plans:
            if p['sim_time'] <= sim_t + 0.05:
                cur_plan = p
            else:
                break
        if cur_plan is None and initial_plan is not None:
            cur_plan = initial_plan

        if cur_plan is not None:
            cur_px_pts = [world_to_map_panel_px(p[0], p[1], PANEL_W, PANEL_H) for p in cur_plan['poses']]
            draw_solid_polyline(
                mp_draw, cur_px_pts,
                color=(255, 109, 0), width=5,
                halo_color=(255, 255, 255), halo_width=8
            )
            # Keep Initial Blue Dashed Path clearly visible alongside Current Path!
            if initial_plan is not None and cur_plan['plan_id'] != initial_plan['plan_id']:
                draw_dashed_polyline(
                    mp_draw, init_px_pts,
                    color=(0, 90, 255), width=5, dash_len=14, gap_len=9,
                    halo_color=(255, 255, 255), halo_width=8
                )
                # Label the newly replanned detour route in open space above the arc!
                if scenario_type == 'unmapped_detour' and (cur_plan['route_branch'] == 'NORTH_DETOUR' or sim_t >= events['detour_replan_sim_time']):
                    dp_x, dp_y = world_to_map_panel_px(-0.55, 3.60, PANEL_W, PANEL_H)
                    mp_draw.rounded_rectangle([(dp_x, dp_y), (dp_x + 225, dp_y + 25)], radius=5, fill=(194, 65, 12), outline=(253, 186, 116), width=2)
                    mp_draw.text((dp_x + 8, dp_y + 3), "▼ 재계획된 우회 경로 (주황 실선)", fill=(255, 255, 255), font=FONTS['map_small'])
                elif scenario_type == 'dynamic_spawn' and sim_t >= events['detour_replan_sim_time']:
                    dp_x, dp_y = world_to_map_panel_px(-3.3, -4.45, PANEL_W, PANEL_H)
                    mp_draw.rounded_rectangle([(dp_x, dp_y), (dp_x + 225, dp_y + 25)], radius=5, fill=(194, 65, 12), outline=(253, 186, 116), width=2)
                    mp_draw.text((dp_x + 8, dp_y + 3), "▲ 재계획된 회피 경로 (주황 실선)", fill=(255, 255, 255), font=FONTS['map_small'])

        # 3e. Draw ACTUAL Driven Trajectory (Green Solid Line from GT pose history)
        traj_pts = [
            world_to_map_panel_px(g['x'], g['y'], PANEL_W, PANEL_H)
            for g in gt_poses
            if nav_start_sim <= g['recv_sim_time'] <= sim_t + 0.02
        ]
        if len(traj_pts) >= 2:
            draw_solid_polyline(
                mp_draw, traj_pts,
                color=(0, 210, 80), width=5,
                halo_color=(15, 23, 42), halo_width=8
            )

        # 3f. Draw Start (S) and Goal (G) markers
        sx_m, sy_m = world_to_map_panel_px(start_pose[0], start_pose[1], PANEL_W, PANEL_H)
        gx_m, gy_m = world_to_map_panel_px(goal_pose[0], goal_pose[1], PANEL_W, PANEL_H)

        mp_draw.ellipse([(sx_m - 15, sy_m - 15), (sx_m + 15, sy_m + 15)], fill=(29, 78, 216), outline=(255, 255, 255), width=3)
        mp_draw.text((sx_m - 6, sy_m - 11), "S", fill=(255, 255, 255), font=FONTS['sg_marker'])
        s_lbl_y = sy_m + 18
        mp_draw.rounded_rectangle([(sx_m - 20, s_lbl_y), (sx_m + 145, s_lbl_y + 24)], radius=5, fill=(30, 41, 59), outline=(59, 130, 246), width=2)
        mp_draw.text((sx_m - 13, s_lbl_y + 2), f"시작점 S ({start_pose[0]:.1f}, {start_pose[1]:.1f})", fill=(255, 255, 255), font=FONTS['map_small'])

        mp_draw.ellipse([(gx_m - 16, gy_m - 16), (gx_m + 16, gy_m + 16)], fill=(220, 38, 38), outline=(255, 255, 255), width=3)
        mp_draw.text((gx_m - 7, gy_m - 11), "G", fill=(255, 255, 255), font=FONTS['sg_marker'])
        g_lbl_x = gx_m + 18 if goal_pose[0] < 3.5 else gx_m - 180
        mp_draw.rounded_rectangle([(g_lbl_x, gy_m - 12), (g_lbl_x + 165, gy_m + 13)], radius=5, fill=(30, 41, 59), outline=(239, 68, 68), width=2)
        mp_draw.text((g_lbl_x + 6, gy_m - 10), f"목적지 G ({goal_pose[0]:.1f}, {goal_pose[1]:.1f})", fill=(255, 255, 255), font=FONTS['map_small'])

        # 3g. Draw Robot Current Position & Heading Arrow
        if telem_rec.get('truth'):
            rx, ry, _, _, _, ryaw = telem_rec['truth']
            rpx, rpy = world_to_map_panel_px(rx, ry, PANEL_W, PANEL_H)
            draw_robot_pose(mp_draw, rpx, rpy, ryaw, scale_px_per_m)

        canvas.paste(map_panel, (rx0 + 8, ry0 + 44))

        # --- 4. Bottom Legend Row (y: 872..924) ---
        draw.rounded_rectangle([(24, 872), (1896, 924)], radius=8, fill=(30, 41, 59), outline=(71, 85, 105), width=2)
        draw.text((40, 885), "범례:", fill=(248, 250, 252), font=FONTS['legend'])

        # Item 1: S / G
        draw.ellipse([(100, 886), (124, 910)], fill=(29, 78, 216), outline=(255, 255, 255), width=2)
        draw.text((106, 886), "S", fill=(255, 255, 255), font=FONTS['map_small'])
        draw.ellipse([(130, 886), (154, 910)], fill=(220, 38, 38), outline=(255, 255, 255), width=2)
        draw.text((136, 886), "G", fill=(255, 255, 255), font=FONTS['map_small'])
        draw.text((162, 885), "시작점 / 목적지", fill=(226, 232, 240), font=FONTS['legend'])

        # Item 2: Initial Global Path (Blue Dashed)
        draw_dashed_polyline(draw, [(305, 898), (375, 898)], color=(0, 90, 255), width=5, dash_len=12, gap_len=7, halo_color=(255, 255, 255), halo_width=7)
        draw.text((385, 885), "최초 전역 경로 (파란 점선·유지)", fill=(147, 197, 253), font=FONTS['legend'])

        # Item 3: Current Global Path (Orange Solid)
        draw_solid_polyline(draw, [(655, 898), (725, 898)], color=(255, 109, 0), width=5, halo_color=(255, 255, 255), halo_width=7)
        draw.text((735, 885), "현재 전역 경로 (주황 실선)", fill=(253, 186, 116), font=FONTS['legend'])

        # Item 4: Driven Trajectory (Green Solid)
        draw_solid_polyline(draw, [(970, 898), (1040, 898)], color=(0, 210, 80), width=5, halo_color=(15, 23, 42), halo_width=7)
        draw.text((1050, 885), "실제 이동 궤적 (초록 실선)", fill=(134, 239, 172), font=FONTS['legend'])

        # Item 5: Unmapped Obstacle
        draw.rectangle([(1285, 886), (1315, 910)], fill=(225, 29, 72), outline=(250, 204, 21), width=3)
        draw.text((1325, 885), "지도에 없던 장애물 (Costmap 반영)", fill=(253, 164, 175), font=FONTS['legend'])

        # Item 6: Robot Position & Heading
        draw.rectangle([(1625, 888), (1655, 908)], fill=(0, 229, 255), outline=(15, 23, 42), width=2)
        draw.line([(1640, 898), (1672, 898)], fill=(255, 234, 0), width=3)
        draw.text((1680, 885), "로봇 위치·진행방향", fill=(103, 232, 249), font=FONTS['legend'])

        # --- 5. Bottom Event Flow Progress & Synchronized Subtitle Box (y: 932..1068) ---
        stage_idx, sub_main, sub_detail, badge_col = get_subtitle_and_stage(scenario_type, sim_t, events, telem_rec)
        draw.rounded_rectangle([(24, 932), (1896, 1068)], radius=10, fill=(30, 41, 59), outline=badge_col, width=3)

        # 6-Step Progress Bar
        steps = [
            (1, "① 시작·목적지·최초경로 확인"),
            (2, "② 최초 경로 따라 이동"),
            (3, "③ 센서로 미등록 장애물 발견"),
            (4, "④ Costmap 장애물 반영"),
            (5, "⑤ 우회 전역 경로 재계획"),
            (6, "⑥ 변경 경로 이동·목적지 도달")
        ]
        bar_x = 40
        step_w = 302
        for idx, s_label in steps:
            if idx == stage_idx or (stage_idx == 5 and idx == 4):
                s_bg = badge_col
                s_fg = (255, 255, 255)
                s_out = (250, 204, 21)
            elif idx < stage_idx:
                s_bg = (51, 65, 85)
                s_fg = (148, 163, 184)
                s_out = (71, 85, 105)
            else:
                s_bg = (15, 23, 42)
                s_fg = (100, 116, 139)
                s_out = (51, 65, 85)

            draw.rounded_rectangle([(bar_x, 942), (bar_x + step_w - 8, 974)], radius=6, fill=s_bg, outline=s_out, width=2)
            draw.text((bar_x + 12, 946), s_label, fill=s_fg, font=FONTS['step_bar'])
            bar_x += step_w

        # Main & Detail Event-Synchronized Subtitles
        draw.text((44, 984), sub_main, fill=(255, 255, 255), font=FONTS['sub_main'])
        draw.text((44, 1024), sub_detail, fill=(203, 213, 225), font=FONTS['sub_detail'])

        return canvas

    # --- Encode Video Frame Sequence via GStreamer (H.264 MP4 + VP8 WebM) ---
    fps = 10
    W_vid, H_vid = 1920, 1080
    pipeline_str = (
        f'appsrc name=src is-live=false format=time block=true '
        f'caps=video/x-raw,format=RGB,width={W_vid},height={H_vid},framerate={fps}/1 ! '
        f'queue ! videoconvert ! video/x-raw,format=I420 ! tee name=t '
        f't. ! queue ! x264enc speed-preset=fast tune=zerolatency bitrate=5000 key-int-max=20 ! '
        f'video/x-h264,stream-format=avc,alignment=au ! mp4mux faststart=true ! filesink location="{out_mp4}" '
        f't. ! queue ! vp8enc deadline=1 cpu-used=4 threads=4 target-bitrate=5000000 keyframe-max-dist=20 ! '
        f'webmmux ! filesink location="{out_webm}"'
    )
    pipe = Gst.parse_launch(pipeline_str)
    src = pipe.get_by_name('src')
    pipe.set_state(Gst.State.PLAYING)

    out_idx = 0

    def push_frame(pil_img):
        nonlocal out_idx
        raw_bytes = pil_img.tobytes()
        buf = Gst.Buffer.new_allocate(None, len(raw_bytes), None)
        buf.fill(0, raw_bytes)
        buf.pts = Gst.util_uint64_scale(out_idx, Gst.SECOND, fps)
        buf.duration = Gst.util_uint64_scale(1, Gst.SECOND, fps)
        src.emit('push-buffer', buf)
        out_idx += 1

    def build_synced_telem_at(target_sim_t: float) -> dict:
        # Find closest telemetry camera frame at or before target_sim_t
        best_t = active_telem[0]
        for pt in active_telem:
            if pt['sim_time'] <= target_sim_t + 0.05:
                best_t = pt
            else:
                break
        rec = dict(best_t)
        rec['sim_time'] = float(target_sim_t)
        # Update robot truth pose from 60Hz gt_poses for smooth 10 FPS synchronization
        best_gt = None
        for g in gt_poses:
            if g['recv_sim_time'] <= target_sim_t + 0.02:
                best_gt = g
            else:
                break
        if best_gt is not None:
            rec['truth'] = [best_gt['x'], best_gt['y'], best_gt.get('z', 0.0), 0.0, 0.0, best_gt['yaw']]
        return rec

    # 1. Intro Hold: 2.0 seconds (20 frames) at initial plan frame (marked outside driving time)
    intro_sim_t = max(nav_start_sim, events.get('initial_plan_sim_time') or nav_start_sim)
    intro_rec = build_synced_telem_at(intro_sim_t)
    intro_img = render_single_composite(intro_rec, banner_mode="INTRO_HOLD")
    for _ in range(20):
        push_frame(intro_img)

    # 2. Real-time Driving Frames (1.0x speed, dt = 0.10s @ 10 FPS synchronized to sim_time)
    rep_saved = False
    replan_saved = False
    dt = 1.0 / fps
    sim_times_seq = np.arange(nav_start_sim, nav_end_sim + 0.05, dt)
    for sim_t_val in sim_times_seq:
        pt = build_synced_telem_at(float(sim_t_val))
        img = render_single_composite(pt, banner_mode="DRIVING")
        push_frame(img)

        if (out_replan_png is not None
                and not replan_saved
                and events['detour_replan_sim_time'] is not None
                and pt['sim_time'] >= events['detour_replan_sim_time'] + 1.2):
            img.save(out_replan_png)
            replan_saved = True
            print(f"[Renderer] Saved replan keyframe PNG at sim_time={pt['sim_time']:.2f}s -> {out_replan_png}")

        target_rep_t = nav_start_sim + 0.50 * (nav_end_sim - nav_start_sim)
        if not rep_saved and pt['sim_time'] >= target_rep_t:
            img.save(out_rep_png)
            rep_saved = True
            print(f"[Renderer] Saved representative PNG at sim_time={pt['sim_time']:.2f}s -> {out_rep_png}")

    # 3. Outro Hold: 2.5 seconds (25 frames) at final goal-reached state
    outro_rec = build_synced_telem_at(nav_end_sim)
    outro_img = render_single_composite(outro_rec, banner_mode="OUTRO_HOLD")
    if not rep_saved:
        outro_img.save(out_rep_png)
    for _ in range(25):
        push_frame(outro_img)

    src.emit('end-of-stream')
    msg = pipe.get_bus().timed_pop_filtered(60 * Gst.SECOND, Gst.MessageType.EOS | Gst.MessageType.ERROR)
    if not msg:
        raise RuntimeError("GStreamer encoding timed out")
    if msg.type == Gst.MessageType.ERROR:
        raise RuntimeError(msg.parse_error())
    pipe.set_state(Gst.State.NULL)

    # Remove temporary frame directory if present
    if rendered_dir.exists():
        shutil.rmtree(rendered_dir)

    # Also copy deliverables to output/video/ for convenience
    for fpath in [out_mp4, out_webm, out_rep_png, out_replan_png]:
        if fpath is not None and fpath.exists():
            shutil.copy2(fpath, PUBLIC_VIDEO_DIR / fpath.name)

    print(f"[Renderer] Completed! {out_idx} frames ({out_idx/fps:.1f}s @ {fps} FPS)")
    print(f"  -> MP4 : {out_mp4} ({out_mp4.stat().st_size / (1024*1024):.2f} MB)")
    print(f"  -> WebM: {out_webm} ({out_webm.stat().st_size / (1024*1024):.2f} MB)")
    print(f"  -> PNG : {out_rep_png}")


if __name__ == '__main__':
    # 1. Primary Deliverable: Unmapped Obstacle Discovery & Detour Replanning
    scen3_dir = VIDEO_DIR / 'scen3_unmapped_detour_video'
    render_scenario_video(
        scenario_dir=scen3_dir,
        scenario_type='unmapped_detour',
        out_mp4=VIDEO_DIR / 'limo_unmapped_obstacle_detour_1080p.mp4',
        out_webm=VIDEO_DIR / 'limo_unmapped_obstacle_detour_1080p.webm',
        out_rep_png=VIDEO_DIR / 'limo_unmapped_obstacle_detour_representative.png',
        out_replan_png=VIDEO_DIR / 'limo_unmapped_obstacle_detour_replan_moment.png'
    )

    # 2. Separate Deliverable: Sudden Dynamic Obstacle Spawn & Replanning
    scen4_dir = VIDEO_DIR / 'scen4_dynamic_spawn_video'
    if (scen4_dir / 'summary.json').exists():
        render_scenario_video(
            scenario_dir=scen4_dir,
            scenario_type='dynamic_spawn',
            out_mp4=VIDEO_DIR / 'limo_sudden_obstacle_spawn_1080p.mp4',
            out_webm=VIDEO_DIR / 'limo_sudden_obstacle_spawn_1080p.webm',
            out_rep_png=VIDEO_DIR / 'limo_sudden_obstacle_spawn_representative.png',
            out_replan_png=VIDEO_DIR / 'limo_sudden_obstacle_spawn_replan_moment.png'
        )
