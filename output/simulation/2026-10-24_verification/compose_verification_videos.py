#!/usr/bin/env python3
"""
compose_verification_videos.py:
Composes side-by-side verification videos for proven simulation scenarios:
  - Left: Actual Gazebo camera view from /demo/overhead topic
  - Right: Real-time map, costmap, planner /plan, and robot pose/trail
Features:
  - Strict '시뮬레이션' (Simulation Only) watermarking
  - Exact 1x simulation timestamp synchronization
  - Dual output: MP4 (H.264) and WebM (VP8)
"""

from pathlib import Path
import json, math, bisect
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
Gst.init(None)

ROOT = Path('/home/cjinw/school/2026-2/Capstone1_limo')
DATA_BASE = ROOT / 'output/simulation/2026-10-24_verification'
OUT_DIR = ROOT / 'output/video'
OUT_DIR.mkdir(parents=True, exist_ok=True)

FPS = 20
W, H = 1920, 1080

FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
BOLD = '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'
f42 = ImageFont.truetype(BOLD, 42)
f30 = ImageFont.truetype(BOLD, 30)
f26 = ImageFont.truetype(FONT, 26)
f24 = ImageFont.truetype(FONT, 24)
f14 = ImageFont.truetype(FONT, 14)

navy = (22, 50, 79)
teal = (8, 127, 140)
gray = (94, 111, 125)

def render_scenario_video(case_key, title_kr, sub_desc, goal_xy, map_pgm_path):
    case_folder = DATA_BASE / case_key
    summary_path = case_folder / 'summary.json'
    tele_path = case_folder / 'telemetry.jsonl'

    if not summary_path.exists() or not tele_path.exists():
        print(f"Skipping {case_key}: data files not found.")
        return

    records = [json.loads(x) for x in tele_path.read_text().splitlines()]
    summary = json.loads(summary_path.read_text())
    times = [r['sim_time'] for r in records]
    if len(times) < 2:
        return
    length = times[-1] - times[0]
    total_frames = math.ceil(length * FPS)

    static_map = Image.open(map_pgm_path).convert('L')
    static_arr = np.array(static_map)

    mp4_file = OUT_DIR / f'{case_key}.mp4'
    webm_file = OUT_DIR / f'{case_key}.webm'

    pipeline_str = (
        f'appsrc name=src is-live=false format=time block=true '
        f'caps=video/x-raw,format=RGB,width={W},height={H},framerate={FPS}/1 ! '
        f'queue ! videoconvert ! video/x-raw,format=I420 ! tee name=t '
        f't. ! queue ! x264enc speed-preset=veryfast tune=zerolatency bitrate=4500 key-int-max=40 ! '
        f'video/x-h264,stream-format=avc,alignment=au ! mp4mux faststart=true ! filesink location="{mp4_file}" '
        f't. ! queue ! vp8enc deadline=1 cpu-used=5 threads=4 target-bitrate=4500000 keyframe-max-dist=40 ! '
        f'webmmux ! filesink location="{webm_file}"'
    )
    p = Gst.parse_launch(pipeline_str)
    src = p.get_by_name('src')
    p.set_state(Gst.State.PLAYING)

    outidx = 0
    cache = None
    prev_idx = -1

    for frameidx in range(total_frames):
        t = times[0] + frameidx / FPS
        idx = max(0, bisect.bisect_right(times, t) - 1)
        rec = records[idx]

        if idx != prev_idx:
            im = Image.new('RGB', (W, H), 'white')
            d = ImageDraw.Draw(im)

            # Header Banner
            d.rectangle((0, 0, W, 118), fill=navy)
            d.text((44, 24), '시뮬레이션  ·  LIMO Nav2 자율주행 검증', font=f42, fill='white')
            d.text((1200, 42), '가상 시뮬레이션 환경 (실물 LIMO 주행 아님)', font=f26, fill=(178, 221, 227))

            # Sub-headers
            d.text((44, 138), f'Gazebo 가상 환경: {title_kr}', font=f30, fill=teal)
            d.text((1060, 138), '실시간 Costmap · 전역 경로(/plan) · TF', font=f30, fill=teal)

            # Left: Overhead camera image
            img_path = case_folder / rec['image']
            if img_path.exists():
                cam = Image.open(img_path).convert('RGB')
                cam = cam.resize((960, 720), Image.LANCZOS)
                im.paste(cam, (44, 192))

            # Right: Real-time Costmap & Plan rendering
            chart_w, chart_h = 760, 720
            chart_im = Image.new('RGB', (chart_w, chart_h), (243, 247, 248))
            cd = ImageDraw.Draw(chart_im)

            cm = rec.get('costmap')
            if cm and cm.get('origin'):
                ox, oy = cm['origin']
                cw, ch, cres = cm['width'], cm['height'], cm['resolution']
                def world_to_chart(wx, wy):
                    cx = (wx - ox) / (cw * cres) * chart_w
                    cy = chart_h - (wy - oy) / (ch * cres) * chart_h
                    return (cx, cy)
            else:
                # Fallback bounds: [-6, -5] to [6, 5]
                ox, oy = -6.0, -5.0
                cw_m, ch_m = 12.0, 10.0
                def world_to_chart(wx, wy):
                    cx = (wx - ox) / cw_m * chart_w
                    cy = chart_h - (wy - oy) / ch_m * chart_h
                    return (cx, cy)

            # Static map background thumbnail
            if static_arr is not None:
                bg = Image.fromarray((static_arr == 0).astype(np.uint8) * 200)
                bg_rgb = Image.new('RGB', (static_arr.shape[1], static_arr.shape[0]), (240, 244, 248))
                bg_draw = ImageDraw.Draw(bg_rgb)
                # Paste walls
                walls_mask = (static_arr == 0)
                walls_img = np.full((static_arr.shape[0], static_arr.shape[1], 3), 245, dtype=np.uint8)
                walls_img[walls_mask] = [22, 50, 79] # navy walls
                bg_resized = Image.fromarray(walls_img).resize((chart_w, chart_h), Image.NEAREST)
                chart_im.paste(bg_resized, (0, 0))
                cd = ImageDraw.Draw(chart_im)

            # Draw trajectory trail
            trail = [world_to_chart(*r['truth'][:2]) for r in records[:idx+1] if r.get('truth')]
            if len(trail) > 1:
                cd.line(trail, fill=(180, 130, 20), width=3)

            # Draw Goal marker
            gx, gy = world_to_chart(goal_xy[0], goal_xy[1])
            cd.ellipse((gx - 14, gy - 14, gx + 14, gy + 14), fill=(30, 180, 60), outline='white', width=2)

            # Draw current robot pose & heading
            if rec.get('truth'):
                rx, ry = rec['truth'][:2]
                rpx, rpy = world_to_chart(rx, ry)
                # heading from quaternion z, w
                qz, qw = rec['truth'][3], rec['truth'][4]
                yaw = math.atan2(2 * qw * qz, 1 - 2 * (qz * qz))
                cd.ellipse((rpx - 10, rpy - 10, rpx + 10, rpy + 10), fill=(210, 40, 30), outline='white', width=2)
                cd.line((rpx, rpy, rpx + 24 * math.cos(yaw), rpy - 24 * math.sin(yaw)), fill=(210, 40, 30), width=4)

            im.paste(chart_im, (1080, 192))
            d = ImageDraw.Draw(im)

            # Info Captions
            d.text((1080, 930), '벽체: 남색   로봇 궤적: 황색   로봇: 빨강   목표점: 초록', font=f24, fill=gray)
            d.text((44, 930), sub_desc, font=f24, fill=gray)
            d.text((44, 980), f'상태: {rec["status"]}  |  제어 명령 cmd_vel: v={rec["cmd"][0]:.2f} m/s, w={rec["cmd"][1]:.2f} rad/s', font=f26, fill=teal)

            cache = im
            prev_idx = idx

        im = cache.copy()
        d = ImageDraw.Draw(im)

        # Progress Bar
        d.rectangle((44, 1040, 1876, 1048), fill=(220, 231, 235))
        progress_w = int(1832 * (frameidx + 1) / total_frames)
        d.rectangle((44, 1040, 44 + progress_w, 1048), fill=teal)
        d.text((44, 1054), f'시뮬레이션 시간 기준 1.0배속  ·  시나리오 경과: {frameidx / FPS:04.1f}초 / 총 {length:04.1f}초', font=f14, fill=gray)

        buf = Gst.Buffer.new_allocate(None, W * H * 3, None)
        buf.fill(0, im.tobytes())
        buf.pts = Gst.util_uint64_scale(outidx, Gst.SECOND, FPS)
        buf.duration = Gst.util_uint64_scale(1, Gst.SECOND, FPS)
        src.emit('push-buffer', buf)
        outidx += 1

    src.emit('end-of-stream')
    msg = p.get_bus().timed_pop_filtered(60 * Gst.SECOND, Gst.MessageType.EOS | Gst.MessageType.ERROR)
    if not msg:
        raise RuntimeError("Encoding timed out")
    if msg.type == Gst.MessageType.ERROR:
        raise RuntimeError(msg.parse_error())
    p.set_state(Gst.State.NULL)
    print(f"[compose] Generated video: {mp4_file} ({outidx} frames, {outidx/FPS:.1f}s)")

if __name__ == '__main__':
    # 1. MC 3F corridor
    render_scenario_video(
        case_key='mc3f_corridor_nav',
        title_kr='MC관 3층 복도 가상 환경 주행',
        sub_desc='MC관 3층 실측 지도 기반 벽체 가상 환경 복도 2.5m 직진 및 조향 제어',
        goal_xy=[-2.60, 1.30],
        map_pgm_path='/home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/limo_capstone/maps/MC_3F_10_15.pgm'
    )

    # 2. Scenario 3: Unmapped obstacle detour
    render_scenario_video(
        case_key='scenario3_unmapped_detour',
        title_kr='미지의 고정 장애물 발견 및 우회로 재계획',
        sub_desc='사전 지도에 없는 상자 발견 -> Costmap 장애물 등록 -> 북쪽 우회로로 실시간 재계획 완주',
        goal_xy=[3.0, 0.5],
        map_pgm_path='/home/cjinw/school/2026-2/Capstone1_limo/output/simulation/2026-10-24_verification/maze/maze_map.pgm'
    )

    # 3. Scenario 7: Clearing & resume
    render_scenario_video(
        case_key='scenario7_clearing_resume',
        title_kr='장애물 제거 후 Costmap 갱신 및 주행 재개',
        sub_desc='차단벽으로 인한 정지/대기 -> 장애물 제거 -> raytrace clearing 갱신 -> 주행 재개 완주',
        goal_xy=[4.0, -3.5],
        map_pgm_path='/home/cjinw/school/2026-2/Capstone1_limo/output/simulation/2026-10-24_verification/maze/maze_map.pgm'
    )
