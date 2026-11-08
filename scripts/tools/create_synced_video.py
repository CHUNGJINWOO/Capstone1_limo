#!/usr/bin/env python3
"""
create_synced_video.py
Combines MC3F.webm (RViz2) and MC3F_복도.webm (Real LIMO) into a single 1080p MP4.
Time synchronization: MC3F_복도.webm starts moving at t = 6.5s of MC3F.webm.
Preserves aspect ratios: RViz2 is 16:10 (1152x720), Real LIMO is 9:16 (405x720).
"""

import os
import sys
import time
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
from PIL import Image, ImageDraw, ImageFont

Gst.init(None)

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
WORKSPACE = os.environ.get("CAPSTONE_WORKSPACE", REPO_ROOT)

def resolve_path(preferred_rel, legacy_rel):
    p = os.path.join(WORKSPACE, preferred_rel)
    if os.path.exists(p):
        return p
    return os.path.join(WORKSPACE, legacy_rel)

RVIZ_PATH = resolve_path("local_data/videos/presentation_videos/MC3F.webm", "발표/videos/MC3F.webm")
BOKDO_PATH = resolve_path("local_data/videos/presentation_videos/MC3F_복도.webm", "발표/videos/MC3F_복도.webm")
OUTPUT_MP4 = os.path.join(WORKSPACE, "local_data/videos/presentation_videos/MC3F_synced_demo.mp4")
POSTER_PNG = os.path.join(WORKSPACE, "docs/images/architecture/poster_synced_demo.png")


FPS = 25
TOTAL_SEC = 108.6
TOTAL_FRAMES = int(TOTAL_SEC * FPS)  # ~2715 frames
OFFSET_SEC = 6.5
OFFSET_FRAMES = int(OFFSET_SEC * FPS)  # 162 frames

FONT_PATH_BOLD = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
FONT_PATH_MED = "/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc"

font_title = ImageFont.truetype(FONT_PATH_BOLD, 30)
font_sub = ImageFont.truetype(FONT_PATH_BOLD, 18)
font_body = ImageFont.truetype(FONT_PATH_MED, 20)
font_small = ImageFont.truetype(FONT_PATH_MED, 15)
font_standby = ImageFont.truetype(FONT_PATH_BOLD, 16)

def build_decode_pipeline(uri, width, height):
    pipe_str = (
        f'uridecodebin uri="file://{uri}" ! '
        f'videoconvert ! videoscale ! videorate ! '
        f'video/x-raw,format=RGB,width={width},height={height},framerate={FPS}/1 ! '
        f'appsink name=sink sync=false drop=false max-buffers=10'
    )
    pipe = Gst.parse_launch(pipe_str)
    sink = pipe.get_by_name('sink')
    return pipe, sink

def build_encode_pipeline(out_path):
    pipe_str = (
        f'appsrc name=src is-live=false format=time '
        f'caps=video/x-raw,format=RGB,width=1920,height=1080,framerate={FPS}/1 ! '
        f'videoconvert ! '
        f'x264enc speed-preset=ultrafast tune=zerolatency bitrate=3500 ! '
        f'mp4mux ! '
        f'filesink location="{out_path}"'
    )
    pipe = Gst.parse_launch(pipe_str)
    src = pipe.get_by_name('src')
    return pipe, src

def pull_rgb_frame(sink, width, height):
    sample = sink.emit('pull-sample')
    if not sample:
        return None
    buf = sample.get_buffer()
    success, map_info = buf.map(Gst.MapFlags.READ)
    if not success:
        return None
    data = bytes(map_info.data)
    buf.unmap(map_info)
    return Image.frombytes('RGB', (width, height), data)

def main():
    print("Initializing video processing pipelines...")
    p_rviz, sink_rviz = build_decode_pipeline(RVIZ_PATH, 1152, 720)
    p_bokdo, sink_bokdo = build_decode_pipeline(BOKDO_PATH, 405, 720)
    p_out, src_out = build_encode_pipeline(OUTPUT_MP4)

    p_rviz.set_state(Gst.State.PLAYING)
    p_bokdo.set_state(Gst.State.PLAYING)
    p_out.set_state(Gst.State.PLAYING)

    dur_ns = int(1e9 / FPS)
    frame_idx = 0
    t0 = time.time()

    initial_bokdo_frame = None
    last_bokdo_frame = None

    # Pre-pull frame 0 for bokdo
    initial_bokdo_frame = pull_rgb_frame(sink_bokdo, 405, 720)
    last_bokdo_frame = initial_bokdo_frame

    print(f"Starting composition: {TOTAL_FRAMES} frames ({TOTAL_SEC:.1f}s) @ {FPS} fps...")

    while frame_idx < TOTAL_FRAMES:
        t_curr = frame_idx / FPS

        # Pull RViz frame
        img_rviz = pull_rgb_frame(sink_rviz, 1152, 720)
        if img_rviz is None:
            print(f"RViz reached EOF at frame {frame_idx} (t={t_curr:.2f}s). Ending stream.")
            break

        # Pull or hold Bokdo frame
        if frame_idx < OFFSET_FRAMES:
            # Standby mode before robot starts moving
            img_bokdo = initial_bokdo_frame.copy()
            # Draw standby badge over bokdo frame
            bd = ImageDraw.Draw(img_bokdo)
            bd.rounded_rectangle([(15, 660), (390, 705)], radius=6, fill=(15, 23, 42, 230), outline=(234, 179, 8), width=2)
            bd.text((25, 672), "[대기] RViz2 목표 지정 대기 중", fill=(253, 224, 71), font=font_standby)
        else:
            pulled = pull_rgb_frame(sink_bokdo, 405, 720)
            if pulled is not None:
                img_bokdo = pulled
                last_bokdo_frame = pulled
            else:
                img_bokdo = last_bokdo_frame

        # Create composite canvas
        canvas = Image.new('RGB', (1920, 1080), (15, 23, 42))
        draw = ImageDraw.Draw(canvas)

        # Header background
        draw.rectangle([(0, 0), (1920, 110)], fill=(11, 17, 32))
        draw.line([(0, 110), (1920, 110)], fill=(51, 65, 85), width=2)

        # Header Title & Badges
        draw.text((60, 22), 'LIMO 미래관 3층 복도 자율주행 시연 (RViz2 계획 vs 실차 주행 동기화)', fill=(248, 250, 252), font=font_title)

        badges = [
            ('ROS 2 Humble / Nav2', (30, 58, 138), (147, 197, 253)),
            ('시간 동기화 오프셋 +6.5s', (6, 95, 70), (167, 243, 208)),
            ('AgileX LIMO Pro', (55, 65, 81), (229, 231, 235))
        ]
        bx = 60
        for text, bg, fg in badges:
            bbox = font_small.getbbox(text)
            bw = bbox[2] - bbox[0] + 20
            draw.rounded_rectangle([(bx, 72), (bx + bw, 97)], radius=6, fill=bg)
            draw.text((bx + 10, 75), text, fill=fg, font=font_small)
            bx += bw + 15

        # Video panes labels
        draw.text((60, 118), '▶ RViz2 자율주행 모니터링 (글로벌 Navfn 경로 / DWB 로컬 궤적 / Costmap)', fill=(96, 165, 250), font=font_sub)
        draw.text((1360, 118), '▶ LIMO Pro 실차 복도 주행 (스마트폰 촬영 / 동기화)', fill=(52, 211, 153), font=font_sub)

        # Paste videos
        canvas.paste(img_rviz, (60, 145))
        canvas.paste(img_bokdo, (1360, 145))

        # Borders around videos
        draw.rectangle([(59, 144), (60+1152, 145+720)], outline=(59, 130, 246), width=2)
        draw.rectangle([(1359, 144), (1360+405, 145+720)], outline=(16, 185, 129), width=2)

        # Bottom Card
        draw.rounded_rectangle([(60, 885), (1860, 1060)], radius=12, fill=(30, 41, 59), outline=(51, 65, 85), width=2)

        # Determine phase and status
        # 0 ~ 7s: 목표 지정
        # 7 ~ 9s: 경로 계획
        # 9 ~ 59s: 복도 전진 주행
        # 59 ~ 69s: 반환점 180도 회전
        # 69 ~ 89s: 출발지 복귀 주행
        # 89s ~ : 도착 및 정지
        active_phase_idx = 0
        desc_text = ""

        if t_curr < 7.0:
            active_phase_idx = 0
            desc_text = "• 현재 상태: RViz2에서 복도 끝 목표 지점(2D Goal Pose)을 지정하고 초기 유효성을 검증합니다."
        elif t_curr < 9.0:
            active_phase_idx = 1
            desc_text = "• 현재 상태: Navfn 글로벌 플래너가 최적 경로(초록색)를 생성하고, DWB 로컬 플래너가 초기 궤적을 연산합니다."
        elif t_curr < 59.0:
            active_phase_idx = 2
            desc_text = "• 현재 상태: LIMO Pro 실차가 타일 라인을 따라 직진 주행 중이며, 10Hz LiDAR 스캔과 50.1Hz 오도메트리로 오차를 보정합니다."
        elif t_curr < 69.0:
            active_phase_idx = 3
            desc_text = "• 현재 상태: 복도 끝 창가 반환 지점에서 인플레이션 안전 마진을 유지하며 180° 제자리 회전 기동을 수행합니다."
        elif t_curr < 89.0:
            active_phase_idx = 4
            desc_text = "• 현재 상태: 출발지를 향해 복귀 경로를 추종하며, 센서 피드백을 통해 헤딩 각도와 궤적을 실시간 제어합니다."
        else:
            active_phase_idx = 5
            desc_text = "• 현재 상태: 목표 허용오차(xy: 0.25m, yaw: 0.25rad) 내 진입 후 DWB 컨트롤러가 감속하여 복귀 정지를 완료했습니다."

        # Draw Phase Steps
        phases = [
            '1. 목표 지정',
            '2. 경로 계획',
            '3. 복도 전진 주행',
            '4. 반환점 180° 회전',
            '5. 출발지 복귀 주행',
            '6. 도착 및 정지'
        ]

        px = 80
        for p_idx, name in enumerate(phases):
            is_active = (p_idx == active_phase_idx)
            p_bg = (37, 99, 235) if is_active else (51, 65, 85)
            p_fg = (255, 255, 255) if is_active else (148, 163, 184)
            bbox = font_sub.getbbox(name)
            pw = bbox[2] - bbox[0] + 30
            draw.rounded_rectangle([(px, 900), (px + pw, 932)], radius=8, fill=p_bg)
            draw.text((px + 15, 905), name, fill=p_fg, font=font_sub)
            px += pw + 25

        # Draw Phase Description
        draw.text((80, 955), desc_text, fill=(241, 245, 249), font=font_body)

        # Draw Progress Bar & Time code
        mins = int(t_curr) // 60
        secs = int(t_curr) % 60
        time_str = f"{mins:02d}:{secs:02d} / 01:48"
        draw.text((80, 1005), time_str, fill=(148, 163, 184), font=font_small)
        draw.rounded_rectangle([(180, 1012), (1840, 1022)], radius=5, fill=(51, 65, 85))
        prog_w = int((1840 - 180) * min(1.0, t_curr / TOTAL_SEC))
        draw.rounded_rectangle([(180, 1012), (180 + prog_w, 1022)], radius=5, fill=(56, 189, 248))

        # Push to encoder
        frame_bytes = canvas.tobytes()
        buf = Gst.Buffer.new_allocate(None, len(frame_bytes), None)
        buf.fill(0, frame_bytes)
        buf.pts = frame_idx * dur_ns
        buf.duration = dur_ns
        src_out.emit('push-buffer', buf)

        # Save poster frame around t = 25.0s
        if frame_idx == int(25.0 * FPS):
            os.makedirs(os.path.dirname(POSTER_PNG), exist_ok=True)
            canvas.save(POSTER_PNG)
            print(f"Saved representative poster frame at t=25.0s: {POSTER_PNG}")

        frame_idx += 1
        if frame_idx % 250 == 0:
            elapsed = time.time() - t0
            fps_speed = frame_idx / elapsed
            print(f"Processed {frame_idx}/{TOTAL_FRAMES} frames ({t_curr:.1f}s) - speed: {fps_speed:.1f} fps")

    # Send EOS to encoder
    print("Ending stream and finalizing MP4...")
    src_out.emit('end-of-stream')
    bus = p_out.get_bus()
    bus.timed_pop_filtered(15 * Gst.SECOND, Gst.MessageType.EOS)

    p_rviz.set_state(Gst.State.NULL)
    p_bokdo.set_state(Gst.State.NULL)
    p_out.set_state(Gst.State.NULL)

    total_time = time.time() - t0
    size_mb = os.path.getsize(OUTPUT_MP4) / (1024 * 1024)
    print(f"SUCCESS: Rendered {frame_idx} frames in {total_time:.1f}s ({frame_idx/total_time:.1f} fps)")
    print(f"Output video: {OUTPUT_MP4} ({size_mb:.2f} MB)")

if __name__ == '__main__':
    main()
