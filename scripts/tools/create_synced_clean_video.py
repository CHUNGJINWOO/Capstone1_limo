#!/usr/bin/env python3
"""
create_synced_clean_video.py
Combines MC3F.webm (RViz2) and MC3F_복도.webm (Real LIMO) into a clean, wide 1080p MP4.
Key improvements:
1. Removes heavy top headers, bulky bottom card UI, and redundant descriptions.
2. Maximizes active video area (Bokdo 912px height, RViz 825px height).
3. Preserves exact native aspect ratios (16:10 and 9:16) with zero artificial crop or zoom.
4. Clean minimal labels and timecode overlay.
5. Strict H.264 High Profile, yuv420p encoding for universal PowerPoint compatibility.
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
OUTPUT_MP4 = os.path.join(WORKSPACE, "local_data/videos/presentation_videos/MC3F_synced_clean_yuv420.mp4")
POSTER_PNG = os.path.join(WORKSPACE, "docs/images/architecture/poster_synced_clean.png")


FPS = 25
TOTAL_SEC = 108.6
TOTAL_FRAMES = int(TOTAL_SEC * FPS)  # ~2715 frames
OFFSET_SEC = 6.5
OFFSET_FRAMES = int(OFFSET_SEC * FPS)  # 162 frames

# Video frame sizes (strictly preserving native aspect ratios)
# RViz: 16:10 -> 1320 x 825 (1320 / 825 = 1.600)
# Bokdo: 9:16 -> 513 x 912 (513 / 912 = 0.5625)
RVIZ_W, RVIZ_H = 1320, 825
RVIZ_X, RVIZ_Y = 32, 115

BOKDO_W, BOKDO_H = 513, 912
BOKDO_X, BOKDO_Y = 1375, 71

FONT_BOLD = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
FONT_MED = "/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc"

font_header = ImageFont.truetype(FONT_BOLD, 20)
font_label = ImageFont.truetype(FONT_BOLD, 15)
font_caption = ImageFont.truetype(FONT_MED, 15)
font_small = ImageFont.truetype(FONT_MED, 13)

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
    # Strictly enforce yuv420p (I420) and H.264 High Profile
    pipe_str = (
        f'appsrc name=src is-live=false format=time '
        f'caps=video/x-raw,format=RGB,width=1920,height=1080,framerate={FPS}/1 ! '
        f'videoconvert ! video/x-raw,format=I420 ! '
        f'x264enc speed-preset=ultrafast tune=zerolatency bitrate=4500 ! '
        f'video/x-h264,profile=high ! '
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
    print(f"Initializing clean video compositor...")
    print(f"Source 1: RViz2 ({RVIZ_W}x{RVIZ_H}, 16:10)")
    print(f"Source 2: Bokdo ({BOKDO_W}x{BOKDO_H}, 9:16)")
    print(f"Output: {OUTPUT_MP4}")

    p_rviz, sink_rviz = build_decode_pipeline(RVIZ_PATH, RVIZ_W, RVIZ_H)
    p_bokdo, sink_bokdo = build_decode_pipeline(BOKDO_PATH, BOKDO_W, BOKDO_H)
    p_out, src_out = build_encode_pipeline(OUTPUT_MP4)

    p_rviz.set_state(Gst.State.PLAYING)
    p_bokdo.set_state(Gst.State.PLAYING)
    p_out.set_state(Gst.State.PLAYING)

    dur_ns = int(1e9 / FPS)
    frame_idx = 0
    t0 = time.time()

    initial_bokdo_frame = pull_rgb_frame(sink_bokdo, BOKDO_W, BOKDO_H)
    last_bokdo_frame = initial_bokdo_frame

    print(f"Processing {TOTAL_FRAMES} frames ({TOTAL_SEC:.1f}s) at {FPS} FPS...")

    while frame_idx < TOTAL_FRAMES:
        t_curr = frame_idx / FPS

        # Pull RViz frame
        img_rviz = pull_rgb_frame(sink_rviz, RVIZ_W, RVIZ_H)
        if img_rviz is None:
            print(f"RViz reached EOF at frame {frame_idx} (t={t_curr:.2f}s). Finalizing.")
            break

        # Pull or hold Bokdo frame
        if frame_idx < OFFSET_FRAMES:
            img_bokdo = initial_bokdo_frame.copy()
            # Draw subtle standby pill
            bd = ImageDraw.Draw(img_bokdo)
            bd.rounded_rectangle([(20, BOKDO_H - 55), (BOKDO_W - 20, BOKDO_H - 20)], radius=6, fill=(15, 23, 42, 220), outline=(234, 179, 8), width=2)
            bd.text((BOKDO_W // 2, BOKDO_H - 38), "출발 대기 중 (t = 6.5s 경로 생성 연동)", fill=(253, 224, 71), font=font_small, anchor="mm")
        else:
            pulled = pull_rgb_frame(sink_bokdo, BOKDO_W, BOKDO_H)
            if pulled is not None:
                img_bokdo = pulled
                last_bokdo_frame = pulled
            else:
                img_bokdo = last_bokdo_frame

        # Clean 1080p canvas
        canvas = Image.new('RGB', (1920, 1080), (15, 23, 42))
        draw = ImageDraw.Draw(canvas)

        # Subtle top titles (Minimal, non-distracting)
        draw.text((RVIZ_X, 85), "■ RViz2 자율주행 모니터링 (글로벌 Navfn 경로 / DWB 로컬 궤적 / Costmap)", fill=(147, 197, 253), font=font_label)
        draw.text((BOKDO_X, 42), "■ LIMO Pro 실차 복도 주행 (스마트폰 촬영)", fill=(167, 243, 208), font=font_label)

        # Paste video frames
        canvas.paste(img_rviz, (RVIZ_X, RVIZ_Y))
        canvas.paste(img_bokdo, (BOKDO_X, BOKDO_Y))

        # Thin elegant border around video panes
        draw.rectangle([(RVIZ_X - 1, RVIZ_Y - 1), (RVIZ_X + RVIZ_W, RVIZ_Y + RVIZ_H)], outline=(59, 130, 246), width=1)
        draw.rectangle([(BOKDO_X - 1, BOKDO_Y - 1), (BOKDO_X + BOKDO_W, BOKDO_Y + BOKDO_H)], outline=(16, 185, 129), width=1)

        # Clean bottom synchronization footer
        draw.line([(32, 1005), (1888, 1005)], fill=(51, 65, 85), width=1)

        # Time code
        mins = int(t_curr) // 60
        secs = int(t_curr) % 60
        time_str = f"{mins:02d}:{secs:02d} / 01:48"
        draw.text((32, 1025), time_str, fill=(203, 213, 225), font=font_caption)

        # Clean status pill
        sync_desc = "동기화 정합: RViz2 목표 지정(t=6.5s) ↔ 실차 출발 동시 연동 (+6.5s 오프셋) | 38m 복도 주행 및 180° 반환 완주"
        draw.text((180, 1025), sync_desc, fill=(148, 163, 184), font=font_small)

        # Progress bar
        prog_x = 1100
        prog_w = 788
        draw.rounded_rectangle([(prog_x, 1030), (prog_x + prog_w, 1038)], radius=4, fill=(51, 65, 85))
        filled_w = int(prog_w * min(1.0, t_curr / TOTAL_SEC))
        if filled_w > 0:
            draw.rounded_rectangle([(prog_x, 1030), (prog_x + filled_w, 1038)], radius=4, fill=(56, 189, 248))

        # Push frame to encoder
        frame_bytes = canvas.tobytes()
        buf = Gst.Buffer.new_allocate(None, len(frame_bytes), None)
        buf.fill(0, frame_bytes)
        buf.pts = frame_idx * dur_ns
        buf.duration = dur_ns
        src_out.emit('push-buffer', buf)

        # Save representative poster image at t=25.0s (while moving)
        if frame_idx == int(25.0 * FPS):
            os.makedirs(os.path.dirname(POSTER_PNG), exist_ok=True)
            canvas.save(POSTER_PNG)
            print(f"Saved clean poster frame: {POSTER_PNG}")

        frame_idx += 1
        if frame_idx % 300 == 0:
            elapsed = time.time() - t0
            print(f"Rendered {frame_idx}/{TOTAL_FRAMES} frames ({t_curr:.1f}s) - speed: {frame_idx/elapsed:.1f} fps")

    print("Finalizing video stream...")
    src_out.emit('end-of-stream')
    bus = p_out.get_bus()
    bus.timed_pop_filtered(15 * Gst.SECOND, Gst.MessageType.EOS)

    p_rviz.set_state(Gst.State.NULL)
    p_bokdo.set_state(Gst.State.NULL)
    p_out.set_state(Gst.State.NULL)

    total_time = time.time() - t0
    size_mb = os.path.getsize(OUTPUT_MP4) / (1024 * 1024)
    print(f"COMPLETE: {OUTPUT_MP4} ({size_mb:.2f} MB, {frame_idx} frames in {total_time:.1f}s)")

if __name__ == '__main__':
    main()
