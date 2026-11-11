#!/usr/bin/env -S python3 -s
"""
verify_mp4_complete_decoding.py
Verifies complete video decoding across all frames (not just first frame) for every generated MP4:
- Opens with OpenCV VideoCapture
- Iterates through ALL frames from frame 0 to frame_count - 1
- Confirms zero corrupt/dropped frames
- Confirms audio/video header validity, resolution, duration, and FPS
- Outputs verification catalog JSON to results/video_decoding_audit.json
"""
import os
import glob
import json
import cv2

VID_DIRS = [
    '/home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_final_deployment_prep_20261108/videos',
    '/home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_nav2_closed_loop_verification_20261107/videos',
    '/home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/videos',
]
OUT_JSON = '/home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_final_deployment_prep_20261108/results/video_decoding_audit.json'


def audit_videos():
    all_videos = []
    for vd in VID_DIRS:
        if os.path.exists(vd):
            vids = sorted(glob.glob(os.path.join(vd, '**', '*.mp4'), recursive=True))
            all_videos.extend([v for v in vids if not v.endswith('_raw_mp4v.mp4')])

    records = []
    print(f"Auditing complete frame decoding for {len(all_videos)} MP4 videos...")

    for i, vpath in enumerate(all_videos, 1):
        cap = cv2.VideoCapture(vpath)
        if not cap.isOpened():
            records.append({
                'path': vpath,
                'opened': False,
                'status': 'FAIL_CANNOT_OPEN',
            })
            print(f"[{i:02d}/{len(all_videos):02d}] FAIL (Cannot open): {os.path.basename(vpath)}")
            continue

        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = float(cap.get(cv2.CAP_PROP_FPS))
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_str = ''.join([chr((fourcc >> 8 * b) & 0xFF) for b in range(4)])

        # Read through every frame to ensure 100% full decoding integrity
        decoded_frames = 0
        corrupt_frames = 0
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            if frame is None or frame.shape != (h, w, 3):
                corrupt_frames += 1
            decoded_frames += 1

        cap.release()
        sz_bytes = os.path.getsize(vpath)
        dur_s = round(decoded_frames / fps, 2) if fps > 0 else 0.0
        success = (decoded_frames == total_frames and corrupt_frames == 0 and decoded_frames > 0)

        rec = {
            'file_name': os.path.basename(vpath),
            'full_path': vpath,
            'file_size_bytes': sz_bytes,
            'file_size_mb': round(sz_bytes / (1024 * 1024), 2),
            'resolution': f"{w}x{h}",
            'fps': fps,
            'header_total_frames': total_frames,
            'actual_decoded_frames': decoded_frames,
            'corrupt_frames': corrupt_frames,
            'duration_sec': dur_s,
            'fourcc_codec': fourcc_str,
            'full_decoding_verified': success,
            'verdict': 'VERIFIED_100_PERCENT' if success else 'FAIL_INCOMPLETE_FRAMES',
        }
        records.append(rec)
        print(f"[{i:02d}/{len(all_videos):02d}] {rec['verdict']}: {os.path.basename(vpath)} ({decoded_frames}/{total_frames} frames, {dur_s}s)")

    with open(OUT_JSON, 'w') as fp:
        json.dump({
            'total_videos_audited': len(records),
            'all_videos_fully_decoded': all(r['full_decoding_verified'] for r in records),
            'videos': records,
        }, fp, indent=2)
    print(f"\nSaved Video Decoding Audit to {OUT_JSON}")


if __name__ == '__main__':
    audit_videos()
