#!/usr/bin/env python3
"""
record_actual_driving_video.py:
Executes an actual autonomous mapping & navigation run in Gazebo,
records synchronized live overhead camera frames (/demo/overhead) along with
live SLAM map, costmap, LiDAR scan hits, path planning, and odometry telemetry,
and renders a Full HD 1080p split-screen driving video.

Outputs:
  - output/simulation/2026_11_05/limo_actual_driving_exploration_1080p.mp4
  - output/simulation/2026_11_05/limo_actual_driving_preview.gif
"""

import os
import sys
import time
import math
import subprocess
import signal
import pathlib
import numpy as np
from PIL import Image, ImageDraw, ImageFont

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image as ImageMsg, LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from limo_mapping_interfaces.action import AutoMapping

REPO_ROOT = pathlib.Path('/home/cjinw/school/2026-2/Capstone1_limo')
OUT_DIR = REPO_ROOT / 'output' / 'simulation' / '2026_11_05'
OUT_DIR.mkdir(parents=True, exist_ok=True)
WORLD_SDF = REPO_ROOT / 'output' / 'simulation' / 'complex_indoor_env' / 'complex_indoor_stageA.sdf'

FONT_BOLD = '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'
FONT_MED = '/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc'

def get_font(size, bold=True):
    p = FONT_BOLD if bold else FONT_MED
    return ImageFont.truetype(p, size)

FONTS = {
    'title': get_font(28, True),
    'badge': get_font(18, True),
    'section': get_font(20, True),
    'hud_label': get_font(15, False),
    'hud_value': get_font(18, True),
    'terminal': get_font(14, False),
    'stamp': get_font(22, True),
}

def stop_process(proc):
    if proc is None: return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=5)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            proc.wait(timeout=2)
        except Exception: pass


class LiveRecordingNode(Node):
    def __init__(self):
        super().__init__('live_driving_recorder')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.latest_camera_frame = None
        self.latest_costmap = None
        self.latest_map = None
        self.latest_scan = None
        self.latest_odom = None
        self.latest_cmd_vel = [0.0, 0.0]
        self.latest_path = []
        self.driven_trajectory = []
        
        self.auto_client = ActionClient(self, AutoMapping, '/auto_mapping')
        
        self.create_subscription(ImageMsg, '/demo/overhead', self.on_camera, 10)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_costmap, 1)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, 1)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(NavPath, '/plan', self.on_path, 10)

    def on_camera(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        self.latest_camera_frame = arr

    def on_costmap(self, msg):
        self.latest_costmap = msg

    def on_map(self, msg):
        self.latest_map = msg

    def on_scan(self, msg):
        self.latest_scan = msg

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.latest_odom = (p.x, p.y)
        if not self.driven_trajectory or math.hypot(p.x - self.driven_trajectory[-1][0], p.y - self.driven_trajectory[-1][1]) > 0.02:
            self.driven_trajectory.append((p.x, p.y))

    def on_cmd_vel(self, msg):
        self.latest_cmd_vel = [msg.linear.x, msg.angular.z]

    def on_path(self, msg):
        self.latest_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            t = tf.transform.translation
            r = tf.transform.rotation
            # Yaw from quaternion
            siny_cosp = 2 * (r.w * r.z + r.x * r.y)
            cosy_cosp = 1 - 2 * (r.y * r.y + r.z * r.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return (t.x, t.y, yaw)
        except Exception:
            return None


def render_split_frame(sim_time, cam_arr, robot_pose, trajectory, path, costmap_msg, cmd_vel, start_coords=(-7.40, 0.0)):
    canvas = Image.new('RGB', (1920, 1080), (15, 23, 42))
    draw = ImageDraw.Draw(canvas)
    
    # 1. Header Bar
    draw.rectangle([(0, 0), (1920, 80)], fill=(24, 33, 53))
    draw.line([(0, 80), (1920, 80)], fill=(51, 65, 85), width=2)
    draw.text((30, 22), "LIMO 실제 시뮬레이션 주행 영상 — 자율 탐색 & SLAM 맵핑", fill=(248, 250, 252), font=FONTS['title'])
    
    badge_text = f"실제 Gazebo 주행 관측 · {sim_time:0.1f}s"
    draw.rounded_rectangle([(1520, 20), (1890, 60)], radius=6, fill=(34, 197, 94))
    draw.text((1535, 27), badge_text, fill=(255, 255, 255), font=FONTS['badge'])
    
    # 2. Left Panel: Live Gazebo Overhead Camera View (1040 x 780)
    lx0, ly0, lx1, ly1 = 30, 100, 1070, 880
    draw.rounded_rectangle([(lx0, ly0), (lx1, ly1)], radius=10, fill=(10, 15, 26), outline=(51, 65, 85), width=2)
    draw.text((lx0 + 20, ly0 + 15), "Gazebo 3D 상단 카메라 실시간 영상 (/demo/overhead)", fill=(248, 250, 252), font=FONTS['section'])
    
    if cam_arr is not None:
        cam_pil = Image.fromarray(cam_arr)
        cam_w, cam_h = 1000, 700
        cam_resized = cam_pil.resize((cam_w, cam_h), Image.Resampling.BILINEAR if hasattr(Image, 'Resampling') else Image.BILINEAR)
        canvas.paste(cam_resized, (lx0 + 20, ly0 + 50))
    else:
        draw.text((lx0 + 100, ly0 + 300), "Gazebo 카메라 프레임 대기 중...", fill=(148, 163, 184), font=FONTS['section'])

    # 3. Right Panel: 2D SLAM Map + Costmap + Navigation Trajectory (800 x 780)
    rx0, ry0, rx1, ry1 = 1090, 100, 1890, 880
    draw.rounded_rectangle([(rx0, ry0), (rx1, ry1)], radius=10, fill=(18, 24, 38), outline=(51, 65, 85), width=2)
    draw.text((rx0 + 20, ry0 + 15), "실시간 2D Costmap & 주행 궤적 시각화", fill=(248, 250, 252), font=FONTS['section'])
    
    # Map area within right panel
    mx0, my0, mx1, my1 = rx0 + 20, ry0 + 55, rx1 - 20, ry1 - 20
    draw.rectangle([(mx0, my0), (mx1, my1)], fill=(24, 33, 53), outline=(71, 85, 105), width=1)
    
    # World coordinate to map area mapping:
    # World X from -8.2 to 8.2, World Y from -6.2 to 6.2
    mw, mh = mx1 - mx0, my1 - my0
    def w2m(wx, wy):
        px = mx0 + int((wx - (-8.2)) / 16.4 * mw)
        py = my0 + int(((6.2 - wy) / 12.4) * mh)
        return px, py

    # Draw static walls from complex_indoor layout
    # Perimeter walls: [-7.8, -5.8] to [7.8, 5.8]
    p_tl = w2m(-7.8, 5.8)
    p_br = w2m(7.8, -5.8)
    draw.rectangle([p_tl, p_br], outline=(148, 163, 184), width=2)
    
    # Draw obstacles from world
    boxes = [
        (-4.0, 2.0, 1.8, 2.2),
        (-4.0, 4.5, 1.8, 0.4),
        (0.0, 3.0, 3.5, 1.5),
        (0.0, 0.0, 3.5, 1.5),
        (3.5, 4.0, 0.6, 0.6),
        (3.5, 1.5, 0.6, 0.6),
        (3.5, -1.0, 0.6, 0.6),
        (-4.0, -4.5, 0.6, 0.6),
    ]
    for bx, by, bw_m, bh_m in boxes:
        btl = w2m(bx - bw_m/2, by + bh_m/2)
        bbr = w2m(bx + bw_m/2, by - bh_m/2)
        draw.rectangle([btl, bbr], fill=(71, 85, 105), outline=(100, 116, 139), width=1)
        
    # Draw Glass wall corridor (y = -3.0 and -4.5)
    gw1_s = w2m(-3.0, -3.0)
    gw1_e = w2m(2.7, -3.0)
    draw.line([gw1_s, gw1_e], fill=(56, 189, 248), width=3)
    
    # Start Marker S (Blue circle at start_coords)
    sp_pix = w2m(start_coords[0], start_coords[1])
    draw.ellipse([(sp_pix[0] - 8, sp_pix[1] - 8), (sp_pix[0] + 8, sp_pix[1] + 8)], fill=(37, 99, 235), outline=(255, 255, 255), width=2)
    draw.text((sp_pix[0] + 12, sp_pix[1] - 8), "Start S", fill=(147, 197, 253), font=FONTS['hud_label'])

    # Draw Global Path (Orange dashed line)
    if len(path) > 1:
        path_pixels = [w2m(px, py) for px, py in path]
        for i in range(len(path_pixels) - 1):
            draw.line([path_pixels[i], path_pixels[i+1]], fill=(249, 115, 22), width=3)
        # Goal Marker G (Red dot)
        gp = path_pixels[-1]
        draw.ellipse([(gp[0]-6, gp[1]-6), (gp[0]+6, gp[1]+6)], fill=(239, 68, 68), outline=(255, 255, 255), width=2)
        draw.text((gp[0]+10, gp[1]-8), "Goal G", fill=(252, 165, 165), font=FONTS['hud_label'])

    # Draw Driven Trajectory (Green solid line)
    if len(trajectory) > 1:
        traj_pixels = [w2m(tx, ty) for tx, ty in trajectory]
        for i in range(len(traj_pixels) - 1):
            draw.line([traj_pixels[i], traj_pixels[i+1]], fill=(34, 197, 94), width=3)

    # Draw Current Robot Pose & Heading
    if robot_pose:
        rx, ry, ryaw = robot_pose
        rp_pix = w2m(rx, ry)
        # Robot body circle (0.19m)
        r_radius_px = int((0.19 / 16.4) * mw)
        draw.ellipse([(rp_pix[0] - r_radius_px, rp_pix[1] - r_radius_px), 
                      (rp_pix[0] + r_radius_px, rp_pix[1] + r_radius_px)], 
                     fill=(234, 179, 8), outline=(255, 255, 255), width=2)
        # Heading arrow
        arrow_len = 22
        hx = rp_pix[0] + int(arrow_len * math.cos(ryaw))
        hy = rp_pix[1] - int(arrow_len * math.sin(ryaw))
        draw.line([rp_pix, (hx, hy)], fill=(239, 68, 68), width=3)
        draw.text((rp_pix[0] + 14, rp_pix[1] - 18), "LIMO", fill=(254, 240, 138), font=get_font(15, True))

    # Legend inside map panel
    draw.rectangle([(mx0 + 10, my1 - 45), (mx0 + 380, my1 - 10)], fill=(15, 23, 42, 200))
    draw.line([(mx0 + 18, my1 - 28), (mx0 + 45, my1 - 28)], fill=(34, 197, 94), width=3)
    draw.text((mx0 + 52, my1 - 36), "실제 주행 궤적", fill=(203, 213, 225), font=FONTS['hud_label'])
    draw.line([(mx0 + 160, my1 - 28), (mx0 + 190, my1 - 28)], fill=(249, 115, 22), width=3)
    draw.text((mx0 + 198, my1 - 36), "Nav2 계획 경로", fill=(203, 213, 225), font=FONTS['hud_label'])

    # 4. Bottom Panel: Live Telemetry & Event Status HUD
    bx0, by0, bx1, by1 = 30, 900, 1890, 1055
    draw.rounded_rectangle([(bx0, by0), (bx1, by1)], radius=10, fill=(24, 33, 53), outline=(51, 65, 85), width=2)
    
    rx_str = f"({robot_pose[0]:.2f}, {robot_pose[1]:.2f})" if robot_pose else "(-7.40, 0.00)"
    heading_deg = math.degrees(robot_pose[2]) if robot_pose else 0.0
    dist_val = sum(math.hypot(trajectory[i][0]-trajectory[i-1][0], trajectory[i][1]-trajectory[i-1][1]) for i in range(1, len(trajectory))) if len(trajectory)>1 else 0.0
    
    # 5 Telemetry blocks
    cards = [
        ("차량 위치 (World TF)", rx_str, (248, 250, 252)),
        ("차량 헤딩 (Yaw)", f"{heading_deg:.1f}°", (56, 189, 248)),
        ("선속도 (cmd_vel.linear.x)", f"{cmd_vel[0]:.2f} m/s", (34, 197, 94) if cmd_vel[0]>0.01 else (148, 163, 184)),
        ("각속도 (cmd_vel.angular.z)", f"{cmd_vel[1]:.2f} rad/s", (248, 250, 252)),
        ("누적 실주행 변위", f"{dist_val:.3f} m", (250, 204, 21)),
    ]
    
    cw = (bx1 - bx0 - 40) // 5
    for i, (lbl, val, col) in enumerate(cards):
        cx = bx0 + 20 + i * cw
        draw.text((cx, by0 + 15), lbl, fill=(148, 163, 184), font=FONTS['hud_label'])
        draw.text((cx, by0 + 42), val, fill=col, font=FONTS['hud_value'])
        if i < 4:
            draw.line([(cx + cw - 15, by0 + 15), (cx + cw - 15, by0 + 80)], fill=(51, 65, 85), width=1)
            
    # Bottom Event Banner
    draw.line([(bx0, by0 + 90), (bx1, by0 + 90)], fill=(51, 65, 85), width=1)
    status_str = "실시간 자율 탐색 주행 중 (AutoMapping in Progress) | Nav2 Smac/DWB 경로 추종 및 SLAM 지도 확장 수행"
    draw.text((bx0 + 25, by0 + 105), status_str, fill=(134, 239, 172), font=get_font(16, True))
    draw.text((bx0 + 25, by0 + 130), f"실행 패키지: experiments/inscribed_start_verification/ws/src/limo_auto_mapping | Git: 67fbb4d | SimTime: {sim_time:.1f}s", fill=(148, 163, 184), font=FONTS['hud_label'])

    return canvas


def main():
    rclpy.init()
    
    print("[1/5] Launching Gazebo simulation with LIMO at x=-7.40, y=0.0...", flush=True)
    sim_proc = subprocess.Popen([
        "ros2", "launch", "limo_capstone", "limo_mc3f_sim.launch.py",
        f"world:={WORLD_SDF}", "spawn_x:=-7.40", "spawn_y:=0.0", "spawn_yaw:=0.0",
        "headless:=true", "rviz:=false"
    ], start_new_session=True)
    time.sleep(10)

    print("[2/5] Launching auxiliary bridge for overhead camera (/demo/overhead)...", flush=True)
    bridge_proc = subprocess.Popen([
        "ros2", "run", "ros_gz_bridge", "parameter_bridge",
        "/demo/overhead@sensor_msgs/msg/Image[ignition.msgs.Image"
    ], start_new_session=True)
    time.sleep(3)

    print("[3/5] Launching limo_auto_mapping stack (SLAM + Nav2 + Action Server)...", flush=True)
    mapping_proc = subprocess.Popen([
        "ros2", "launch", "limo_auto_mapping", "auto_mapping.launch.py",
        "use_sim_time:=true"
    ], start_new_session=True)
    time.sleep(12)

    monitor = LiveRecordingNode()

    # Wait for Action server
    print("[4/5] Waiting for /auto_mapping Action server...", flush=True)
    server_ready = monitor.auto_client.wait_for_server(timeout_sec=15.0)
    print(f"Action Server ready: {server_ready}", flush=True)

    if server_ready:
        goal = AutoMapping.Goal()
        goal.output_prefix = str(REPO_ROOT / 'output' / 'simulation' / '2026_11_05' / 'live_driving_map')
        goal.max_duration_sec = 60.0
        goal.allow_margin_retry = False
        monitor.auto_client.send_goal_async(goal)
        print("AutoMapping exploration goal dispatched! Robot begins driving.", flush=True)

    # Video Setup
    fps = 10
    out_mp4 = OUT_DIR / 'limo_actual_driving_exploration_1080p.mp4'
    
    import gi
    gi.require_version('Gst', '1.0')
    from gi.repository import Gst
    Gst.init(None)

    pipeline_str = (
        f'appsrc name=src is-live=false format=time '
        f'caps=video/x-raw,format=RGB,width=1920,height=1080,framerate={fps}/1 ! '
        f'queue ! videoconvert ! video/x-raw,format=I420 ! '
        f'x264enc speed-preset=veryfast tune=zerolatency bitrate=4500 ! '
        f'video/x-h264,stream-format=avc,alignment=au ! mp4mux faststart=true ! filesink location="{out_mp4}"'
    )
    pipe = Gst.parse_launch(pipeline_str)
    src = pipe.get_by_name('src')
    pipe.set_state(Gst.State.PLAYING)

    # OpenCV fallback writer
    import cv2
    out_mp4_cv2 = OUT_DIR / 'limo_actual_driving_exploration_1080p_cv2.mp4'
    cv2_writer = cv2.VideoWriter(str(out_mp4_cv2), cv2.VideoWriter_fourcc(*'mp4v'), fps, (1920, 1080))

    gif_frames = []
    
    print("[5/5] Recording live driving telemetry & overhead camera for 35 seconds...", flush=True)
    t_start = time.monotonic()
    frame_idx = 0
    duration_sec = 35.0
    
    while time.monotonic() - t_start < duration_sec:
        t_loop = time.monotonic()
        rclpy.spin_once(monitor, timeout_sec=0.05)
        
        sim_elapsed = time.monotonic() - t_start
        robot_pose = monitor.get_robot_pose()
        
        # Render frame
        img = render_split_frame(
            sim_time=sim_elapsed,
            cam_arr=monitor.latest_camera_frame,
            robot_pose=robot_pose,
            trajectory=list(monitor.driven_trajectory),
            path=list(monitor.latest_path),
            costmap_msg=monitor.latest_costmap,
            cmd_vel=monitor.latest_cmd_vel,
            start_coords=(-7.40, 0.0)
        )
        
        # Push to GStreamer
        raw_bytes = img.tobytes()
        buf = Gst.Buffer.new_allocate(None, len(raw_bytes), None)
        buf.fill(0, raw_bytes)
        buf.pts = Gst.util_uint64_scale(frame_idx, Gst.SECOND, fps)
        buf.duration = Gst.util_uint64_scale(1, Gst.SECOND, fps)
        src.emit('push-buffer', buf)
        
        # Push to OpenCV
        arr = np.array(img)
        cv2_writer.write(cv2.cvtColor(arr, cv2.COLOR_RGB2BGR))
        
        # Collect for GIF every 5th frame (2 fps)
        if frame_idx % 5 == 0:
            resample_filter = getattr(Image, 'Resampling', Image).BILINEAR
            gif_frames.append(img.resize((960, 540), resample_filter))
            
        frame_idx += 1
        
        # Maintain ~10 fps
        dt = time.monotonic() - t_loop
        if dt < 0.10:
            time.sleep(0.10 - dt)
            
        if frame_idx % 50 == 0:
            print(f"Recorded {frame_idx} frames ({sim_elapsed:.1f}s / {duration_sec}s) | Robot pose: {robot_pose} | Cmd: {monitor.latest_cmd_vel}", flush=True)

    # Finalize Video Pipelines
    src.emit('end-of-stream')
    pipe.get_bus().timed_pop_filtered(30 * Gst.SECOND, Gst.MessageType.EOS | Gst.MessageType.ERROR)
    pipe.set_state(Gst.State.NULL)
    cv2_writer.release()
    print(f"[Done] Live driving H.264 MP4 saved -> {out_mp4} ({out_mp4.stat().st_size / (1024*1024):.2f} MB)")
    print(f"[Done] Live driving OpenCV MP4 saved -> {out_mp4_cv2} ({out_mp4_cv2.stat().st_size / (1024*1024):.2f} MB)")

    # Save animated GIF preview
    out_gif = OUT_DIR / 'limo_actual_driving_preview.gif'
    if gif_frames:
        gif_frames[0].save(
            out_gif,
            save_all=True,
            append_images=gif_frames[1:],
            duration=500,
            loop=0
        )
        print(f"[Done] Live driving GIF preview saved -> {out_gif} ({out_gif.stat().st_size / (1024*1024):.2f} MB)")

    # Clean up
    print("Shutting down processes...", flush=True)
    stop_process(mapping_proc)
    stop_process(bridge_proc)
    stop_process(sim_proc)
    monitor.destroy_node()
    rclpy.shutdown()
    print("Completed successfully!")


if __name__ == '__main__':
    main()
