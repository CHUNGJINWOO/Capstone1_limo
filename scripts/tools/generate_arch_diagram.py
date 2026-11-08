#!/usr/bin/env python3
"""
generate_arch_diagram.py
Generates a clean, prominent, high-resolution ROS 2 node-topic data flow diagram
for Slide 4 of the LIMO midterm presentation, strictly eliminating redundant QoS/TF
text that belongs to Appendix Slide 10.
"""

from PIL import Image, ImageDraw, ImageFont
import os

W, H = 1600, 720
img = Image.new('RGB', (W, H), (248, 250, 252)) # Light slate bg
draw = ImageDraw.Draw(img)

FONT_BOLD = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
FONT_MED = "/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc"

f_title = ImageFont.truetype(FONT_BOLD, 20)
f_box_h = ImageFont.truetype(FONT_BOLD, 16)
f_box_b = ImageFont.truetype(FONT_MED, 13)
f_badge = ImageFont.truetype(FONT_BOLD, 12)
f_flow = ImageFont.truetype(FONT_BOLD, 12)
f_node = ImageFont.truetype(FONT_BOLD, 13)

# 4 Major Stages
cols = [
    {
        "stage": "STAGE 01",
        "title": "센싱 및 입력 (Sensing)",
        "badge_col": (30, 58, 138),
        "border_col": (191, 219, 254),
        "bg_col": (239, 246, 255),
        "items": [
            ("2D LiDAR (EAI X4)", "ydlidar_ros2_driver_node\n토픽: /scan (거리 계측 데이터)"),
            ("차륜 엔코더 & IMU", "모터 회전각 및 각속도 계측\n토픽: /odom (바퀴 오도메트리)"),
            ("Astra Pro RGB-D", "시각/깊이 영상 취득\n(전방 환경 인식 및 안내 보조)")
        ]
    },
    {
        "stage": "STAGE 02",
        "title": "지도 & 위치추정 (Localization)",
        "badge_col": (15, 118, 110),
        "border_col": (153, 246, 228),
        "bg_col": (240, 253, 250),
        "items": [
            ("[매핑] SLAM Toolbox", "async_slam_toolbox_node\n온라인 비동기 점유격자 (/map)"),
            ("[위치추정] Nav2 AMCL", "amcl 노드 (정적 지도 기반)\n파티클 필터 위치 보정 (map→odom)"),
            ("[기준선] Cartographer", "wego 패키지 기준선 비교\n2D 서브맵 그래프 최적화")
        ]
    },
    {
        "stage": "STAGE 03",
        "title": "Nav2 경로 계획 (Planning)",
        "badge_col": (67, 56, 202),
        "border_col": (199, 210, 254),
        "bg_col": (238, 242, 255),
        "items": [
            ("Costmap 2D (전역/로컬)", "정적 지도 + 실시간 장애물 레이어\n로봇 안전 인플레이션 마진 계산"),
            ("Navfn Planner (Global)", "A* 다익스트라 최단 경로 (/plan)\n목표점까지 전역 궤적 생성"),
            ("DWB Controller (Local)", "궤적 샘플링 평가 및 장애물 회피\n로봇 속도 제어 명령 연산")
        ]
    },
    {
        "stage": "STAGE 04",
        "title": "구동 및 폐루프 (Actuation)",
        "badge_col": (180, 83, 9),
        "border_col": (254, 215, 170),
        "bg_col": (255, 251, 235),
        "items": [
            ("limo_base_node", "Nav2 속도 명령 수신 (/cmd_vel)\n4WD / Ackermann 기구학 변환"),
            ("하부 MCU 시리얼 제어", "/dev/ttyTHS1 UART 통신\n좌우 4개 차륜 모터 드라이브"),
            ("오도메트리 폐루프", "실시간 바퀴 회전 피드백\nTF 브로드캐스트 (odom→base_link)")
        ]
    }
]

card_w = 325
card_h = 460
gap = 65
start_x = 42
start_y = 35

# Connecting topic labels between stages
flow_labels = [
    ("/scan, /odom", (30, 58, 138)),
    ("/map, TF", (15, 118, 110)),
    ("/cmd_vel", (67, 56, 202))
]

for idx, col in enumerate(cols):
    cx = start_x + idx * (card_w + gap)
    cy = start_y

    # Main Card Box
    draw.rounded_rectangle([(cx, cy), (cx + card_w, cy + card_h)], radius=12, fill=col["bg_col"], outline=col["border_col"], width=2)

    # Stage Badge
    draw.rounded_rectangle([(cx + 16, cy + 16), (cx + 96, cy + 38)], radius=6, fill=col["badge_col"])
    draw.text((cx + 24, cy + 20), col["stage"], fill=(255, 255, 255), font=f_badge)

    # Title
    draw.text((cx + 16, cy + 46), col["title"], fill=(15, 23, 42), font=f_title)
    draw.line([(cx + 16, cy + 78), (cx + card_w - 16, cy + 78)], fill=col["border_col"], width=1)

    # Items inside stage
    iy = cy + 92
    for item_tit, item_desc in col["items"]:
        draw.rounded_rectangle([(cx + 14, iy), (cx + card_w - 14, iy + 108)], radius=8, fill=(255, 255, 255), outline=col["border_col"], width=1)

        # Item bullet dot & title
        draw.ellipse([(cx + 26, iy + 14), (cx + 34, iy + 22)], fill=col["badge_col"])
        draw.text((cx + 42, iy + 10), item_tit, fill=(15, 23, 42), font=f_box_h)

        # Item desc
        lines = item_desc.split("\n")
        dy = iy + 36
        for line in lines:
            draw.text((cx + 42, dy), line, fill=(71, 85, 105), font=f_box_b)
            dy += 22

        iy += 120

    # Flow label & Arrow to next stage
    if idx < 3:
        ax = cx + card_w + 10
        ay = cy + card_h // 2
        fl_text, fl_col = flow_labels[idx]

        # Arrow line and head
        draw.line([(ax, ay), (ax + 34, ay)], fill=(100, 116, 139), width=3)
        draw.polygon([(ax + 44, ay), (ax + 30, ay - 6), (ax + 30, ay + 6)], fill=(100, 116, 139))

        # Flow topic pill above arrow (centered)
        pill_w = 78
        draw.rounded_rectangle([(ax - 16, ay - 30), (ax - 16 + pill_w, ay - 8)], radius=4, fill=(255, 255, 255), outline=fl_col, width=1)
        draw.text((ax - 12, ay - 28), fl_text, fill=fl_col, font=f_flow)

# Bottom Node-Topic Streamline (rqt_graph Pipeline Architecture)
b_bar_y = start_y + card_h + 20
draw.rounded_rectangle([(start_x, b_bar_y), (start_x + 4 * card_w + 3 * gap, b_bar_y + 175)], radius=10, fill=(15, 23, 42), outline=(51, 65, 85), width=2)

draw.text((start_x + 25, b_bar_y + 14), "■ ROS 2 노드-토픽 데이터 파이프라인 (rqt_graph 기반 실제 토폴로지)", fill=(248, 250, 252), font=f_title)
draw.text((start_x + 680, b_bar_y + 17), "(※ 상세 QoS 프로파일 및 Lifecycle 상태 전이는 [부록 2]에 기술)", fill=(148, 163, 184), font=f_flow)

nodes = [
    ("ydlidar_driver", "LiDAR 센서 드라이버"),
    ("slam_toolbox / amcl", "지도 구축 및 위치추정"),
    ("planner_server", "Navfn 전역 최적경로"),
    ("controller_server", "DWB 로컬 장애물 회피"),
    ("limo_base", "MCU 시리얼 모터 구동")
]

topics = [
    "/scan",
    "/map",
    "/plan",
    "/cmd_vel"
]

nx = start_x + 25
ny = b_bar_y + 60
nw = 220
nh = 80

for i, (n_name, n_role) in enumerate(nodes):
    draw.rounded_rectangle([(nx, ny), (nx + nw, ny + nh)], radius=8, fill=(30, 41, 59), outline=(59, 130, 246) if i in [1, 3] else (100, 116, 139), width=2)
    draw.text((nx + 12, ny + 12), n_name, fill=(255, 255, 255), font=f_node)
    draw.text((nx + 12, ny + 42), n_role, fill=(148, 163, 184), font=f_box_b)

    if i < len(topics):
        tx = nx + nw + 10
        # Arrow and topic
        draw.line([(tx, ny + nh//2), (tx + 55, ny + nh//2)], fill=(56, 189, 248), width=2)
        draw.polygon([(tx + 60, ny + nh//2), (tx + 48, ny + nh//2 - 6), (tx + 48, ny + nh//2 + 6)], fill=(56, 189, 248))
        draw.text((tx + 2, ny + nh//2 - 22), topics[i], fill=(56, 189, 248), font=f_flow)
        nx += nw + 75

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
out_dir = os.path.join(REPO_ROOT, "docs/images/architecture")
os.makedirs(out_dir, exist_ok=True)
out_file = os.path.join(out_dir, "system_architecture_diagram.png")
img.save(out_file)
# Also save to local_data if directory exists
local_opt = os.path.join(REPO_ROOT, "local_data/reports/presentation_assets/images/opt")
if os.path.exists(local_opt):
    img.save(os.path.join(local_opt, "system_architecture_diagram.png"))
print(f"Generated clean architecture diagram: {out_file}")
