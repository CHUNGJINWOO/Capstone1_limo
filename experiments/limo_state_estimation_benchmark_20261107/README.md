# LIMO Pro State Estimation & Video Recording Benchmark (`20261107`)

Dedicated, self-contained experiment directory for evaluating LIMO Pro state estimation algorithms (`B0`, `F1a`, `F1b`, `F2`, `F3a`, `F3b`, `F4`, `F5`, `F6`) and recording synchronized Gazebo Fortress + trajectory HUD MP4 videos without modifying any existing project files.

## Quick Start: Run the 1-Run Smoke Test

```bash
source /opt/ros/humble/setup.bash
python3 -s /home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107/scripts/verify_pipeline_smoke_test.py
```

## Key Outputs from Smoke Test
- **JSON Report:** `results/smoke_test/s1_smoke_test_report.json`
- **Raw Time-Series CSV:** `results/smoke_test/s1_smoke_test_timeseries.csv`
- **Trajectory & Error Plot:** `results/plots/s1_smoke_test_comparison.png`
- **Synchronized H.264 MP4 Video:** `videos/smoke_test/s1_normal_multiturn_smoke_test.mp4`
- **Extracted Keyframe PNG:** `videos/smoke_test/s1_normal_multiturn_smoke_test_mid_frame.png`
- **Rosbag2 Archive:** `results/smoke_test/rosbag2_s1_smoke_test/`
