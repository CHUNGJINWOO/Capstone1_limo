# rmf_map_shim — keep `/map` = `nav_msgs/OccupancyGrid`, move Open-RMF `BuildingMap` to `/rmf/building_map`

## Problem (reproduced 2026-11-17)
`/map` carried two types in the Office + RMF Gazebo graph: SLAM Toolbox / Nav2 `nav_msgs/msg/OccupancyGrid` and
`rmf_building_map_msgs/msg/BuildingMap` (subscribed by the Gazebo slotcar plugins `plugin_tinyRobot1/2`, published by RMF's `building_map_server`).
Consequences observed: (1) `ros2 bag record` — `Topic '/map' has more than one type associated. Only topics with one type are supported` (591 errors in one run), no map in the bag;
(2) with a `building_map_server` running, the Agent's SLAM pre-check (`count_publishers('/map')`) rejects web SLAM start ("map producer already detected").

## Root cause / why a launch remap is not enough
The slotcar plugin (`libslotcar_common.so` from the rmf_robot_sim debs) hard-codes the **absolute** name `/map` (`strings` shows the literal) and calls `rclcpp::init(0, nullptr)`,
so no `-r map:=…` can reach it. The binary is a third-party deb extract (`~/rmf_demos_humble/prefix`), not project source, and is not modified.

## Fix
`rmf_map_remap.c` — an `LD_PRELOAD` shim interposing `rcl_subscription_init` / `rcl_publisher_init`. It renames **only** endpoints whose message type is
`rmf_building_map_msgs/msg/BuildingMap` (checked through rosidl introspection) and whose topic is `/map` or `map` to `/rmf/building_map`
(override: env `RMF_BUILDING_MAP_TOPIC`, absolute). Everything else, including OccupancyGrid on `/map`, passes through. If the type cannot be determined, it passes through.
Preload it only into the RMF/Gazebo launch tree (the Agent, SLAM, Nav2, rosbag need no preload).

```bash
bash fleet_control/rmf_adapter/rmf_map_shim/build.sh        # -> build/librmf_map_remap.so (generated, not committed)
LD_PRELOAD=$PWD/fleet_control/rmf_adapter/rmf_map_shim/build/librmf_map_remap.so ros2 launch <rmf/gazebo launch>
LD_PRELOAD=... ros2 run rmf_building_map_tools building_map_server <x.building.yaml>
```
The recorder profile (`recording_session.py`) records `/rmf/building_map` in addition to the existing `/scan_slam` and `run_recorder.py` CORE_TOPICS (`/map` included).

## Limits / to know
- Any RMF consumer that expects BuildingMap on `/map` and runs **without** the preload (e.g. rmf_visualization, rmf web API `building_map` bridge, rmf_traffic_editor tools) will not see it. In the checked graphs the only subscribers were the two slotcar plugins. Anything new must be started with the shim or pointed at `/rmf/building_map`.
- An interposer is a workaround for a binary we cannot rebuild here. The durable alternative is rebuilding `rmf_robot_sim_common` with a configurable topic (not done: sources are not in this repository).
- Not applicable to real-robot graphs without RMF Gazebo plugins (no BuildingMap subscribers there).
