"""Names of the velocity / safety chain (Phase 3.5, 2026-11-21). One place for the topic and node names that several modules check.

Values are unchanged from the literals they replace. The locked chain is:
  VelocityArbiter --NAV_CMD_VEL--> fleet_motion_watchdog --CMD_VEL_RAW--> Safety Gate --CMD_VEL (remapped to CMD_VEL_LOCKED in isolation)
  Nav2 velocity_smoother --NAV_SMOOTHED--> VelocityArbiter
Changing a value here changes the safety graph checks everywhere; do it only together with the launch files and the Gate.
"""
CMD_VEL = '/cmd_vel'                         # motor topic; must have NO endpoint in the isolated / locked configurations
CMD_VEL_DRIVER = '/limo_driver/cmd_vel'       # real robot: vendor driver input (remapped away from /cmd_vel); written only by the Gate when unlocked
CMD_VEL_RAW = '/cmd_vel_raw'                 # Gate input, written only by the Motion Watchdog
CMD_VEL_LOCKED = '/cmd_vel_locked'           # Gate output in isolated tests (the Gate's /cmd_vel remapped)
NAV_CMD_VEL = '/fleet/nav_cmd_vel'           # Arbiter output = Watchdog input
MANUAL_CMD_VEL = '/fleet/manual_cmd_vel'     # on-board joystick channel of the Watchdog (mode gate only)
NAV_SMOOTHED = '/fleet/cmd_vel_nav_smoothed' # Nav2 velocity_smoother output = Arbiter NAVIGATION/MAPPING input
NAV_RAW = '/fleet/cmd_vel_nav'               # controller_server / behavior_server output = velocity_smoother input (Arbiter: liveness only)
GATE_STATUS = '/stall_detector/status_json'
GATE_RESET_SERVICE = '/stall_detector/reset_latch'
SPEED_LIMIT = '/fleet/speed_limit'             # Agent -> Watchdog: approved speed limits (JSON), only when the Watchdog clamp is enabled
WATCHDOG_STATUS = '/fleet/motion_watchdog/status'
AGENT_HEARTBEAT = '/fleet/agent_heartbeat'

GATE_NODE = 'stall_detector_node'
WATCHDOG_NODE = 'fleet_motion_watchdog'
SMOOTHER_NODE = 'velocity_smoother'
