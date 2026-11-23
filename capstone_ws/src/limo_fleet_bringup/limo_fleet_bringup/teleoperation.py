def input_parameters(manual, velocity):
    # G2: the v5 Agent's Arbiter parameter is manual_locked_arbiter (origin/main: locked_velocity_arbiter). The v5 Agent
    # refuses a mock bench without the Arbiter and the Arbiter without the watchdog (fail-closed sys.exit).
    return {'manual_web_bench': manual, 'locked_velocity_arbiter': velocity, 'manual_locked_arbiter': velocity,
            'require_motion_watchdog': True}
