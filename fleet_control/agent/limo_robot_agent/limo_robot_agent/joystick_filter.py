"""Input validation for a future gated manual-mapping path; never publishes motion."""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class JoystickConfig:
    linear_axis: int
    angular_axis: int
    deadman_button: int
    linear_limit: float
    angular_limit: float
    timeout_s: float

    def __post_init__(self):
        for value in (self.linear_axis, self.angular_axis, self.deadman_button):
            if type(value) is not int or value < 0:
                raise ValueError('axis and button indices must be nonnegative integers')
        for value in (self.linear_limit, self.angular_limit, self.timeout_s):
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError('limits and timeout must be finite and positive')


def joystick_command(config, axes, buttons, *, received_at, now, manual_enabled):
    """Return (linear, angular, reason); absence and bad inputs always yield zero."""
    if manual_enabled is not True:
        return 0.0, 0.0, 'manual_not_enabled'
    if any(type(x) not in (int, float) or not math.isfinite(x) for x in (received_at, now)):
        return 0.0, 0.0, 'invalid_time'
    if now < received_at or now - received_at >= config.timeout_s:
        return 0.0, 0.0, 'stale'
    if not isinstance(axes, (list, tuple)) or not isinstance(buttons, (list, tuple)):
        return 0.0, 0.0, 'invalid_input'
    if max(config.linear_axis, config.angular_axis) >= len(axes) or config.deadman_button >= len(buttons):
        return 0.0, 0.0, 'missing_input'
    if type(buttons[config.deadman_button]) is not int or buttons[config.deadman_button] != 1:
        return 0.0, 0.0, 'deadman_released'
    linear, angular = axes[config.linear_axis], axes[config.angular_axis]
    if any(type(v) not in (int, float) or not math.isfinite(v) or abs(v) > 1 for v in (linear, angular)):
        return 0.0, 0.0, 'invalid_axis'
    return linear * config.linear_limit, angular * config.angular_limit, 'active'
