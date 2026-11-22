"""RMF <-> Nav2 `map` frame transform (2D rigid transform).  p_rmf = R(theta) * p_nav + t,  yaw_rmf = yaw_nav + theta."""
import math


def wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi


class Transform:
    def __init__(self, rotation_rad=0.0, translation=(0.0, 0.0)):
        for v in (rotation_rad, *translation):
            if not isinstance(v, (int, float)) or not math.isfinite(v): raise ValueError('transform values must be finite numbers')
        self.theta = float(rotation_rad); self.tx, self.ty = float(translation[0]), float(translation[1])
        self._c, self._s = math.cos(self.theta), math.sin(self.theta)

    @classmethod
    def from_config(cls, d): return cls(d.get('rotation_rad', 0.0), d.get('translation', (0.0, 0.0)))

    def nav2_to_rmf(self, x, y, yaw=0.0):
        return (self._c * x - self._s * y + self.tx, self._s * x + self._c * y + self.ty, wrap(yaw + self.theta))

    def rmf_to_nav2(self, x, y, yaw=0.0):
        dx, dy = x - self.tx, y - self.ty
        return (self._c * dx + self._s * dy, -self._s * dx + self._c * dy, wrap(yaw - self.theta))
