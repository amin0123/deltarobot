"""Three-DOF Delta IK/FK using measured bearing and ball-centre locations.

The paired rods constrain platform rotation. Analytic IK uses the midpoint of
each parallelogram. FK intersects three spheres; the lower assembly branch is
selected. Joint angles are upper-arm angles below the outward horizontal.
"""
import numpy as np


class KinematicsError(ValueError):
    pass


class DeltaKinematics:
    def __init__(self, geometry, config=None):
        self.g = geometry
        self.b = np.array(geometry["shoulders"])
        self.r = np.array(geometry["radial"])
        self.a = np.array(geometry["axes"])
        self.u = np.array(geometry["upper_lengths"])
        self.l = np.array(geometry["lower_lengths"]).mean(1)
        self.offsets = np.array(geometry["platform_offsets"]).mean(1)
        self.tool = np.array(geometry["tool_offset"])
        self.limits = geometry["joint_limits"]
        self.cfg = config

    def elbows(self, q):
        q = np.asarray(q)
        return self.b + self.u[:, None] * (self.r * np.cos(q)[:, None]
                       + np.array([0, 0, -1]) * np.sin(q)[:, None])

    def check_target(self, target):
        p = np.asarray(target, dtype=float)
        if p.shape != (3,) or not np.isfinite(p).all():
            raise KinematicsError("target must contain three finite coordinates")
        if self.cfg:
            robot = self.cfg["robot"]
            if np.linalg.norm(p[:2]) > robot["workspace_radius"]:
                raise KinematicsError("outside commissioned radial workspace")
            if not robot["z_range"][0] <= p[2] <= robot["z_range"][1]:
                raise KinematicsError("outside commissioned vertical workspace")
        return p

    def inverse(self, target):
        p = self.check_target(target) - self.tool
        d = p + self.offsets - self.b
        aa = np.sum(d * self.r, axis=1)
        bb = -d[:, 2]
        cc = (np.sum(d * d, axis=1) + self.u**2 - self.l**2) / (2 * self.u)
        denom = np.hypot(aa, bb)
        if np.any(denom < 1e-8) or np.any(abs(cc) > denom):
            raise KinematicsError("target has no real Delta IK solution")
        q = np.arctan2(bb, aa) - np.arccos(cc / denom)
        if np.any(q < self.limits[0]) or np.any(q > self.limits[1]):
            raise KinematicsError("target exceeds assumed shoulder limits")
        # The transmission vanishes at a serial singularity.
        transmission = abs(-aa * np.sin(q) + bb * np.cos(q))
        if np.min(transmission) < 0.015:
            raise KinematicsError("target too close to a singularity")
        return q

    def forward(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (3,) or not np.isfinite(q).all():
            raise KinematicsError("invalid joint coordinates")
        c = self.elbows(q) - self.offsets
        ex = c[1] - c[0]
        d = np.linalg.norm(ex)
        if d < 1e-9:
            raise KinematicsError("degenerate sphere centres")
        ex /= d
        i = np.dot(ex, c[2] - c[0])
        ey = c[2] - c[0] - i * ex
        j = np.linalg.norm(ey)
        if j < 1e-9:
            raise KinematicsError("parallel singularity")
        ey /= j
        ez = np.cross(ex, ey)
        x = (self.l[0]**2 - self.l[1]**2 + d*d) / (2*d)
        y = (self.l[0]**2 - self.l[2]**2 + i*i + j*j - 2*i*x) / (2*j)
        zz = self.l[0]**2 - x*x - y*y
        if zz < 0:
            raise KinematicsError("inconsistent lower-rod constraints")
        points = [c[0] + x*ex + y*ey + sign*np.sqrt(zz)*ez for sign in [-1, 1]]
        return min(points, key=lambda p: p[2]) + self.tool

    def path(self, start, end, speed):
        """Prevalidate a straight tool path and quintic time scaling, including joint rates."""
        start, end = np.asarray(start), np.asarray(end)
        if not np.isfinite(speed) or speed <= 0:
            raise KinematicsError("speed must be positive and finite")
        speed = min(speed, self.cfg["robot"]["cartesian_speed"] if self.cfg else 0.12)
        duration = max(0.5, 1.875*np.linalg.norm(end-start) / speed)
        s = np.linspace(0, 1, 161)
        blend = 10*s**3 - 15*s**4 + 6*s**5
        positions = start + blend[:, None]*(end-start)
        q = np.array([self.inverse(p) for p in positions])
        if self.cfg:
            self.check_collisions(positions)
            velocity = np.max(abs(np.gradient(q, s, axis=0)))
            acceleration = np.max(abs(np.gradient(np.gradient(q, s, axis=0), s, axis=0)))
            duration = max(duration, velocity / self.cfg["robot"]["joint_speed"],
                           np.sqrt(acceleration / self.cfg["robot"]["joint_acceleration"]))
        return duration, q

    def check_collisions(self, positions):
        """Conservative tool envelope for this fixed cell; not a generic mesh planner."""
        cfg = self.cfg
        belt = cfg["conveyor"]
        for p in positions:
            z = p[2] + cfg["robot"]["base_height"]
            if abs(p[1]) < belt["width"]/2 + 0.008 and z < belt["top"] + 0.009:
                raise KinematicsError("tool would contact the belt")
            for centre in cfg["bins"].values():
                delta = abs(p[:2] - centre)
                half = cfg["bin_geometry"]["inner_width"] / 2
                if z < cfg["bin_geometry"]["rim"] + 0.016 and np.max(delta) < half + 0.016:
                    raise KinematicsError("low tool path intersects a bin envelope")
