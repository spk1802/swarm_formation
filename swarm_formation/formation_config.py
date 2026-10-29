#!/usr/bin/env python3
"""
Initial poses and formation slots from config/formation.yaml.

Shared by start_swarm.sh (spawn poses) and every node, so the spawn table and
the slots can no longer drift apart between files. ROS-free on purpose:
start_swarm.sh calls it before ROS is sourced.

    python3 formation_config.py FILE --spawns   # "i north east heading_rad" per drone
    python3 formation_config.py FILE --summary  # one line for run_config.txt
"""
import os
import sys
from dataclasses import dataclass
from math import atan2, cos, degrees, hypot, radians, sin
from typing import List, Tuple

import yaml

MAX_FOLLOWERS = 4  # follower_controller has publishers for px4_1..px4_4


@dataclass
class Follower:
    slot: Tuple[float, float]       # G_i = (ahead, right) of the leader [m]
    spawn: Tuple[float, float]      # NED (north, east) Gazebo spawn [m]
    heading: float                  # spawn heading [rad], 0 = North


@dataclass
class FormationConfig:
    path: str
    leader_start: Tuple[float, float]   # NED (north, east) [m]
    leader_heading: float               # [rad], 0 = North
    followers: List[Follower]

    def slot_world(self, gx, gy):
        """Slot (ahead, right) -> NED point around the leader's start pose."""
        n0, e0 = self.leader_start
        c, s = cos(self.leader_heading), sin(self.leader_heading)
        return n0 + gx * c - gy * s, e0 + gx * s + gy * c


def default_path():
    """Installed share/ copy when ROS is sourced, else the source tree's."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('swarm_formation'),
                            'config', 'formation.yaml')
    except Exception:
        here = os.path.dirname(os.path.realpath(__file__))
        return os.path.join(here, '..', 'config', 'formation.yaml')


def _pair(v, what):
    if not (isinstance(v, (list, tuple)) and len(v) == 2):
        raise ValueError(f"{what} must be [a, b], got {v!r}")
    return float(v[0]), float(v[1])


def load(path=''):
    path = os.path.realpath(os.path.expanduser(path or default_path()))
    with open(path) as f:
        raw = yaml.safe_load(f) or {}

    lead = raw.get('leader') or {}
    cfg = FormationConfig(
        path=path,
        leader_start=_pair(lead.get('start', [0.0, 0.0]), 'leader.start'),
        leader_heading=radians(float(lead.get('heading_deg', 0.0))),
        followers=[])

    entries = raw.get('followers') or []
    if not 1 <= len(entries) <= MAX_FOLLOWERS:
        raise ValueError(f"{path}: need 1..{MAX_FOLLOWERS} followers, got {len(entries)}")
    for i, e in enumerate(entries, 1):
        slot = _pair(e.get('slot'), f'followers[{i}].slot')
        spawn = (_pair(e['spawn'], f'followers[{i}].spawn') if 'spawn' in e
                 else cfg.slot_world(*slot))
        heading = (radians(float(e['heading_deg'])) if 'heading_deg' in e
                   else cfg.leader_heading)
        cfg.followers.append(Follower(slot, spawn, heading))

    # Two drones spawned on top of each other collide on takeoff.
    for i, a in enumerate(cfg.followers, 1):
        for j, b in enumerate(cfg.followers[i:], i + 1):
            if hypot(a.spawn[0] - b.spawn[0], a.spawn[1] - b.spawn[1]) < 1.5:
                raise ValueError(f"{path}: drones {i} and {j} spawn under 1.5 m apart")
    return cfg


def summary(cfg):
    n0, e0 = cfg.leader_start
    parts = [f"leader=({n0:g},{e0:g})@{degrees(cfg.leader_heading):g}deg"]
    for i, f in enumerate(cfg.followers, 1):
        parts.append(f"d{i}: slot=({f.slot[0]:g},{f.slot[1]:g}) "
                     f"spawn=({f.spawn[0]:g},{f.spawn[1]:g})@{degrees(f.heading):g}deg")
    return " | ".join(parts)


def leader_on_path_start(cfg, tol=1e-6):
    """True when the leader starts exactly where every path starts: the
    origin, heading North (exact mode). Otherwise it must fly onto the path."""
    n0, e0 = cfg.leader_start
    h = atan2(sin(cfg.leader_heading), cos(cfg.leader_heading))
    return hypot(n0, e0) < tol and abs(h) < tol


if __name__ == '__main__':
    if len(sys.argv) != 3 or sys.argv[2] not in ('--spawns', '--summary'):
        sys.exit(__doc__)
    try:
        c = load(sys.argv[1])
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as e:
        sys.exit(f"formation config: {e}")
    if sys.argv[2] == '--spawns':
        r = lambda x: round(x, 6) + 0.0   # noqa: E731  (+0.0 drops -0.0)
        for k, f in enumerate(c.followers, 1):
            print(k, r(f.spawn[0]), r(f.spawn[1]), r(f.heading))
    else:
        print(summary(c))
