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
CLEARANCE = 1.5    # [m] min spacing between spawns and between spawn->slot routes


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
        name = f'followers[{i}]'
        slot = _pair(e.get('slot'), f'{name}.slot')
        # Spawn: spawn_rel = [ahead, right] of the leader's start pose (turns
        # and moves with the leader), or spawn = absolute [north, east].
        # Neither: on the slot.
        if 'spawn_rel' in e and 'spawn' in e:
            raise ValueError(f"{path}: {name} has both spawn_rel and spawn; use one")
        if 'spawn' in e:
            spawn = _pair(e['spawn'], f'{name}.spawn')
        else:
            spawn = cfg.slot_world(*_pair(e.get('spawn_rel', slot), f'{name}.spawn_rel'))
        # Heading: heading_rel_deg relative to the leader's, or heading_deg
        # absolute. Neither: same as the leader.
        if 'heading_rel_deg' in e and 'heading_deg' in e:
            raise ValueError(f"{path}: {name} has both heading_rel_deg and heading_deg; use one")
        if 'heading_deg' in e:
            heading = radians(float(e['heading_deg']))
        else:
            heading = cfg.leader_heading + radians(float(e.get('heading_rel_deg', 0.0)))
        cfg.followers.append(Follower(slot, spawn, heading))

    check_clearance(cfg)
    return cfg


def _seg_dist(p, q, a, b):
    """Shortest distance between 2-D segments p-q and a-b."""
    def pt_seg(x, u, v):
        dx, dy = v[0] - u[0], v[1] - u[1]
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x[0] - u[0]) * dx + (x[1] - u[1]) * dy) / L2))
        return hypot(x[0] - u[0] - t * dx, x[1] - u[1] - t * dy)

    def cross(o, u, v):
        return (u[0] - o[0]) * (v[1] - o[1]) - (u[1] - o[1]) * (v[0] - o[0])

    d1, d2 = cross(a, b, p), cross(a, b, q)
    d3, d4 = cross(p, q, a), cross(p, q, b)
    if d1 * d2 < 0 and d3 * d4 < 0:
        return 0.0                                   # proper crossing
    return min(pt_seg(p, a, b), pt_seg(q, a, b), pt_seg(a, p, q), pt_seg(b, p, q))


def check_clearance(cfg):
    """All drones leave their spawns together when the leader starts moving
    and head for their slots; nothing keeps them apart. So refuse spawns that
    are too close, and spawn->slot
    routes (straight lines around the leader's start pose, an approximation of
    the real curved flight) that cross or pass within CLEARANCE."""
    route = [(f.spawn, cfg.slot_world(*f.slot)) for f in cfg.followers]
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            (si, gi), (sj, gj) = route[i], route[j]
            if hypot(si[0] - sj[0], si[1] - sj[1]) < CLEARANCE:
                raise ValueError(f"{cfg.path}: drones {i + 1} and {j + 1} spawn "
                                 f"under {CLEARANCE} m apart")
            d = _seg_dist(si, gi, sj, gj)
            if d < CLEARANCE:
                raise ValueError(
                    f"{cfg.path}: drones {i + 1} and {j + 1} would pass {d:.2f} m apart "
                    f"flying spawn->slot (drone {i + 1}: ({si[0]:g},{si[1]:g})->"
                    f"({gi[0]:g},{gi[1]:g}), drone {j + 1}: ({sj[0]:g},{sj[1]:g})->"
                    f"({gj[0]:g},{gj[1]:g}) NED). Use spawn_rel so spawns turn with "
                    "the leader, or move the spawns.")


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
