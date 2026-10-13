#!/usr/bin/env python3
"""
path_leader.py -- the virtual leader: any path, exact or vector-field mode.

One node for every reference path (Basak & Ghosh, ICC 2025):

    path:=sinusoid | figure8 | circle | straight_line      (default sinusoid)
    path_offset:=0    exact mode  -- leader generated exactly on the path
    path_offset:=5    VF mode     -- path shifted 5 m East; the leader starts
                                     at the origin and flies ONTO it with the
                                     paper's vector field (Eq. 5) and Lemma-1
                                     saturation law (Eq. 6)

    ./start_swarm.sh --leader figure8 --path-offset 5 --delta on

Publishes (NED; the follower's carousel Delta and dDelta/dt need all three)
  /virtual_leader_pose      PoseStamped   x, y, z, quaternion(yaw = chi_l)
  /virtual_leader_velocity  TwistStamped  linear xy, angular.z = chi_l_dot
  /virtual_leader_accel     AccelStamped  linear xy (inertial), angular.z = chi_l_ddot
  /reference_path_marker    Marker        ENU, frame 'map', latched

Paths are parametric r(u) with closed-form r', r'', r'''. Frame: NED
horizontal plane, x = North, y = East, heading chi = atan2(y', x'). Every
path starts at the origin heading North (before the East offset), with zero
curvature or (circle) a constant one, so after the start hold and smoothstep
ramp chi_l_dot starts at exactly 0 and Delta grows continuously from 0.

Exact mode: position is r(u) itself (no drift); only u is integrated, from
ds = v dt. Nothing is finite-differenced:

    kappa     = (x' y'' - y' x'') / |r'|^3        (signed, + turns toward +y)
    chi_dot   = kappa * v
    chi_ddot  = kappa * a + v^2 * dkappa/ds

VF mode: d and chi_p come from a warm-started Newton closest-point projection
over u; chi_p_dot and d_dot are analytic (implicit differentiation of the
projection condition). The turn rate

    chi_d   = chi_p - chi_inf (2/pi) atan(K d)
    chi_dot = chi_p_dot - chi_inf (2/pi) K/(1+(Kd)^2) d_dot - beta sat(chi_t/eps)

is scaled by v/v_nom during the speed ramp (arc-length form) so chi_l_dot
starts at exactly 0, and the leader is moved along the exact unicycle arc.
chi_l_ddot is the backward difference of that continuous chi_l_dot -- no
cold-start kick (checked against a 10 kHz reference in step8_ros_harness.py;
one-sample deviation only at the saturation corner |chi_t| = eps). VF gains
(K=0.3, beta=0.08, eps=0.3, omega_max=0.08) are sized for v=0.5 m/s and
|G|=4.24 m: the formation turns leader turn rate w into follower sideways
speed w*g_x, so the paper's beta=pi/4 saturates the followers.

Sinusoid is parametrized as r(u) = (u, f(u)), so the projection and chi_p_dot
reduce exactly to the graph-form y = f(x) expressions used through step 8.
"""

from math import sin, cos, atan, atan2, sqrt, pi, hypot, degrees, inf

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped, Point
    from visualization_msgs.msg import Marker
    from std_msgs.msg import Bool
except ImportError:
    # study/ scripts import the path math below without ROS sourced.
    rclpy = None
    Node = object


def wrap(a):
    return (a + pi) % (2.0 * pi) - pi


def sat(z):
    return max(-1.0, min(1.0, z))


# ======================================================================
# PATHS (ROS-free)
# ======================================================================

class ParametricPath:
    """r(u) in NED plus three derivatives w.r.t. u.

    Subclasses implement _p(u) -> (p, p', p'', p''') in their own frame.
    `theta` rotates that frame so the path starts heading North; `x_off`,
    `y_off`    then shifts the whole path East (VF-mode initial cross-track error).
    """

    theta = 0.0                 # frame rotation [rad]
    y_off = 0.0                 # East shift [m]
    x_off = 0.0                 # North shift [m]
    u_marker = (0.0, 1.0)       # parameter range drawn in RViz / sampled

    def _p(self, u):
        raise NotImplementedError

    def derivs(self, u):
        pts = self._p(u)
        if self.theta != 0.0:
            c, s = cos(self.theta), sin(self.theta)
            pts = [(x * c - y * s, x * s + y * c) for x, y in pts]
        (x0, y0), d1, d2, d3 = pts
        return (x0 + self.x_off, y0 + self.y_off), d1, d2, d3

    def point(self, u):
        return self.derivs(u)[0]

    def heading(self, u):
        _, (x1, y1), _, _ = self.derivs(u)
        return atan2(y1, x1)

    def curvature(self, u):
        """(kappa, dkappa/ds, |r'(u)|)."""
        _, (x1, y1), (x2, y2), (x3, y3) = self.derivs(u)
        sp2 = x1 * x1 + y1 * y1
        sp = sqrt(sp2)
        c = x1 * y2 - y1 * x2
        dc = x1 * y3 - y1 * x3                    # d/du of the cross product
        kappa = c / (sp2 * sp)
        dkappa_du = dc / (sp2 * sp) - 3.0 * c * (x1 * x2 + y1 * y2) / (sp2 * sp2 * sp)
        return kappa, dkappa_du / sp, sp

    def min_turn_radius(self, n=4000):
        u0, u1 = self.u_marker
        kmax = max(abs(self.curvature(u0 + (u1 - u0) * i / n)[0]) for i in range(n + 1))
        return 1.0 / kmax if kmax > 1e-12 else inf

    def project(self, xq, yq, u_guess):
        """Closest point to (xq, yq): (u, signed d along the left normal, chi_p)."""
        u = closest_point(self, xq, yq, u_guess)
        (xs, ys), (x1, y1), _, _ = self.derivs(u)
        chi_p = atan2(y1, x1)
        d = -(xq - xs) * sin(chi_p) + (yq - ys) * cos(chi_p)
        return u, d, chi_p


class SinusoidPath(ParametricPath):
    """y = A (cos(kx) - 1) + y_off, parametrized by u = x. Heads North at x=0."""

    def __init__(self, amplitude=10.0, wavelength=130.0, y_off=0.0):
        self.A = amplitude
        self.k = 2.0 * pi / wavelength
        self.y_off = y_off
        self.u_marker = (-10.0, 2.0 * wavelength)

    def _p(self, u):
        A, k = self.A, self.k
        return ((u, A * (cos(k * u) - 1.0)),
                (1.0, -A * k * sin(k * u)),
                (0.0, -A * k * k * cos(k * u)),
                (0.0, A * k * k * k * sin(k * u)))


class StraightPath(ParametricPath):
    """Due North from the origin."""

    def __init__(self, y_off=0.0):
        self.y_off = y_off
        self.u_marker = (-10.0, 400.0)

    def _p(self, u):
        return (u, 0.0), (1.0, 0.0), (0.0, 0.0), (0.0, 0.0)


class CirclePath(ParametricPath):
    """Radius R, starts at the origin heading North, turns toward East
    (clockwise seen from above) for direction=+1, toward West for -1."""

    def __init__(self, radius=20.0, direction=1.0, y_off=0.0):
        self.R = radius
        self.sgn = 1.0 if direction >= 0 else -1.0
        self.y_off = y_off
        self.u_marker = (0.0, 2.0 * pi)

    def _p(self, u):
        R, g = self.R, self.sgn
        return ((R * sin(u), g * R * (1.0 - cos(u))),
                (R * cos(u), g * R * sin(u)),
                (-R * sin(u), g * R * cos(u)),
                (-R * cos(u), -g * R * sin(u)))


class Figure8Path(ParametricPath):
    """1:2 Lissajous figure-8, x = A sin u, y = B sin 2u, rotated so it starts
    at the crossing point heading North (zero curvature there). A spans the
    long axis, B the lobe half-width; R_min grows with both (A=60, B=20 gives
    R_min = 15 m; a 30 x 15 m shape gives ~3 m, which is unflyable)."""

    def __init__(self, length=60.0, width=20.0, y_off=0.0):
        self.A, self.B = length, width
        self.theta = -atan2(2.0 * width, length)   # start tangent -> North
        self.y_off = y_off
        self.u_marker = (0.0, 2.0 * pi)

    def _p(self, u):
        A, B = self.A, self.B
        return ((A * sin(u), B * sin(2 * u)),
                (A * cos(u), 2 * B * cos(2 * u)),
                (-A * sin(u), -4 * B * sin(2 * u)),
                (-A * cos(u), -8 * B * cos(2 * u)))


PATHS = ('sinusoid', 'figure8', 'circle', 'straight_line')


def make_path(name, y_off=0.0, amplitude=10.0, wavelength=130.0, radius=20.0,
              direction=1.0, f8_length=60.0, f8_width=20.0, x_off=0.0):
    """Build a path by name, shifted (x_off North, y_off East); shared by the
    node and the study/ scripts."""
    p = _make_path(name, y_off, amplitude, wavelength, radius, direction,
                   f8_length, f8_width)
    p.x_off = x_off
    return p


def _make_path(name, y_off, amplitude, wavelength, radius, direction,
               f8_length, f8_width):
    if name == 'sinusoid':
        return SinusoidPath(amplitude, wavelength, y_off)
    if name == 'figure8':
        return Figure8Path(f8_length, f8_width, y_off)
    if name == 'circle':
        return CirclePath(radius, direction, y_off)
    if name == 'straight_line':
        return StraightPath(y_off)
    raise ValueError(f"unknown path '{name}' (choices: {', '.join(PATHS)})")


def closest_point(path, xq, yq, u_guess, iters=8, max_step=2.0):
    """Newton on g(u) = (r(u) - q) . r'(u) = 0, warm-started.

    Each step is capped at `max_step` metres of arc so a closed path can't
    jump to a distant branch (the cap never binds once warm-started).
    """
    u = u_guess
    for _ in range(iters):
        (x, y), (x1, y1), (x2, y2), _ = path.derivs(u)
        g = (x - xq) * x1 + (y - yq) * y1
        gp = x1 * x1 + y1 * y1 + (x - xq) * x2 + (y - yq) * y2
        if abs(gp) < 1e-9:
            break
        step = g / gp
        lim = max_step / max(sqrt(x1 * x1 + y1 * y1), 1e-9)
        u -= max(-lim, min(lim, step))
    return u


# ======================================================================
# LEADERS (ROS-free; shared attribute interface for the harness)
# ======================================================================

class ExactPathLeader:
    """Moves along `path` at the commanded speed; always exactly on it."""

    def __init__(self, path):
        self.path = path
        self.u = 0.0
        self.v = 0.0
        self.v_dot = 0.0
        self.d = 0.0             # exact by construction
        self.chi_t = 0.0
        self.clamped = False
        self._set_state(0.0)

    def _set_state(self, a):
        p = self.path
        self.x, self.y = p.point(self.u)
        self.chi = p.heading(self.u)
        kappa, dkappa_ds, _ = p.curvature(self.u)
        self.kappa = kappa
        self.chi_dot = kappa * self.v
        self.chi_ddot = kappa * a + self.v * self.v * dkappa_ds

    def step(self, dt, v_cmd, a_cmd=None):
        v_prev = self.v
        self.v = v_cmd
        a = (v_cmd - v_prev) / dt if a_cmd is None else a_cmd
        self.v_dot = a
        # arc length this step (trapezoid -- exact for the linear-in-time
        # speed of one ramp step), converted to du with a midpoint |r'|
        ds = 0.5 * (v_prev + v_cmd) * dt
        _, _, sp0 = self.path.curvature(self.u)
        _, _, spm = self.path.curvature(self.u + 0.5 * ds / sp0)
        self.u += ds / spm
        self._set_state(a)


class VFLeader:
    """
    Virtual leader on the paper's vector field (Eq. 5) with the Lemma-1
    saturation reaching law (Eq. 6), integrated directly as a turn rate.

    Two scale-driven additions:
      * omega_max : turn-rate clamp. The formation offsets G_i turn any leader
        turn rate into a follower sideways speed omega*g_x; at v=0.5 m/s and
        |G|=4.24 m, omega above ~0.05 rad/s makes Delta ~ 45 deg+. The paper
        itself (Sec. III-C) notes leader turn rate must be restricted.
      * speed ramp: the whole turn rate is scaled by v_now/v_nom, i.e. the law
        is run in arc length. Path geometry is unchanged, the leader does not
        spin in place while stationary, and Delta is ramp-invariant.
    """

    def __init__(self, path, v=0.5, K=0.3, chi_inf=pi / 2, beta=0.08,
                 eps=0.3, omega_max=0.08, x0=0.0, y0=0.0, chi0=0.0):
        self.path = path
        self.v_nom = v
        self.K, self.chi_inf, self.beta, self.eps = K, chi_inf, beta, eps
        self.omega_max = omega_max
        self.x, self.y, self.chi = x0, y0, chi0
        self.v = 0.0
        self.v_dot = 0.0
        self.chi_dot = 0.0
        self.chi_ddot = 0.0
        self.u = 0.0                      # closest-point warm start
        self.d = 0.0
        self.chi_p = 0.0
        self.chi_t = 0.0
        self.clamped = False
        self._update_geometry()           # sets d, chi_p, chi_t

    def _update_geometry(self):
        self.u, self.d, self.chi_p = self.path.project(self.x, self.y, self.u)
        chi_d = self.chi_p - self.chi_inf * (2.0 / pi) * atan(self.K * self.d)
        self.chi_t = wrap(self.chi - chi_d)

    def turn_rate(self):
        v = self.v
        vx, vy = v * cos(self.chi), v * sin(self.chi)
        (xs, ys), (x1, y1), (x2, y2), _ = self.path.derivs(self.u)
        # implicit differentiation of (r(u) - q) . r'(u) = 0
        gp = x1 * x1 + y1 * y1 + (xs - self.x) * x2 + (ys - self.y) * y2
        u_dot = (vx * x1 + vy * y1) / gp
        chi_p_dot = (x1 * y2 - y1 * x2) / (x1 * x1 + y1 * y1) * u_dot
        d_dot = v * sin(self.chi - self.chi_p)

        chi_d = self.chi_p - self.chi_inf * (2.0 / pi) * atan(self.K * self.d)
        self.chi_t = wrap(self.chi - chi_d)
        ff = chi_p_dot - self.chi_inf * (2.0 / pi) * \
            self.K / (1.0 + (self.K * self.d) ** 2) * d_dot
        # reaching term scaled by v/v_nom (arc-length form, see docstring)
        reach = self.beta * sat(self.chi_t / self.eps) * (v / self.v_nom)
        w = ff - reach
        self.clamped = abs(w) > self.omega_max
        return max(-self.omega_max, min(self.omega_max, w))

    def step(self, dt, v_cmd):
        v_prev, w_prev = self.v, self.chi_dot
        self.v = v_cmd
        w = self.turn_rate()
        # Exact unicycle arc for constant (v, w) over the step. Forward Euler
        # (straight chord along the pre-step heading) cuts every curve and
        # leaves a curvature-shaped |d| floor.
        if abs(w) > 1e-9:
            self.x += self.v / w * (sin(self.chi + w * dt) - sin(self.chi))
            self.y -= self.v / w * (cos(self.chi + w * dt) - cos(self.chi))
        else:
            self.x += self.v * cos(self.chi) * dt
            self.y += self.v * sin(self.chi) * dt
        self.chi = wrap(self.chi + w * dt)
        self._update_geometry()
        self.chi_dot = w
        self.chi_ddot = (w - w_prev) / dt
        self.v_dot = (self.v - v_prev) / dt


# ======================================================================
# ROS NODE
# ======================================================================

class PathLeaderNode(Node):

    def __init__(self):
        super().__init__('path_leader')

        defaults = {
            'path': 'sinusoid',            # sinusoid | figure8 | circle | straight_line
            'path_offset': 0.0,            # [m] East shift; != 0 -> VF mode
            'path_offset_north': 0.0,      # [m] North shift; != 0 -> VF mode
            'speed': 0.5,                  # v_gl [m/s]
            'altitude': -5.0,              # NED, negative = up
            # Start gate: hold at the origin until the follower publishes the
            # latched /formation_engaged (all drones in OFFBOARD), then wait
            # engage_delay. wait_for_formation:=false -> fixed start_delay.
            # Fallback: no flag within engage_timeout of node start -> warn
            # and release anyway (some drone may then not be under guidance).
            'wait_for_formation': True,
            'engage_delay': 3.0,           # [s] after the flag
            'engage_timeout': 60.0,        # [s] from node start
            'start_delay': 20.0,           # [s] hold before moving (gate off)
            'ramp_time': 6.0,              # [s] smoothstep to full speed
            'publish_rate': 100.0,         # [Hz]
            # path shapes
            'amplitude': 10.0, 'wavelength': 130.0,     # sinusoid [m]
            'radius': 20.0, 'direction': 1.0,           # circle [m], +1 East / -1 West
            'f8_length': 60.0, 'f8_width': 20.0,        # figure8 [m]
            # VF-mode gains
            'vf_K': 0.3, 'vf_chi_inf': pi / 2, 'vf_beta': 0.08,
            'vf_eps': 0.3, 'vf_omega_max': 0.08,
            # diagnostics only
            'formation_radius': 4.2426, 'g_x_max': 3.0,
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        g = lambda k: float(self.get_parameter(k).value)  # noqa: E731

        self.path_name = str(self.get_parameter('path').value)
        self.path_offset = g('path_offset')
        self.path_offset_north = g('path_offset_north')
        self.v_nom = g('speed')
        self.z = g('altitude')
        self.start_delay = g('start_delay')
        self.wait_for_formation = bool(self.get_parameter('wait_for_formation').value)
        self.engage_delay = g('engage_delay')
        self.engage_timeout = g('engage_timeout')
        self.engage_timed_out = False
        self.engaged = False
        self.since_engaged = 0.0
        self.ramp_time = g('ramp_time')
        self.dt = 1.0 / g('publish_rate')

        self.path = make_path(self.path_name, self.path_offset,
                              g('amplitude'), g('wavelength'), g('radius'),
                              g('direction'), g('f8_length'), g('f8_width'),
                              x_off=self.path_offset_north)
        self.vf = self.path_offset != 0.0 or self.path_offset_north != 0.0
        if self.vf:
            self.L = VFLeader(self.path, v=self.v_nom, K=g('vf_K'),
                              chi_inf=g('vf_chi_inf'), beta=g('vf_beta'),
                              eps=g('vf_eps'), omega_max=g('vf_omega_max'))
        else:
            self.L = ExactPathLeader(self.path)
        self._clamp_warned = False
        self.elapsed = 0.0
        self.t_move = 0.0

        self.pose_pub = self.create_publisher(PoseStamped, '/virtual_leader_pose', 10)
        self.vel_pub = self.create_publisher(TwistStamped, '/virtual_leader_velocity', 10)
        self.acc_pub = self.create_publisher(AccelStamped, '/virtual_leader_accel', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Marker, '/reference_path_marker', latched)
        self._publish_path_marker()

        if self.wait_for_formation:
            latched_rel = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Bool, '/formation_engaged',
                                     self._on_engaged, latched_rel)

        self.timer = self.create_timer(self.dt, self.tick)

        self._log_geometry()
        if self.vf:
            self.get_logger().info(
                f"[vf] path={self.path_name} path_offset={self.path_offset:+.2f} m path_offset_north={self.path_offset_north:+.2f} m | "
                f"K={g('vf_K')} beta={g('vf_beta')} eps={g('vf_eps')} "
                f"omega_max={g('vf_omega_max')} | d0={self.L.d:+.2f} m, "
                f"chi_tilde0={self.L.chi_t:+.3f} rad")

    def _speed_profile(self, t):
        """C1-continuous smoothstep 3s^2 - 2s^3, and its derivative."""
        T = self.ramp_time
        if T <= 0.0 or t >= T:
            return self.v_nom, 0.0
        s = t / T
        return (self.v_nom * (3.0 * s * s - 2.0 * s ** 3),
                self.v_nom * (6.0 * s - 6.0 * s * s) / T)

    def _on_engaged(self, msg):
        if msg.data and not self.engaged:
            self.engaged = True
            self.get_logger().info(
                f"[start] /formation_engaged received after {self.elapsed:.1f} s; "
                f"moving in {self.engage_delay:.1f} s")

    def _holding(self):
        """True while the leader must stay at the origin."""
        if not self.wait_for_formation:
            return self.elapsed < self.start_delay
        if not self.engaged and self.elapsed >= self.engage_timeout:
            self.engaged = True
            self.engage_timed_out = True
            self.get_logger().warn(
                f"[start] FALLBACK: no /formation_engaged after {self.engage_timeout:.0f} s; "
                f"releasing anyway, moving in {self.engage_delay:.1f} s. Some drones may "
                "not be under guidance -- check the follower log.")
        if not self.engaged:
            n = int(round(self.elapsed / self.dt))
            if n > 0 and n % int(round(10.0 / self.dt)) == 0:
                self.get_logger().info(
                    f"[start] holding at origin: waiting for /formation_engaged "
                    f"({self.elapsed:.0f} s)")
            return True
        return self.since_engaged < self.engage_delay

    def tick(self):
        L = self.L
        if self._holding():
            # Hold at the origin heading North, no rotation: followers see
            # Delta = 0, so guidance can engage with no initial mismatch.
            self.elapsed += self.dt
            if self.engaged:
                self.since_engaged += self.dt
            v, a = 0.0, 0.0
            chi_dot = chi_ddot = 0.0
        else:
            v, a = self._speed_profile(self.t_move)
            if self.vf:
                L.step(self.dt, v)
                if L.clamped and not self._clamp_warned:
                    self.get_logger().warn('[vf] leader turn-rate clamp omega_max active')
                    self._clamp_warned = True
            else:
                L.step(self.dt, v, a)
            self.t_move += self.dt
            chi_dot, chi_ddot = L.chi_dot, L.chi_ddot

        stamp = self.get_clock().now().to_msg()
        c, s = cos(L.chi), sin(L.chi)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = L.x, L.y, self.z
        pose.pose.orientation.z, pose.pose.orientation.w = sin(L.chi / 2), cos(L.chi / 2)
        self.pose_pub.publish(pose)

        tw = TwistStamped()
        tw.header.stamp = stamp
        tw.twist.linear.x, tw.twist.linear.y = v * c, v * s
        tw.twist.angular.z = chi_dot
        self.vel_pub.publish(tw)

        ac = AccelStamped()
        ac.header.stamp = stamp
        ac.accel.linear.x = a * c - v * s * chi_dot
        ac.accel.linear.y = a * s + v * c * chi_dot
        ac.accel.angular.z = chi_ddot
        self.acc_pub.publish(ac)

    def _publish_path_marker(self):
        m = Marker()
        m.header.frame_id = 'map'
        m.ns, m.id = 'reference_path', 0
        m.type, m.action = Marker.LINE_STRIP, Marker.ADD
        m.scale.x = 0.05
        m.color.a = m.color.r = m.color.g = m.color.b = 1.0
        m.pose.orientation.w = 1.0
        u0, u1 = self.path.u_marker
        for i in range(401):
            x, y = self.path.point(u0 + (u1 - u0) * i / 400)
            m.points.append(Point(x=y, y=x, z=-self.z))          # NED -> ENU
        self.path_pub.publish(m)

    def _log_geometry(self):
        """Feasibility for the formation: Delta = atan(g_x / (R - g_y))."""
        gnorm = float(self.get_parameter('formation_radius').value)
        gx = float(self.get_parameter('g_x_max').value)
        gy = gx
        R = self.path.min_turn_radius()
        w = self.v_nom / R if R < inf else 0.0
        delta = (0.0 if R == inf else
                 degrees(atan(gx / (R - gy))) if R > gy else float('nan'))
        self.get_logger().info(
            f"[{self.path_name}] mode={'vf' if self.vf else 'exact'} "
            f"v={self.v_nom:.2f} m/s | R_min={R:.1f} m | R_min/|G|={R / gnorm:.1f} | "
            f"omega_max={w:.4f} rad/s | Delta_max={delta:.1f} deg | "
            f"omega*g_y/v={w * gy / max(self.v_nom, 1e-6):.2f}")
        # Delta's singularity is omega*g_y = v_gl; past ~0.5 the slot is close
        # enough to the turn centre that Delta and Eq. 21 get ill-conditioned.
        if w * gy >= 0.5 * self.v_nom:
            self.get_logger().warn(
                f"[{self.path_name}] omega_max*g_y = {w * gy:.3f} is over half of "
                f"v_gl = {self.v_nom:.2f}: inner slots pass close to the leader's "
                "turn centre; Delta and Eq. 21 are ill-conditioned. Enlarge the path.")


def main(args=None):
    rclpy.init(args=args)
    node = PathLeaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
