#!/usr/bin/env bash
#
# start_swarm.sh -- launch PX4 SITL multi-vehicle swarm + ROS 2 guidance stack.
#
# Stack: ROS 2 Jazzy / Gazebo Harmonic / PX4 v1.17.0 / Micro-XRCE-DDS-Agent v2.4.3
#
# Usage:
#   ./start_swarm.sh                                  # sinusoid leader (exact mode), rviz on
#   ./start_swarm.sh --leader circle --no-rviz
#   ./start_swarm.sh --leader figure8 --path-offset 5 # VF mode on any path
#   ./start_swarm.sh --path-offset 10,10               # shift North 10, East 10
#   ./start_swarm.sh --n 1 --leader straight_line     # single-drone harness
#   ./start_swarm.sh --log-dir ~/swarm_logs
#
# Carousel A/B (two runs differing ONLY in the Delta correction):
#   ./start_swarm.sh --leader sinusoid --wavelength 100 --delta off --tag nodelta
#   ./start_swarm.sh --leader sinusoid --wavelength 100 --delta on  --tag delta
#
# Step 6/8 -- leader flies onto the path from 5 m off (VF mode; works with any --leader):
#   ./start_swarm.sh --leader sinusoid --path-offset 5 --delta off                --tag vf_none
#   ./start_swarm.sh --leader sinusoid --path-offset 5 --delta on                 --tag vf_full
#   ./start_swarm.sh --leader sinusoid --path-offset 5 --delta on --delta-dot off --tag vf_pw
#
set -euo pipefail

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
PX4_DIR="${PX4_DIR:-$HOME/tools/PX4-Autopilot}"
ROS_WS="${ROS_WS:-$HOME/ros2_ws}"
N_DRONES=4
LEADER="sinusoid"
USE_RVIZ="true"
LOG_DIR="${HOME}/swarm_logs/$(date +%Y%m%d_%H%M%S)"
GZ_MODEL="x500"
PX4_AIRFRAME=4001

# Valid leader node names. Must match the choices handled in swarm.launch.py.
VALID_LEADERS=(sinusoid figure8 circle straight_line)

# --- carousel / path options (passed through to swarm.launch.py) ------------
ENABLE_DELTA="true"     # follower's carousel heading correction (Delta)
ENABLE_DELTA_DOT="true" # follower's dDelta/dt feedforward (off = pointwise)
PATH_OFFSET=""          # path shift East [m]; blank -> exact mode (any leader)
PATH_OFFSET_NORTH=""    # path shift North [m] (from --path-offset N,E)
LOG_TAG=""              # appended to the logger's CSV filename
LEADER_SPEED=""         # blank -> use the node's own default
LEADER_AMPLITUDE=""
LEADER_WAVELENGTH=""

declare -A SPAWN_NED=(
  [1]="3.0 3.0"
  [2]="-3.0 3.0"
  [3]="3.0 -3.0"
  [4]="-3.0 -3.0"
)
SPAWN_HEADING_NED=0.0   # all drones face North at spawn

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
usage() {
  sed -n '2,20p' "$0" | sed 's/^# \?//'
  echo
  echo "Leaders: ${VALID_LEADERS[*]}"
  echo "Carousel options:"
  echo "  --delta on|off        follower Delta correction   (default: on)"
  echo "  --tag NAME            suffix for the logger CSV    (default: none)"
  echo "  --speed M_PER_S       leader ground speed"
  echo "  --amplitude M         sinusoid amplitude A"
  echo "  --wavelength M        sinusoid wavelength"
  echo "  --delta-dot on|off    dDelta/dt feedforward        (default: on)"
  echo "  --path-offset E|N,E   shift the path East (E) or North,East (N,E) [m]: leader flies onto it"
  echo "                        (VF mode, any leader; 0 = exact). Leader always starts at the origin."
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --n)          N_DRONES="$2"; shift 2 ;;
    --leader)     LEADER="$2";   shift 2 ;;
    --log-dir)    LOG_DIR="$2";  shift 2 ;;
    --rviz)       USE_RVIZ="true";  shift ;;
    --no-rviz)    USE_RVIZ="false"; shift ;;
    --delta)
      case "$2" in
        on|true|1)   ENABLE_DELTA="true" ;;
        off|false|0) ENABLE_DELTA="false" ;;
        *) echo "ERROR: --delta expects on|off, got '$2'" >&2; exit 1 ;;
      esac
      shift 2 ;;
    --delta-dot)
      case "$2" in
        on|true|1)   ENABLE_DELTA_DOT="true" ;;
        off|false|0) ENABLE_DELTA_DOT="false" ;;
        *) echo "ERROR: --delta-dot expects on|off, got '$2'" >&2; exit 1 ;;
      esac
      shift 2 ;;
    --path-offset)
      if [[ "$2" == *,* ]]; then          # N,E
        PATH_OFFSET_NORTH="${2%%,*}"; PATH_OFFSET="${2#*,}"
      else                                # E only (backward compatible)
        PATH_OFFSET="$2"
      fi
      shift 2 ;;
    --tag)        LOG_TAG="$2";            shift 2 ;;
    --speed)      LEADER_SPEED="$2";       shift 2 ;;
    --amplitude)  LEADER_AMPLITUDE="$2";   shift 2 ;;
    --wavelength) LEADER_WAVELENGTH="$2";  shift 2 ;;
    -h|--help)    usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; exit 1 ;;
  esac
done

if (( N_DRONES < 1 || N_DRONES > 4 )); then
  echo "ERROR: --n must be 1..4 (spawn table has 4 entries)" >&2
  exit 1
fi

# Fail fast on a bad leader name instead of dying inside ros2 launch, where
# the error is buried in swarm_launch.log.
leader_ok="false"
for l in "${VALID_LEADERS[@]}"; do
  [[ "$LEADER" == "$l" ]] && leader_ok="true"
done
if [[ "$leader_ok" != "true" ]]; then
  echo "ERROR: --leader must be one of: ${VALID_LEADERS[*]} (got '$LEADER')" >&2
  exit 1
fi

# Amplitude/wavelength shape only the sinusoid; --path-offset works for any path.
if [[ "$LEADER" != "sinusoid" ]] \
   && { [[ -n "$LEADER_AMPLITUDE" ]] || [[ -n "$LEADER_WAVELENGTH" ]]; }; then
  echo "WARNING: --amplitude/--wavelength are ignored for leader '$LEADER'" >&2
fi

mkdir -p "$LOG_DIR"
echo "Logs -> $LOG_DIR"
echo "Leader: $LEADER | Delta correction: $ENABLE_DELTA${LOG_TAG:+ | tag: $LOG_TAG}"

# Record the run configuration alongside the logs. Without this, two CSVs from
# an A/B are indistinguishable a week later.
{
  echo "date=$(date -Is)"
  echo "leader=$LEADER"
  echo "n_drones=$N_DRONES"
  echo "enable_delta=$ENABLE_DELTA"
  echo "enable_delta_dot=$ENABLE_DELTA_DOT"
  echo "path_offset=${PATH_OFFSET:-<node default>}"
  echo "path_offset_north=${PATH_OFFSET_NORTH:-<node default>}"
  echo "tag=$LOG_TAG"
  echo "speed=${LEADER_SPEED:-<node default>}"
  echo "amplitude=${LEADER_AMPLITUDE:-<node default>}"
  echo "wavelength=${LEADER_WAVELENGTH:-<node default>}"
  echo "git_rev=$(git -C "$ROS_WS/src/swarm_formation" rev-parse --short HEAD 2>/dev/null || echo unknown)"
} > "$LOG_DIR/run_config.txt"

# ---------------------------------------------------------------------------
# Cleanup: kill everything we started, on exit or Ctrl-C
# ---------------------------------------------------------------------------
PIDS=()
cleanup() {
  echo
  echo "Shutting down..."
  for pid in "${PIDS[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
  # PX4 runs inside a subshell; killing the subshell does not always reap the
  # px4 binary itself, and a survivor holds UDP 8888 / the Gazebo world and
  # makes the NEXT run fail in confusing ways. Belt and braces.
  pkill -f 'build/px4_sitl_default/bin/px4' 2>/dev/null || true
  # Instance 1 spawns the Gazebo server + GUI detached; they outlive PX4.
  pkill -f '^gz sim' 2>/dev/null || true   # anchored: don't hit shells quoting it
  # The GUI (gz sim -g) can ignore SIGTERM and survive into the next run.
  for _ in 1 2 3; do pgrep -f '^gz sim' >/dev/null || break; sleep 1; done
  pkill -KILL -f '^gz sim' 2>/dev/null || true
  wait 2>/dev/null || true
  echo "Logs remain in $LOG_DIR"
}
trap cleanup EXIT INT TERM

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

# ned_to_enu_pose <north> <east> -> "x,y,z,roll,pitch,yaw" ENU string
ned_to_enu_pose() {
  local north="$1" east="$2"
  python3 - "$north" "$east" "$SPAWN_HEADING_NED" <<'PY'
import math, sys
north, east, chi = (float(a) for a in sys.argv[1:4])
enu_x, enu_y, enu_z = east, north, 0.0
enu_yaw = math.pi / 2.0 - chi
print(f"{enu_x},{enu_y},{enu_z},0,0,{enu_yaw:.6f}")
PY
}

# wait_for <description> <timeout_s> <command...>
wait_for() {
  local desc="$1" timeout="$2"; shift 2
  local deadline=$(( SECONDS + timeout ))
  printf "  waiting for %s " "$desc"
  while (( SECONDS < deadline )); do
    if "$@" >/dev/null 2>&1; then
      printf " ok (%ds)\n" "$(( timeout - (deadline - SECONDS) ))"
      return 0
    fi
    printf "."
    sleep 0.5
  done
  printf " TIMEOUT after %ds\n" "$timeout"
  echo "ERROR: gave up waiting for $desc. Check $LOG_DIR" >&2
  return 1
}

agent_is_up()   { ss -lun 2>/dev/null | grep -q ':8888'; }
gazebo_is_up()  { gz topic -l 2>/dev/null | grep -q '^/world/'; }
px4_topic_is_up() {
  # PX4 >= v1.16 appends _v<N> to versioned messages (vehicle_local_position_v1).
  ros2 topic list 2>/dev/null | grep -qE "^/px4_$1/fmu/out/vehicle_local_position(_v[0-9]+)?$"
}

# ---------------------------------------------------------------------------
# 0. Environment
#
# The readiness probe above calls `ros2`, so ROS must be sourced BEFORE PX4
# starts -- not left to the caller's shell. In a fresh terminal `ros2` is
# not on PATH, the probe's 2>/dev/null hides "command not found", and the
# script waits forever on /px4_1/... even though PX4 is up and publishing.
# ---------------------------------------------------------------------------
# colcon's setup scripts read unset vars (COLCON_TRACE, ...): relax -u here.
set +u
# shellcheck disable=SC1090,SC1091
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
# shellcheck disable=SC1091
source "$ROS_WS/install/setup.bash"
set -u
if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: 'ros2' not found after sourcing ROS ${ROS_DISTRO:-jazzy} and $ROS_WS" >&2
  exit 1
fi
# Pin gz-transport discovery to loopback: multicast on the Wi-Fi interface
# fails with 'Network is unreachable' whenever the link drops, and Gazebo
# then never reports its world. Override by exporting GZ_IP yourself.
export GZ_IP="${GZ_IP:-127.0.0.1}"
echo "ROS ${ROS_DISTRO} | ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} | GZ_IP=$GZ_IP"

# ---------------------------------------------------------------------------
# 1. Micro-XRCE-DDS Agent
# ---------------------------------------------------------------------------
echo "[1/3] Starting Micro-XRCE-DDS Agent..."
MicroXRCEAgent udp4 -p 8888 > "$LOG_DIR/agent.log" 2>&1 &
PIDS+=($!)
wait_for "agent on UDP 8888" 20 agent_is_up

# ---------------------------------------------------------------------------
# 2. PX4 SITL instances
#
# Instance 1 launches Gazebo. Instances 2..N attach to the already-running
# server and MUST set PX4_GZ_STANDALONE=1 -- previously this flag was on
# instance 1 instead, so every instance raced to create the world.
# ---------------------------------------------------------------------------
echo "[2/3] Starting $N_DRONES PX4 SITL instance(s)..."
for (( i = 1; i <= N_DRONES; i++ )); do
  read -r north east <<< "${SPAWN_NED[$i]}"
  pose="$(ned_to_enu_pose "$north" "$east")"

  # PX4's rc.gzsim only checks `[ -z "$PX4_GZ_STANDALONE" ]` -- a string-
  # emptiness test, not a boolean one. "0" is non-empty, so it still reads
  # as "standalone". Instance 1 (which must launch Gazebo) needs the var
  # truly unset/empty; only instances 2..N (which attach to it) set "1".
  standalone=1
  if (( i == 1 )); then
    standalone=""   # instance 1 owns the Gazebo server
  fi

  printf "  drone %d: NED(%s, %s) -> ENU pose %s\n" "$i" "$north" "$east" "$pose"

  (
    cd "$PX4_DIR"
    PX4_GZ_STANDALONE=$standalone \
    PX4_SYS_AUTOSTART=$PX4_AIRFRAME \
    PX4_GZ_MODEL="$GZ_MODEL" \
    PX4_GZ_MODEL_POSE="$pose" \
    ./build/px4_sitl_default/bin/px4 -d -i "$i"   # -d: no pxh shell (it spins on a background tty)
  ) > "$LOG_DIR/px4_$i.log" 2>&1 &
  PIDS+=($!)

  if (( i == 1 )); then
    wait_for "Gazebo world" 90 gazebo_is_up
  fi
  wait_for "/px4_$i/fmu/out/vehicle_local_position" 90 px4_topic_is_up "$i"
done

# ---------------------------------------------------------------------------
# 3. ROS 2 guidance stack
# ---------------------------------------------------------------------------
echo "[3/3] Starting ROS 2 swarm launch..."
# (ROS + workspace already sourced in step 0.)

# Build the launch argument list. Optional path args are only appended when
# set, so the node's own defaults stay authoritative otherwise.
LAUNCH_ARGS=(
  leader:="$LEADER"
  rviz:="$USE_RVIZ"
  n_drones:="$N_DRONES"
  log_dir:="$LOG_DIR"
  enable_delta:="$ENABLE_DELTA"
  enable_delta_dot:="$ENABLE_DELTA_DOT"
)
# Empty values are optional args: `ros2 launch` rejects `name:=` outright
# ("malformed launch argument"), killing the launch before any node starts.
[[ -n "$LOG_TAG"           ]] && LAUNCH_ARGS+=( log_tag:="$LOG_TAG" )
[[ -n "$LEADER_SPEED"      ]] && LAUNCH_ARGS+=( leader_speed:="$LEADER_SPEED" )
[[ -n "$LEADER_AMPLITUDE"  ]] && LAUNCH_ARGS+=( leader_amplitude:="$LEADER_AMPLITUDE" )
[[ -n "$LEADER_WAVELENGTH" ]] && LAUNCH_ARGS+=( leader_wavelength:="$LEADER_WAVELENGTH" )
[[ -n "$PATH_OFFSET"       ]] && LAUNCH_ARGS+=( path_offset:="$PATH_OFFSET" )
[[ -n "$PATH_OFFSET_NORTH" ]] && LAUNCH_ARGS+=( path_offset_north:="$PATH_OFFSET_NORTH" )

ros2 launch swarm_formation swarm.launch.py "${LAUNCH_ARGS[@]}" \
  > "$LOG_DIR/swarm_launch.log" 2>&1 &
LAUNCH_PID=$!
PIDS+=($LAUNCH_PID)

# A launch that dies at startup (bad argument, missing executable, ...) would
# otherwise leave PX4 idling "Ready for takeoff" with nobody arming it.
sleep 5
if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
  echo "ERROR: ros2 launch exited during startup. Last lines of swarm_launch.log:" >&2
  tail -n 15 "$LOG_DIR/swarm_launch.log" >&2
  exit 1
fi

echo
echo "Swarm running. Ctrl-C to stop."
echo "  tail -f $LOG_DIR/swarm_launch.log"
wait