#!/usr/bin/env bash
# start_kap_stack.sh
#
# Starts:
#   1) franka_control (NOT in conda env)
#   2) clarius-driver-node.py (in conda env "Navid")
#   3) acquisition-controller.py (in conda env "Navid")
#
# Assumes: roscore already running.

set -euo pipefail

# -----------------------------
# Defaults
# -----------------------------
LD_PRELOAD_PATH_DEFAULT="/usr/lib/x86_64-linux-gnu/libffi.so.7"
PROBE_IP="192.168.1.1"
ROBOT_IP="172.31.1.149"
PORT=""

# Extra args passed through to clarius-driver-node.py (besides -a/-p we manage)
DRIVER_EXTRA_ARGS=()

# -----------------------------
# Helpers
# -----------------------------
usage() {
  cat <<EOF
Usage:
  $(basename "$0") -p <port> [-a <probe_ip>] [--robot-ip <robot_ip>] [--ld-preload <path>] [--] [driver_extra_args...]

Examples:
  $(basename "$0") -p 12345
  $(basename "$0") -a 192.168.1.1 -p 12345 --robot-ip 172.31.1.149
  $(basename "$0") -p 12345 -- --some_driver_flag foo

Notes:
  - roscore is NOT started by this script.
  - franka_control runs outside conda.
  - clarius driver + acquisition controller run inside conda env "Navid".
EOF
}

die() { echo "ERROR: $*" >&2; exit 1; }

need_cmd() { command -v "$1" >/dev/null 2>&1 || die "Missing command: $1"; }

# Determine script directory (this file lives in .../clarius_ultrasound/scripts)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Find catkin_ws root by walking up until we see devel/setup.bash
find_catkin_root() {
  local d="$SCRIPT_DIR"
  while [[ "$d" != "/" ]]; do
    if [[ -f "$d/devel/setup.bash" ]]; then
      echo "$d"
      return 0
    fi
    d="$(dirname "$d")"
  done
  return 1
}

CATKIN_ROOT="$(find_catkin_root || true)"
[[ -n "${CATKIN_ROOT:-}" ]] || die "Could not find catkin workspace root (missing devel/setup.bash above $SCRIPT_DIR)."

# Try to locate conda.sh robustly
find_conda_sh() {
  if command -v conda >/dev/null 2>&1; then
    local base
    base="$(conda info --base 2>/dev/null || true)"
    if [[ -n "$base" && -f "$base/etc/profile.d/conda.sh" ]]; then
      echo "$base/etc/profile.d/conda.sh"
      return 0
    fi
  fi

  # Common fallbacks
  for p in \
    "$HOME/miniconda3/etc/profile.d/conda.sh" \
    "$HOME/anaconda3/etc/profile.d/conda.sh" \
    "/opt/conda/etc/profile.d/conda.sh"
  do
    if [[ -f "$p" ]]; then
      echo "$p"
      return 0
    fi
  done

  return 1
}

# -----------------------------
# Arg parsing
# -----------------------------
LD_PRELOAD_PATH="$LD_PRELOAD_PATH_DEFAULT"

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    -a|--ip)
      [[ $# -ge 2 ]] || die "Missing value for $1"
      PROBE_IP="$2"
      shift 2
      ;;
    -p|--port)
      [[ $# -ge 2 ]] || die "Missing value for $1"
      PORT="$2"
      shift 2
      ;;
    --robot-ip)
      [[ $# -ge 2 ]] || die "Missing value for $1"
      ROBOT_IP="$2"
      shift 2
      ;;
    --ld-preload)
      [[ $# -ge 2 ]] || die "Missing value for $1"
      LD_PRELOAD_PATH="$2"
      shift 2
      ;;
    --) # remaining args -> driver extras
      shift
      while [[ $# -gt 0 ]]; do
        DRIVER_EXTRA_ARGS+=("$1")
        shift
      done
      ;;
    *) # unknown -> treat as driver extra
      DRIVER_EXTRA_ARGS+=("$1")
      shift
      ;;
  esac
done

[[ -n "$PORT" ]] || { usage; die "Port is required (-p <port>)."; }
[[ "$PORT" =~ ^[0-9]+$ ]] || die "Port must be an integer, got: $PORT"
[[ -f "$LD_PRELOAD_PATH" ]] || die "LD_PRELOAD library not found: $LD_PRELOAD_PATH"

# -----------------------------
# Preconditions
# -----------------------------
need_cmd roslaunch

# Source ROS + catkin workspace (required for all nodes)
# (Assumes the ROS distro environment is already in your shell, but sourcing devel/setup.bash is still needed.)
# If you also need /opt/ros/<distro>/setup.bash, add it before this line.
# shellcheck disable=SC1090
source "$CATKIN_ROOT/devel/setup.bash"

# Ensure the Clarius shared libs are where clarius-driver-node.py expects them (./libcast.so)
[[ -f "$SCRIPT_DIR/libcast.so" ]] || die "Missing $SCRIPT_DIR/libcast.so"
[[ -f "$SCRIPT_DIR/pyclariuscast.so" ]] || die "Missing $SCRIPT_DIR/pyclariuscast.so"
[[ -f "$SCRIPT_DIR/clarius-driver-node.py" ]] || die "Missing $SCRIPT_DIR/clarius-driver-node.py"
[[ -f "$SCRIPT_DIR/acquisition-controller.py" ]] || die "Missing $SCRIPT_DIR/acquisition-controller.py"

CONDA_SH="$(find_conda_sh || true)"
[[ -n "${CONDA_SH:-}" ]] || die "Could not locate conda.sh. Ensure conda is installed and accessible."

# -----------------------------
# Process management
# -----------------------------
PIDS=()

cleanup() {
  # Kill children on exit / ctrl-c
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill "$pid" >/dev/null 2>&1 || true
    fi
  done
  # Give them a moment, then force kill if needed
  sleep 0.5
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill -9 "$pid" >/dev/null 2>&1 || true
    fi
  done
}
trap cleanup EXIT INT TERM

# -----------------------------
# 1) Start Franka control (outside conda)
# -----------------------------
echo "[kap] Starting franka_control (robot_ip=$ROBOT_IP) ..."
roslaunch franka_control franka_control.launch "robot_ip:=$ROBOT_IP" &
FRANKA_PID=$!
PIDS+=("$FRANKA_PID")

# -----------------------------
# 2) Start Clarius driver + acquisition controller (inside conda env)
# -----------------------------
echo "[kap] Starting Clarius driver + acquisition controller in conda env 'Navid' ..."
(
  set -euo pipefail

  # Activate conda
  # shellcheck disable=SC1090
  source "$CONDA_SH"
  conda activate Navid

  # Required preload (and preserve any existing LD_PRELOAD)
  export LD_PRELOAD="$LD_PRELOAD_PATH${LD_PRELOAD:+:${LD_PRELOAD}}"

  cd "$SCRIPT_DIR"

  echo "[kap]   clarius-driver-node.py --ip $PROBE_IP --port $PORT ${DRIVER_EXTRA_ARGS[*]:-}"
  python clarius-driver-node.py --ip "$PROBE_IP" --port "$PORT" "${DRIVER_EXTRA_ARGS[@]:-}" &
  DRIVER_PID=$!

  echo "[kap]   acquisition-controller.py"
  python acquisition-controller.py &
  ACQ_PID=$!

  # Wait until one exits; propagate exit code
  wait "$DRIVER_PID" "$ACQ_PID"
) &
CONDA_GROUP_PID=$!
PIDS+=("$CONDA_GROUP_PID")

# Keep script in foreground until something exits or user interrupts
wait "$FRANKA_PID" "$CONDA_GROUP_PID"