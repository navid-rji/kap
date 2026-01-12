#!/usr/bin/env bash
# start_kap_stack.sh
#
# Starts:
#   1) franka_control (NOT in conda env)
#   2) clarius_driver_node.py (in conda env "Navid")
#   3) acquisition_controller.py or tui.py (in conda env "Navid")
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

# Extra args passed through to clarius_driver_node.py (besides -a/-p we manage)
DRIVER_EXTRA_ARGS=()
INTERFACE_SCRIPT=""
INTERFACE_DESC=""

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
  - clarius driver + CLI/TUI run inside conda env "Navid".
EOF
}

die() { echo "ERROR: $*" >&2; exit 1; }

need_cmd() { command -v "$1" >/dev/null 2>&1 || die "Missing command: $1"; }

set_interface_choice() {
  local lowered
  lowered="$(printf '%s' "$1" | tr '[:upper:]' '[:lower:]')"
  case "$lowered" in
    c|cli)
      INTERFACE_SCRIPT="acquisition_controller.py"
      INTERFACE_DESC="CLI"
      return 0
      ;;
    t|tui)
      INTERFACE_SCRIPT="tui.py"
      INTERFACE_DESC="TUI"
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

prompt_interface_choice() {
  if [[ -n "${KAP_INTERFACE:-}" ]]; then
    set_interface_choice "$KAP_INTERFACE" || die "Invalid KAP_INTERFACE (use 'cli' or 'tui')."
    return 0
  fi

  if [[ ! -t 0 ]]; then
    die "Non-interactive session detected. Set KAP_INTERFACE=cli or KAP_INTERFACE=tui to skip the prompt."
  fi

  local choice
  while true; do
    read -rp "[kap] Select interface ([c]li/[t]ui): " choice || continue
    if set_interface_choice "$choice"; then
      break
    fi
    echo "Please answer 'cli' or 'tui'." >&2
  done
}

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

# Ensure the Clarius shared libs are where clarius_driver_node.py expects them (./libcast.so)
[[ -f "$SCRIPT_DIR/libcast.so" ]] || die "Missing $SCRIPT_DIR/libcast.so"
[[ -f "$SCRIPT_DIR/pyclariuscast.so" ]] || die "Missing $SCRIPT_DIR/pyclariuscast.so"
[[ -f "$SCRIPT_DIR/clarius_driver_node.py" ]] || die "Missing $SCRIPT_DIR/clarius_driver_node.py"
[[ -f "$SCRIPT_DIR/acquisition_controller.py" ]] || die "Missing $SCRIPT_DIR/acquisition_controller.py"
[[ -f "$SCRIPT_DIR/tui.py" ]] || die "Missing $SCRIPT_DIR/tui.py"

prompt_interface_choice

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
# 2) Start Clarius driver + selected interface (inside conda env)
# -----------------------------
echo "[kap] Starting Clarius driver + $INTERFACE_DESC interface in conda env 'Navid' ..."
(
  set -euo pipefail

  # Activate conda
  # shellcheck disable=SC1090
  source "$CONDA_SH"
  conda activate Navid

  # Required preload (and preserve any existing LD_PRELOAD)
  export LD_PRELOAD="$LD_PRELOAD_PATH${LD_PRELOAD:+:${LD_PRELOAD}}"

  cd "$SCRIPT_DIR"

  DRIVER_CMD=(python clarius_driver_node.py --ip "$PROBE_IP" --port "$PORT")
  if ((${#DRIVER_EXTRA_ARGS[@]})); then
    DRIVER_CMD+=("${DRIVER_EXTRA_ARGS[@]}")
  fi

  INTERFACE_CMD=(python "$INTERFACE_SCRIPT")

  DRIVER_PID=""
  conda_cleanup() {
    if [[ -n "${DRIVER_PID:-}" ]] && kill -0 "$DRIVER_PID" >/dev/null 2>&1; then
      kill "$DRIVER_PID" >/dev/null 2>&1 || true
      wait "$DRIVER_PID" >/dev/null 2>&1 || true
    fi
  }
  trap conda_cleanup EXIT INT TERM

  echo "[kap]   ${DRIVER_CMD[*]}"
  "${DRIVER_CMD[@]}" &
  DRIVER_PID=$!

  echo "[kap]   $INTERFACE_SCRIPT ($INTERFACE_DESC foreground)"
  "${INTERFACE_CMD[@]}"
) &
CONDA_GROUP_PID=$!
PIDS+=("$CONDA_GROUP_PID")

# Keep script in foreground until something exits or user interrupts
wait "$FRANKA_PID" "$CONDA_GROUP_PID"