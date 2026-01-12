## KAP – Clarius ↔ Franka Acquisition Pipeline

KAP connects a Clarius ultrasound probe to a Franka Emika Panda robot so that synchronized B-mode frames, poses, and wrench data can be collected during scripted sweeps. The package contains purely-Python ROS nodes, a convenience launcher, and an optional Textual TUI for driving interactive acquisitions.

---

### Features

- ROS interface to the Clarius `pyclariuscast` SDK (`scripts/clarius_driver_node.py`).
- Interactive acquisition CLI that performs frequency/focus sweeps, stores frames, and logs robot pose/wrench metadata (`scripts/acquisition_controller.py`).
- Optional terminal user interface built with Textual (`scripts/tui.py`).
- Stack bootstrapper that starts Franka control, Clarius driver, and acquisition controller together (`scripts/start_kap_stack.sh`).

---

### Repository Layout

- `scripts/`
  - `clarius_driver_node.py`: Connects to the probe and republishes B-mode frames on `/clarius/bmode` while exposing TX controls.
  - `acquisition_controller.py`: Command-line client that toggles freeze state, runs parameter sweeps, and writes images/metadata.
  - `tui.py`: Optional Textual dashboard sitting on top of the controller logic.
  - `start_kap_stack.sh`: Helper script that wires everything together (expects a `Navid` conda env).
- `CMakeLists.txt` / `package.xml`: Minimal ROS `catkin` package definition (`clarius_ultrasound`).

---

### Prerequisites

**Hardware**

- Clarius research ultrasound probe reachable via Ethernet/Wi-Fi.
- Franka Emika with `franka_control` stack running (robot IP defaults to `172.31.1.149`).

**Software**

- ROS (tested with Noetic on Ubuntu 20.04; other ROS 1 distros should work).
- `catkin` workspace containing this package (`kap`).
- Clarius `libcast.so`, `pyclariuscast.so`
- Python dependencies installed in your active environment (recommend a conda env):
  - `rospy`, `sensor_msgs`, `geometry_msgs`, `franka_msgs`, `tf`, `numpy`, `opencv-python`, `textual` (only for the TUI).
- Export `LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libffi.so.7` before starting the Clarius driver (required by the vendor SDK).

ROS dependencies are declared in [package.xml](package.xml) (`franka_ros`, `moveit_commander`, `cv_bridge`, etc.). Make sure they are available in your workspace.

---

### Build & Workspace Setup

1. Clone the repository into your catkin workspace `src/` directory.
2. Run `catkin_make` (even though the package is Python-only, this generates the env hooks).
3. Source your workspace: `source ~/catkin_ws/devel/setup.bash`.

The launcher script expects to find `devel/setup.bash` by walking up from `scripts/`, so keep the default layout.

---

### Runtime Environment

```bash
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libffi.so.7
source ~/catkin_ws/devel/setup.bash
```

Ensure `scripts/libcast.so` and `scripts/pyclariuscast.so` match the architecture of your machine (typically `x86_64`).

---

### Quick Start (Manual)

1. **ROS master**
   ```bash
   roscore
   ```
2. **Franka controller**
   ```bash
   roslaunch franka_control franka_control.launch robot_ip:=172.31.1.149
   ```
   Replace `robot_ip` if your arm uses a different address.
3. **Clarius driver**
   ```bash
   rosrun kap clarius_driver_node.py -p <probe_port> --ip 192.168.1.1
   ```
   The driver publishes `/clarius/bmode`, `/clarius/tx_freq_ack`, and `/clarius/tx_focus_ack` while accepting control topics.
4. **Acquisition controller**
   ```bash
   rosrun kap acquisition_controller.py
   ```
   Type `h` in the prompt to list all commands. Use `r both` to run a nested frequency/focus sweep and save frames under `clarius_acquisitions/`.

---

### Quick Start (Automated Script)

The helper script spins up Franka control, the Clarius driver, and the acquisition controller. It assumes:

- You are **already running** `roscore`.
- The Clarius binaries live in `scripts/`.
- A conda env named `Navid` hosts the Python dependencies.

Run:

```bash
./scripts/start_kap_stack.sh -p <probe_port> [-a <probe_ip>] [--robot-ip <robot_ip>] [--ld-preload <libffi_path>] [-- driver_extra_args]
```

Examples:

- `./scripts/start_kap_stack.sh -p 12345`
- `./scripts/start_kap_stack.sh -p 12345 -a 192.168.1.10 --robot-ip 172.31.1.200`

The script will:

1. Source your catkin workspace.
2. Validate the Clarius shared libraries in `scripts/`.
3. Activate the `Navid` conda env and prepend `LD_PRELOAD`.
4. Launch `franka_control`, `clarius_driver_node.py`, and `acquisition_controller.py` (using `python` inside the conda env).

Use `Ctrl+C` to tear down the entire stack; the script traps signals and stops every child process.

---

### Acquisition Controller Cheat Sheet

Once `scripts/acquisition_controller.py` is running, you get an interactive prompt. Key commands:

| Command  | Description                  |
| -------- | ---------------------------- | ----------- | ----------------------------------------------------------------- |
| `h`      | Show help text.              |
| `f`      | Toggle Clarius freeze state. |
| `r [freq | focus                        | both]`      | Run a sweep over TX frequency, focus, or both (defaults to both). |
| `F min   | max                          | step <MHz>` | Configure frequency sweep bounds.                                 |
| `Z min   | max                          | step <cm>`  | Configure focus sweep bounds.                                     |
| `q`      | Quit the controller.         |

All captured images are written to `clarius_acquisitions/img_XXXXXX.png`. Metadata is appended to `clarius_acquisitions/metadata.json`, including robot pose and wrench when available.

---

### Optional Textual UI

The Textual-based TUI mirrors the CLI functionality with progress bars and live logs.

```bash
python scripts/tui.py
```

The UI embeds an `InteractiveAcquisitionController`, so the same ROS infrastructure must be running beforehand.

---

### Troubleshooting

- **Missing `libffi.so.7`:** Install `libffi7` from your distro and confirm the path passed to `--ld-preload` exists.
- **`libcast.so` not found:** Copy vendor binaries into `scripts/` (the driver loads them relative to its current directory).
- **No images saved:** Ensure `/clarius/bmode` publishes frames (check with `rqt_image_view`) and verify the probe is unfrozen.
- **Franka pose missing in metadata:** Confirm `/franka_state_controller/franka_states` is available and that `franka_control` is running.

---