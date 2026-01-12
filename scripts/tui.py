#!/usr/bin/env python3
"""
Textual TUI for InteractiveAcquisitionController.

- Left: controls (sweep mode, parameter inputs, actions)
- Right: logs + status
- Progress bar for sweep steps
- Clean shutdown via rospy.signal_shutdown("UI quit")

IMPORTANT:
- Adjust the import below to wherever your controller class lives.
- You said you'll set disable_signal=True on your own; this script does not monkeypatch anything.
"""

from __future__ import annotations

import io
import os
import threading
import time
from contextlib import redirect_stdout, redirect_stderr
from dataclasses import dataclass
from datetime import datetime
from typing import Callable, Optional

import rospy
from std_msgs.msg import Empty, Float32

import cv2

from textual.app import App, ComposeResult
from textual.containers import Horizontal, Vertical, Container
from textual.css.query import NoMatches
from textual.message import Message
from textual.reactive import reactive
from textual.widgets import (
    Button,
    Footer,
    Header,
    Input,
    Label,
    ProgressBar,
    RichLog,
    Select,
    Static,
)

# -------------------------
# ADJUST THIS IMPORT
# -------------------------
# Example: from acquisition_controller import InteractiveAcquisitionController
from acquisition_controller import InteractiveAcquisitionController  # noqa: F401


# -------------------------
# Worker messages
# -------------------------
@dataclass(frozen=True)
class SweepStatus:
    running: bool
    mode: str
    step: int
    total_steps: int
    tx_freq: float
    tx_focus: float
    last_image: Optional[str]
    note: str = ""


class LogLine(Message):
    def __init__(self, text: str) -> None:
        super().__init__()
        self.text = text


class SweepUpdate(Message):
    def __init__(self, status: SweepStatus) -> None:
        super().__init__()
        self.status = status


class SweepDone(Message):
    def __init__(self, ok: bool, reason: str = "") -> None:
        super().__init__()
        self.ok = ok
        self.reason = reason


# -------------------------
# Sweep runner (no monkeypatching)
# -------------------------
def _frange(v_min: float, v_max: float, v_step: float) -> list[float]:
    if v_step <= 0.0:
        return [float(v_min)]
    n_steps = int((v_max - v_min) / v_step + 1e-6)
    return [round(v_min + i * v_step, 6) for i in range(n_steps + 1)]


def run_sweep_with_progress(
    controller: InteractiveAcquisitionController,
    mode: str,
    stop_event: threading.Event,
    post_log: Callable[[str], None],
    post_status: Callable[[SweepStatus], None],
) -> tuple[bool, str]:
    """
    Re-implements the sweep loop using controller's public methods/fields
    so we can drive a progress bar cleanly.
    """
    mode_l = mode.lower().strip()
    if mode_l not in ("freq", "focus", "both"):
        return False, f"Invalid mode '{mode}'. Expected freq|focus|both."

    do_freq = mode_l in ("freq", "both")
    do_focus = mode_l in ("focus", "both")

    # Build sweep values
    freq_values = _frange(controller.freq_min, controller.freq_max, controller.freq_step) if do_freq else [controller.freq_min]
    focus_values = _frange(controller.focus_min, controller.focus_max, controller.focus_step) if do_focus else [controller.focus_min]
    total_steps = len(freq_values) * len(focus_values)

    # Allocate sweep id and metadata container (mirrors controller.perform_sweep format)
    sweep_id = controller.sweep_counter
    controller.sweep_counter += 1

    sweep_meta = {
        "sweep_id": int(sweep_id),
        "mode": mode_l,
        "start_time": datetime.now().isoformat(),
        "parameters": {
            "txFreq": {"min": float(controller.freq_min), "max": float(controller.freq_max), "step": float(controller.freq_step)},
            "txFocus": {"min": float(controller.focus_min), "max": float(controller.focus_max), "step": float(controller.focus_step)},
        },
        "frames": [],
    }

    post_log(f"Starting sweep #{sweep_id} (mode={mode_l}), steps={total_steps}")

    # Main loop
    step_idx = 0
    last_image_file: Optional[str] = None
    current_f = float(freq_values[0])
    current_z = float(focus_values[0])

    for f_val in freq_values:
        if stop_event.is_set() or rospy.is_shutdown():
            break

        current_f = float(f_val)
        post_log(f"Setting txFreq={current_f:.3f} MHz")

        # Prefer blocking set (ack-based) if available; falls back to publish+sleep if ROS is shutting down.
        ok = controller.set_tx_freq_blocking(current_f)
        if not ok and rospy.is_shutdown():
            break

        for z_val in focus_values:
            if stop_event.is_set() or rospy.is_shutdown():
                break

            current_z = float(z_val)
            post_log(f"Setting txFocus={current_z:.3f} cm")

            ok = controller.set_tx_focus_blocking(current_z)
            if not ok and rospy.is_shutdown():
                break

            # Let image settle
            rospy.sleep(controller.settle_time)

            # Clear previous and request new image
            with controller.image_lock:
                controller.last_image = None

            controller.pub_request.publish(Empty())
            img_msg = controller.wait_for_image(timeout=5.0)

            note = ""
            if img_msg is None:
                note = "No image received (timeout)."
                post_log(f"WARNING: {note}")
            else:
                # Get current robot state
                current_pose, current_wrench = controller.get_latest_state()

                # Convert and save image
                try:
                    cv_img = controller.bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
                except Exception as e:
                    post_log(f"ERROR: cv_bridge conversion failed: {e}")
                    cv_img = None

                if cv_img is not None:
                    img_filename = f"img_{controller.global_frame_index:06d}.png"
                    img_path = os.path.join(controller.output_dir, img_filename)

                    ok_write = cv2.imwrite(img_path, cv_img)
                    if not ok_write:
                        post_log(f"ERROR: Failed to write image to {img_path}")
                    else:
                        last_image_file = img_filename

                        frame_meta = {
                            "index": int(controller.global_frame_index),
                            "image_file": img_filename,
                            "timestamp": float(img_msg.header.stamp.to_sec()),
                            "txFreq": float(current_f),
                            "txFocus": float(current_z),
                        }

                        if current_pose is not None:
                            frame_meta["pose"] = {
                                "position": {
                                    "x": float(current_pose.position.x),
                                    "y": float(current_pose.position.y),
                                    "z": float(current_pose.position.z),
                                },
                                "orientation": {
                                    "x": float(current_pose.orientation.x),
                                    "y": float(current_pose.orientation.y),
                                    "z": float(current_pose.orientation.z),
                                    "w": float(current_pose.orientation.w),
                                },
                            }

                        if current_wrench is not None:
                            frame_meta["wrench"] = current_wrench

                        sweep_meta["frames"].append(frame_meta)
                        controller.global_frame_index += 1

            step_idx += 1
            post_status(
                SweepStatus(
                    running=True,
                    mode=mode_l,
                    step=step_idx,
                    total_steps=total_steps,
                    tx_freq=current_f,
                    tx_focus=current_z,
                    last_image=last_image_file,
                    note=note,
                )
            )

            if controller.extra_step_delay and controller.extra_step_delay > 0.0:
                rospy.sleep(controller.extra_step_delay)

    # Finalize sweep metadata (write even if partial)
    sweep_meta["end_time"] = datetime.now().isoformat()
    controller.metadata["sweeps"].append(sweep_meta)
    controller._save_metadata()

    if stop_event.is_set():
        post_log(f"Sweep #{sweep_id} stopped early. Saved {len(sweep_meta['frames'])} frames.")
        return False, "Stopped by user."

    if rospy.is_shutdown():
        post_log(f"Sweep #{sweep_id} ended due to ROS shutdown. Saved {len(sweep_meta['frames'])} frames.")
        return False, "ROS shutdown."

    post_log(f"Finished sweep #{sweep_id}. Saved {len(sweep_meta['frames'])} frames.")
    return True, "OK"


# -------------------------
# UI widgets
# -------------------------
class StatusPanel(Static):
    status_text = reactive("Idle")

    def compose(self) -> ComposeResult:
        yield Label("Status", id="status_title")
        yield Label("", id="status_text")

    def watch_status_text(self, value: str) -> None:
        try:
            self.query_one("#status_text", Label).update(value)
        except NoMatches:
            pass


class AcquisitionTUI(App):
    CSS = """
    Screen {
        layout: vertical;
    }

    #main {
        height: 1fr;
    }

    #left {
        width: 42;
        padding: 1 1;
        border: solid $primary;
    }

    #right {
        width: 1fr;
        padding: 1 1;
        border: solid $primary;
    }

    #controls_title {
        text-style: bold;
        margin-bottom: 1;
    }

    .section {
        margin-bottom: 1;
        padding: 0 0;
    }

    .row {
        layout: horizontal;
        height: auto;
        align: left middle;
        margin: 0 0 1 0;
    }

    .row Label {
        width: 9;
    }

    Input {
        width: 1fr;
    }

    Select {
        width: 1fr;
    }

    #buttons_row Button {
        width: 1fr;
        margin-right: 1;
    }

    #buttons_row Button:last-child {
        margin-right: 0;
    }

    #log {
        height: 1fr;
        border: solid $secondary;
        padding: 0 1;
    }

    #progress_box {
        border: solid $secondary;
        padding: 1 1;
        margin-bottom: 1;
    }

    #progress_label {
        margin-bottom: 1;
    }
    """

    BINDINGS = [
        ("q", "quit_ui", "Quit"),
        ("f", "freeze", "Freeze"),
    ]

    def __init__(self) -> None:
        super().__init__()
        self.controller: Optional[InteractiveAcquisitionController] = None
        self.sweep_thread: Optional[threading.Thread] = None
        self.sweep_stop = threading.Event()

        self._last_status = SweepStatus(
            running=False, mode="both", step=0, total_steps=1, tx_freq=0.0, tx_focus=0.0, last_image=None
        )

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        with Horizontal(id="main"):
            with Vertical(id="left"):
                yield Label("Controls", id="controls_title")

                # Mode + actions
                with Container(classes="section"):
                    yield Label("Sweep", classes="section")
                    with Container(classes="row"):
                        yield Label("Mode")
                        yield Select(
                            options=[("both", "both"), ("freq", "freq"), ("focus", "focus")],
                            value="both",
                            id="mode_select",
                        )
                    with Container(id="buttons_row", classes="row"):
                        yield Button("Start", id="start_btn", variant="success")
                        yield Button("Freeze", id="freeze_btn")
                        yield Button("Stop", id="stop_btn", variant="warning")
                        yield Button("Quit", id="quit_btn", variant="error")

                # Frequency settings
                with Container(classes="section"):
                    yield Label("TX Frequency (MHz)", classes="section")
                    with Container(classes="row"):
                        yield Label("F min")
                        yield Input(placeholder="e.g. 3.0", id="f_min")
                    with Container(classes="row"):
                        yield Label("F max")
                        yield Input(placeholder="e.g. 7.0", id="f_max")
                    with Container(classes="row"):
                        yield Label("F step")
                        yield Input(placeholder="e.g. 1.0", id="f_step")

                # Focus settings
                with Container(classes="section"):
                    yield Label("TX Focus (cm)", classes="section")
                    with Container(classes="row"):
                        yield Label("Z min")
                        yield Input(placeholder="e.g. 2.0", id="z_min")
                    with Container(classes="row"):
                        yield Label("Z max")
                        yield Input(placeholder="e.g. 5.0", id="z_max")
                    with Container(classes="row"):
                        yield Label("Z step")
                        yield Input(placeholder="e.g. 1.0", id="z_step")

                with Container(classes="section"):
                    yield Button("Apply Settings", id="apply_btn")

                # Quick info
                yield StatusPanel(id="status_panel")

            with Vertical(id="right"):
                with Container(id="progress_box"):
                    yield Label("Sweep Progress", id="progress_label")
                    yield ProgressBar(total=1, show_percentage=True, id="progress")
                    yield Label("", id="progress_detail")
                yield RichLog(id="log", wrap=True, highlight=False)

        yield Footer()

    # -------------------------
    # Lifecycle
    # -------------------------
    def on_mount(self) -> None:
        self._log("Initializing controller...")

        # Controller prints help text in its __init__; suppress stdout/stderr to avoid breaking TUI.
        buf_out = io.StringIO()
        buf_err = io.StringIO()
        try:
            with redirect_stdout(buf_out), redirect_stderr(buf_err):
                self.controller = InteractiveAcquisitionController()
        except Exception as e:
            self._log(f"ERROR: Failed to initialize controller: {e}")
            raise

        # Seed inputs with current config
        self._seed_inputs_from_controller()
        self._update_status_panel_idle()
        self._log(f"Output dir: {self.controller.output_dir}")
        self._log("Ready.")

    def _seed_inputs_from_controller(self) -> None:
        c = self.controller
        if c is None:
            return
        self.query_one("#f_min", Input).value = str(c.freq_min)
        self.query_one("#f_max", Input).value = str(c.freq_max)
        self.query_one("#f_step", Input).value = str(c.freq_step)
        self.query_one("#z_min", Input).value = str(c.focus_min)
        self.query_one("#z_max", Input).value = str(c.focus_max)
        self.query_one("#z_step", Input).value = str(c.focus_step)

    # -------------------------
    # UI helpers
    # -------------------------
    def _log(self, text: str) -> None:
        self.post_message(LogLine(text))

    def on_log_line(self, msg: LogLine) -> None:
        log = self.query_one("#log", RichLog)
        ts = time.strftime("%H:%M:%S")
        log.write(f"[{ts}] {msg.text}")

    def _set_progress(self, step: int, total: int, detail: str) -> None:
        pb = self.query_one("#progress", ProgressBar)
        pb.total = max(1, int(total))
        pb.progress = max(0, min(int(step), pb.total))
        self.query_one("#progress_detail", Label).update(detail)

    def _update_status_panel_idle(self) -> None:
        panel = self.query_one("#status_panel", StatusPanel)
        c = self.controller
        if c is None:
            panel.status_text = "Controller not initialized."
            return
        panel.status_text = (
            "Idle\n"
            f"F: min={c.freq_min}, max={c.freq_max}, step={c.freq_step}\n"
            f"Z: min={c.focus_min}, max={c.focus_max}, step={c.focus_step}\n"
            f"settle_time={c.settle_time}s  extra_step_delay={c.extra_step_delay}s\n"
            f"next sweep_id={c.sweep_counter}  next frame_index={c.global_frame_index}"
        )
        self._set_progress(0, 1, "")

    def _update_status_panel_running(self, st: SweepStatus) -> None:
        panel = self.query_one("#status_panel", StatusPanel)
        panel.status_text = (
            "Running\n"
            f"mode={st.mode}\n"
            f"step={st.step}/{st.total_steps}\n"
            f"txFreq={st.tx_freq:.3f} MHz\n"
            f"txFocus={st.tx_focus:.3f} cm\n"
            f"last_image={st.last_image or '-'}\n"
            f"{st.note}".rstrip()
        )
        detail = f"mode={st.mode}  txFreq={st.tx_freq:.3f}MHz  txFocus={st.tx_focus:.3f}cm"
        if st.last_image:
            detail += f"  saved={st.last_image}"
        if st.note:
            detail += f"  ({st.note})"
        self._set_progress(st.step, st.total_steps, detail)

    # -------------------------
    # Button + key actions
    # -------------------------
    def action_quit_ui(self) -> None:
        self._request_shutdown()

    def action_freeze(self) -> None:
        self._toggle_freeze()

    def on_button_pressed(self, event: Button.Pressed) -> None:
        bid = event.button.id
        if bid == "quit_btn":
            self._request_shutdown()
        elif bid == "freeze_btn":
            self._toggle_freeze()
        elif bid == "apply_btn":
            self._apply_settings()
        elif bid == "start_btn":
            self._start_sweep()
        elif bid == "stop_btn":
            self._stop_sweep()

    def _toggle_freeze(self) -> None:
        c = self.controller
        if c is None:
            return
        try:
            c.pub_freeze.publish(Empty())
            self._log("Freeze toggled.")
        except Exception as e:
            self._log(f"ERROR: Freeze publish failed: {e}")

    def _apply_settings(self) -> None:
        c = self.controller
        if c is None:
            return

        def parse_float(widget_id: str) -> float:
            s = self.query_one(widget_id, Input).value.strip()
            return float(s)

        try:
            c.freq_min = parse_float("#f_min")
            c.freq_max = parse_float("#f_max")
            c.freq_step = parse_float("#f_step")
            c.focus_min = parse_float("#z_min")
            c.focus_max = parse_float("#z_max")
            c.focus_step = parse_float("#z_step")
        except Exception as e:
            self._log(f"ERROR: Invalid settings: {e}")
            return

        self._log(
            f"Applied settings: "
            f"F(min={c.freq_min}, max={c.freq_max}, step={c.freq_step}) "
            f"Z(min={c.focus_min}, max={c.focus_max}, step={c.focus_step})"
        )
        self._update_status_panel_idle()

    def _start_sweep(self) -> None:
        c = self.controller
        if c is None:
            return
        if self.sweep_thread and self.sweep_thread.is_alive():
            self._log("Sweep already running.")
            return

        mode = self.query_one("#mode_select", Select).value or "both"
        self.sweep_stop.clear()

        def post_log(text: str) -> None:
            self.post_message(LogLine(text))

        def post_status(st: SweepStatus) -> None:
            self.post_message(SweepUpdate(st))

        def worker() -> None:
            ok, reason = run_sweep_with_progress(
                controller=c,
                mode=str(mode),
                stop_event=self.sweep_stop,
                post_log=post_log,
                post_status=post_status,
            )
            self.post_message(SweepDone(ok=ok, reason=reason))

        self._log(f"Launching sweep thread (mode={mode})")
        self.sweep_thread = threading.Thread(target=worker, name="sweep_worker", daemon=True)
        self.sweep_thread.start()

    def _stop_sweep(self) -> None:
        if self.sweep_thread and self.sweep_thread.is_alive():
            self._log("Stop requested.")
            self.sweep_stop.set()
        else:
            self._log("No sweep running.")

    def _request_shutdown(self) -> None:
        self._log("Shutdown requested.")
        self.sweep_stop.set()

        # Ask ROS to shutdown so any blocking waits (e.g. for image/ack) can exit.
        try:
            rospy.signal_shutdown("UI quit")
        except Exception as e:
            self._log(f"WARNING: rospy.signal_shutdown failed: {e}")

        self.exit()

    # -------------------------
    # Worker message handlers
    # -------------------------
    def on_sweep_update(self, msg: SweepUpdate) -> None:
        self._last_status = msg.status
        self._update_status_panel_running(msg.status)

    def on_sweep_done(self, msg: SweepDone) -> None:
        if msg.ok:
            self._log("Sweep completed.")
        else:
            self._log(f"Sweep ended: {msg.reason}")
        self._update_status_panel_idle()

    # -------------------------
    # Ensure clean shutdown even if app exits unexpectedly
    # -------------------------
    def on_shutdown_request(self) -> None:
        # Textual emits this when the app is asked to shut down (e.g. close terminal).
        self.sweep_stop.set()
        try:
            rospy.signal_shutdown("UI shutdown")
        except Exception:
            pass


def main() -> None:
    AcquisitionTUI().run()


if __name__ == "__main__":
    main()
