#!/usr/bin/env python3

import os
import json
import threading
from datetime import datetime

import rospy
from std_msgs.msg import Empty, Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from franka_msgs.msg import FrankaState
import numpy as np
import tf.transformations as tft


from cv_bridge import CvBridge
import cv2

# You might need to run following export before running this script:
# export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libffi.so.7


HELP_TEXT = """
Interactive B-Mode Acquisition Client

Commands:
  f                        : toggle freeze
  r [freq|focus|both]      : perform a sweep and save images + poses
                             default: both

  # Frequency Controls (MHz)
  F min <MHz>              : set minimum TX frequency   (e.g. "F min 5")
  F max <MHz>              : set maximum TX frequency   (e.g. "F max 10")
  F step <MHz>             : set sweep frequency step   (e.g. "F step 1")

  # Focus Controls (cm)
  Z min <cm>               : set minimum TX focus       (e.g. "Z min 2")
  Z max <cm>               : set maximum TX focus       (e.g. "Z max 6")
  Z step <cm>              : set focus step size        (e.g. "Z step 1")

  h                        : show this help
  q                        : quit

Sweep modes:
  r freq   : sweep TX frequency only (focus fixed at Z min)
  r focus  : sweep TX focus only     (frequency fixed at F min)
  r both   : nested sweep over both  (frequency outer, focus inner)
"""


class InteractiveAcquisitionController:
    def __init__(self):
        rospy.init_node("acquisition_controller_node")

        # Publishers to control Clarius
        self.pub_freeze = rospy.Publisher(
            "/clarius/toggle_freeze", Empty, queue_size=1)
        self.pub_request = rospy.Publisher(
            "/clarius/request_image", Empty, queue_size=1)
        self.pub_tx_freq = rospy.Publisher(
            "/clarius/tx_freq", Float32, queue_size=1)
        self.pub_tx_focus = rospy.Publisher(
            "/clarius/tx_focus", Float32, queue_size=1)

        # Subscribe to B-mode images
        self.image_lock = threading.Condition()
        self.last_image = None  # type: Image | None
        rospy.Subscriber("/clarius/bmode", Image, self.on_image)

        # ack handling
        self.freq_ack_lock = threading.Condition()
        self.last_tx_freq_ack = None
        self.focus_ack_lock = threading.Condition()
        self.last_tx_focus_ack = None

        rospy.Subscriber("/clarius/tx_freq_ack", Float32, self.on_tx_freq_ack)
        rospy.Subscriber("/clarius/tx_focus_ack",
                         Float32, self.on_tx_focus_ack)

        # CV bridge for image saving
        self.bridge = CvBridge()

        # --- Franka EE pose (from franka_state_controller) ---
        self.pose_lock = threading.Lock()
        self.last_ee_pose = None  # type: Pose | None

        rospy.Subscriber("/franka_state_controller/franka_states",
                         FrankaState, self.on_franka_state, queue_size=1, tcp_nodelay=True)

        # Sweep configuration (defaults)
        self.freq_min = 3.0
        self.freq_max = 7.0
        self.freq_step = 1.0  # if 0.0 -> single value at freq_min

        self.focus_min = 2.0
        self.focus_max = 5.0
        self.focus_step = 1.0  # if 0.0 -> single value at focus_min

        # Time to wait after each parameter change (seconds) to let image settle
        self.settle_time = 0.5  # ~3-5 frames at ~20-30 fps

        # Optional extra delay between sweep steps to avoid hammering the driver
        self.extra_step_delay = rospy.get_param("~extra_step_delay", 0.0)

        # Base directory for output
        self.output_dir = rospy.get_param(
            "~output_dir", os.path.expanduser("./clarius_acquisitions"))
        os.makedirs(self.output_dir, exist_ok=True)
        self.meta_path = os.path.join(self.output_dir, "metadata.json")

        self.metadata = self._load_metadata()
        self.sweep_counter, self.global_frame_index = self._init_counters_from_metadata()

        rospy.loginfo("Interactive acquisition controller initialized.")
        rospy.loginfo(f"Output directory: {self.output_dir}")
        rospy.loginfo(f"Metadata file   : {self.meta_path}")
        print(HELP_TEXT)

        rospy.sleep(1.0)

    # ------------------------------------------------------------------
    # Metadata helpers
    # ------------------------------------------------------------------

    def _load_metadata(self):
        """Load existing metadata.json if present, else initialize a new one."""
        if os.path.exists(self.meta_path):
            try:
                with open(self.meta_path, "r") as f:
                    data = json.load(f)
                # Basic sanity
                if "sweeps" in data and isinstance(data["sweeps"], list):
                    rospy.loginfo("Loaded existing metadata.json")
                    return data
            except Exception as e:
                rospy.logwarn(
                    f"Failed to load metadata.json, starting fresh: {e}")

        rospy.loginfo("Initializing new metadata structure.")
        return {
            "created_at": datetime.now().isoformat(),
            "sweeps": []  # list of sweep dicts
        }

    def _init_counters_from_metadata(self):
        """Initialize sweep and frame counters based on existing metadata."""
        sweeps = self.metadata.get("sweeps", [])
        sweep_counter = len(sweeps)

        max_frame_index = -1
        for sw in sweeps:
            for fr in sw.get("frames", []):
                idx = fr.get("index", -1)
                if isinstance(idx, int) and idx > max_frame_index:
                    max_frame_index = idx
        global_frame_index = max_frame_index + 1
        return sweep_counter, global_frame_index

    def _save_metadata(self):
        """Write current metadata to disk."""
        try:
            with open(self.meta_path, "w") as f:
                json.dump(self.metadata, f, indent=2)
        except Exception as e:
            rospy.logerr(f"Failed to write metadata.json: {e}")

    # ------------------------------------------------------------------
    # ROS callbacks / utility
    # ------------------------------------------------------------------

    def on_tx_freq_ack(self, msg: Float32):
        with self.freq_ack_lock:
            self.last_tx_freq_ack = float(msg.data)
            self.freq_ack_lock.notify_all()

    def on_tx_focus_ack(self, msg: Float32):
        with self.focus_ack_lock:
            self.last_tx_focus_ack = float(msg.data)
            self.focus_ack_lock.notify_all()

    def set_tx_freq_blocking(self, value: float) -> bool:
        """
        Publish txFreq and wait indefinitely until RosClariusService acks that value.
        Returns False only if ROS is shutting down before the ack arrives.
        """
        with self.freq_ack_lock:
            # Clear any previous ack
            self.last_tx_freq_ack = None
            # Send new request
            self.pub_tx_freq.publish(Float32(value))

            # Wait until we see the desired value, or ROS is shutting down
            while not rospy.is_shutdown():
                # Did we get an ack?
                if self.last_tx_freq_ack is not None:
                    if abs(self.last_tx_freq_ack - value) < 1e-3:
                        return True
                # Wait with no timeout – will wake on notify_all()
                self.freq_ack_lock.wait()

        # Node is shutting down
        return False

    def set_tx_focus_blocking(self, value: float) -> bool:
        """
        Publish txFocus and wait indefinitely until RosClariusService acks that value.
        Returns False only if ROS is shutting down before the ack arrives.
        """
        with self.focus_ack_lock:
            self.last_tx_focus_ack = None
            self.pub_tx_focus.publish(Float32(value))

            while not rospy.is_shutdown():
                if self.last_tx_focus_ack is not None:
                    if abs(self.last_tx_focus_ack - value) < 1e-3:
                        return True
                self.focus_ack_lock.wait()

        return False

    def on_image(self, msg: Image):
        """Store the most recent image and notify any waiting threads."""
        with self.image_lock:
            self.last_image = msg
            self.image_lock.notify_all()

    def wait_for_image(self, timeout: float | None = None) -> Image | None:
        """
        Wait until a new image arrives or ROS is shutting down.

        If timeout is None, block indefinitely (until an image or shutdown).
        If timeout is a float, wait at most that many seconds.
        """
        with self.image_lock:
            # If an image is already here (callback ran just after request),
            # just return it
            if self.last_image is not None:
                return self.last_image

            if timeout is None:
                # Infinite wait (until image or shutdown)
                while not rospy.is_shutdown() and self.last_image is None:
                    self.image_lock.wait()  # no timeout
            else:
                end_time = rospy.Time.now() + rospy.Duration(timeout)
                while (not rospy.is_shutdown() and self.last_image is None):
                    remaining = (end_time - rospy.Time.now()).to_sec()
                    if remaining <= 0.0:
                        break
                    self.image_lock.wait(timeout=remaining)

            # Either we got an image or we're shutting down / timed out
            if self.last_image is None and timeout is not None:
                rospy.logwarn(
                    "Timed out waiting for image from /clarius/bmode.")
            return self.last_image
    
    def on_franka_state(self, msg: FrankaState):
        # O_T_EE is a 4x4 transform stored column major (length 16)
        T = np.array(msg.O_T_EE, dtype=np.float64).reshape((4, 4), order='F')

        p = Pose()
        p.position.x = float(T[0,3])
        p.position.y = float(T[1,3])
        p.position.z = float(T[2,3])

        # Build quaternion from rotation
        T44 = np.eye(4)
        T44[0:3, 0:3] = T[0:3, 0:3]
        qx, qy, qz, qw = tft.quaternion_from_matrix(T44)
        p.orientation.x = float(qx)
        p.orientation.y = float(qy)
        p.orientation.z = float(qz)
        p.orientation.w = float(qw)

        with self.pose_lock:
            self.last_ee_pose = p

    def get_latest_pose(self):
        with self.pose_lock:
            return self.last_ee_pose

    # ------------------------------------------------------------------
    # Sweep logic
    # ------------------------------------------------------------------

    def perform_sweep(self, mode: str):
        """
        Perform a sweep over frequency and/or focus and save images + metadata.

        All images go into self.output_dir.
        Each sweep is appended to metadata["sweeps"] as one entry with a list of frames
        """
        # Determine sweep configuration explicitly from mode
        mode = mode.lower()
        if mode == "freq":
            do_freq = True
            do_focus = False
        elif mode == "focus":
            do_freq = False
            do_focus = True
        elif mode == "both":
            do_freq = True
            do_focus = True
        else:
            rospy.logerr(f"Invalid sweep mode: {mode}")
            return

        sweep_id = self.sweep_counter
        self.sweep_counter += 1

        rospy.loginfo(f"Starting sweep #{sweep_id} (mode={mode}).")

        sweep_meta = {
            "sweep_id": sweep_id,
            "mode": mode,
            "start_time": datetime.now().isoformat(),
            "parameters": {
                "txFreq": {
                    "min": self.freq_min,
                    "max": self.freq_max,
                    "step": self.freq_step,
                },
                "txFocus": {
                    "min": self.focus_min,
                    "max": self.focus_max,
                    "step": self.focus_step,
                },
            },
            "frames": [],
        }

        # Helper to create a float range (if step <= 0, use a single value at min)
        def frange(v_min, v_max, v_step):
            if v_step <= 0.0:
                return [v_min]
            # +1e-6 to prevent accidental truncation due to floating-point drift
            n_steps = int((v_max - v_min) / v_step + 1e-6)
            # +1 to include the endpoint
            return [round(v_min + i * v_step, 6) for i in range(n_steps + 1)]

        # If we are not sweeping a parameter, we will still use its min value
        freq_values = frange(self.freq_min, self.freq_max,
                             self.freq_step) if do_freq else [self.freq_min]
        focus_values = frange(self.focus_min, self.focus_max,
                              self.focus_step) if do_focus else [self.focus_min]

        # Outer loop over frequency, inner over focus
        for f_val in freq_values:
            rospy.sleep(3)
            rospy.loginfo(f"Setting txFreq = {f_val:.3f} MHz")
            self.pub_tx_freq.publish(Float32(f_val))


            for z_val in focus_values:
                rospy.sleep(3)
                rospy.loginfo(f"Setting txFocus = {z_val:.3f} cm")
                self.pub_tx_focus.publish(Float32(z_val))

                # Let image settle
                rospy.sleep(self.settle_time)

                # Clear any previous response and request a fresh image
                with self.image_lock:
                    self.last_image = None

                # Request a fresh image from Clarius
                rospy.loginfo("Requesting image from Clarius...")
                self.pub_request.publish(Empty())
                img_msg = self.wait_for_image(timeout=None)
                if img_msg is None:
                    # rospy.logwarn("Skipping this step due to missing image.")
                    rospy.logwarn(
                        "No image received (node is shutting down?).")
                    continue

                # Ger current robot pose
                current_pose = self.get_latest_pose()
                rospy.loginfo("Retrieved latest pose.")

                # Convert and save image
                try:
                    cv_img = self.bridge.imgmsg_to_cv2(
                        img_msg, desired_encoding="passthrough")
                except Exception as e:
                    rospy.logerr(
                        f"Failed to convert ROS Image to cv2 image: {e}")
                    continue

                # Unique image filename (global index across all sweeps)
                img_filename = f"img_{self.global_frame_index:06d}.png"
                img_path = os.path.join(self.output_dir, img_filename)

                ok = cv2.imwrite(img_path, cv_img)
                if not ok:
                    rospy.logerr(f"Failed to write image to {img_path}")
                    continue


                # Frame metadata
                frame_meta = {
                    "index": int(self.global_frame_index),
                    "image_file": img_filename,
                    "timestamp": img_msg.header.stamp.to_sec(),
                    "txFreq": f_val,
                    "txFocus": z_val,
                }

                if current_pose is not None:
                    rospy.loginfo("Current pose exists and is added to metadata.")
                    frame_meta["pose"] = {
                        "position": {
                            "x": current_pose.position.x,
                            "y": current_pose.position.y,
                            "z": current_pose.position.z,
                        },
                        "orientation": {
                            "x": current_pose.orientation.x,
                            "y": current_pose.orientation.y,
                            "z": current_pose.orientation.z,
                            "w": current_pose.orientation.w,
                        }
                    }

                sweep_meta["frames"].append(frame_meta)

                rospy.loginfo(
                    f"Sweep #{sweep_id}: saved image {img_filename} "
                    f"(txFreq={f_val:.3f} MHz, txFocus={z_val:.3f} cm)."
                )
                self.global_frame_index += 1

                if self.extra_step_delay > 0.0:
                    rospy.sleep(self.extra_step_delay)

        sweep_meta["end_time"] = datetime.now().isoformat()
        self.metadata["sweeps"].append(sweep_meta)
        self._save_metadata()

        rospy.loginfo(
            f"Finished sweep #{sweep_id}. Metadata updated at {self.meta_path}")

    # ------------------------------------------------------------------
    # Command parsing
    # ------------------------------------------------------------------

    def run(self):
        while not rospy.is_shutdown():
            try:
                line = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\nExiting.")
                return

            if not line:
                continue

            cmd, *rest = line.split()

            if cmd == "q":
                print("Quitting.")
                break

            elif cmd == "h":
                print(HELP_TEXT)

            elif cmd == "f":
                rospy.loginfo("Toggling freeze.")
                self.pub_freeze.publish(Empty())

            elif cmd == "r":
                # Optional arg: sweep type
                sweep_mode = "both"
                if rest:
                    if rest[0].lower() in ("freq", "focus", "both"):
                        sweep_mode = rest[0].lower()
                    else:
                        print("Usage: r [freq|focus|both]")
                        continue

                rospy.loginfo(
                    f"Starting sweep acquisition (mode={sweep_mode}).")
                self.perform_sweep(sweep_mode)

            elif cmd == "F":
                # Frequency parameters
                if len(rest) != 2:
                    print('Usage: F min|max|step <value>')
                    print(
                        f"Frequency config: min={self.freq_min}, max={self.freq_max}, step={self.freq_step}")
                    continue
                which, val_str = rest
                try:
                    val = float(val_str)
                except ValueError:
                    print("Invalid frequency value.")
                    continue

                if which == "min":
                    self.freq_min = val
                elif which == "max":
                    self.freq_max = val
                elif which == "step":
                    self.freq_step = val
                else:
                    print('Unknown F subcommand. Use "min", "max", or "step".')
                    continue
                print(
                    f"Frequency config: min={self.freq_min}, max={self.freq_max}, step={self.freq_step}")

            elif cmd == "Z":
                # Focus parameters
                if len(rest) != 2:
                    print('Usage: Z min|max|step <value>')
                    print(
                        f"Focus config: min={self.focus_min}, max={self.focus_max}, step={self.focus_step}")
                    continue
                which, val_str = rest
                try:
                    val = float(val_str)
                except ValueError:
                    print("Invalid focus value.")
                    continue

                if which == "min":
                    self.focus_min = val
                elif which == "max":
                    self.focus_max = val
                elif which == "step":
                    self.focus_step = val
                else:
                    print('Unknown Z subcommand. Use "min", "max", or "step".')
                    continue
                print(
                    f"Focus config: min={self.focus_min}, max={self.focus_max}, step={self.focus_step}")

            else:
                print("Unknown command. Type 'h' for help.")


def main():
    controller = InteractiveAcquisitionController()
    controller.run()


if __name__ == "__main__":
    main()
