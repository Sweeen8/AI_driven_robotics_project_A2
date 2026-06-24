#!/usr/bin/env python3
"""
Guarded Fast Lite6 SDK Controller

Standalone movement controller supporting virtual (simulated) and physical modes.

Required files:
- movement/guarded_fast_sdk_controller.py
- movement/named_positions.yaml

Boot prompt selects virtual (v) or physical (p) mode.
In virtual mode the robot is fully simulated, no hardware required.
In physical mode the real Lite6 is driven via the xArm SDK.

Movement commands are sent with wait=False so the SDK lock is released
immediately after the command is dispatched. A polling loop then tracks
the live joint position until the target is reached. This prevents the
joint-state publisher thread from being starved while the robot moves.
"""

import math
import os
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

try:
    import rclpy
    from sensor_msgs.msg import JointState
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

import yaml

try:
    from xarm.wrapper import XArmAPI
    XARM_SDK_AVAILABLE = True
except ImportError:
    XARM_SDK_AVAILABLE = False


# ==========================================================
# Configuration
# ==========================================================

ROBOT_IP = "192.168.1.157"
PROJECT_ROOT = Path.home() / "Git-projects" / "AI_driven_robotics_project_A2"
PACKAGE_NAME = "a2_workcell_description"
RVIZ_LAUNCH_FILE = "lite6_workcell_bringup.launch.py"
COLLISION_EXECUTABLE = "add_workcell_collision_objects"

PUBLISH_JOINT_STATES_TO_RVIZ = True
JOINT_STATE_RATE_HZ = 10.0
MAX_NAMED_STATE_ERROR_DEG = 3.0

DEFAULT_SPEED = 0.15
DEFAULT_ACCELERATION = 0.8
FAST_MODE = False

if FAST_MODE:
    DEFAULT_SPEED = 0.35
    DEFAULT_ACCELERATION = 2.0

# Polling thresholds for move_to_target
MOVE_DONE_THRESHOLD_DEG = 1.0
MOVE_POLL_INTERVAL = 0.25
MOVE_TIMEOUT = 30.0

# Gripper position range: 0 (closed) … 850 (fully open), speed in units/s.
# finger_joint1 URDF limit: 0 … 0.0089 m  →  scale = 0.0089 / 850
_GRIPPER_OPEN_POS  = 850
_GRIPPER_CLOSE_POS = 0
_GRIPPER_SPEED     = 5000
_GRIPPER_FINGER_SCALE = 0.0089 / 850.0

# Software-tracked gripper position shared between gripper commands and the
# joint-state publisher.  Written by open/close/turn-on functions; read by
# RealRobotJointStatePublisher.  A list so it is mutable across threads.
_gripper_raw_pos: List[float] = [float(_GRIPPER_OPEN_POS)]


# ==========================================================
# Allowed movement sequences
# ==========================================================

SEQUENCES = {
    "home_to_pick": {
        "required_start": "home",
        "targets": ["home_pick_turn", "pick"],
        "label": "home -> home_pick_turn -> pick",
        "gripper_before": None,
        "gripper_after": None,
    },
    "home_to_sort": {
        "required_start": "home",
        "targets": ["home_sort_turn", "sort"],
        "label": "home -> home_sort_turn -> sort",
        "gripper_before": None,
        "gripper_after": None,
    },
    "pick_to_sort": {
        "required_start": "pick",
        "targets": ["pick_lift", "sort_lift", "sort"],
        "label": "pick -> pick_lift -> sort_lift -> sort",
        "gripper_before": "close",   # grab object at pick before lifting
        "gripper_after":  "open",    # release at sort after arriving
    },
    "sort_to_pick": {
        "required_start": "sort",
        "targets": ["sort_lift", "pick_lift", "pick"],
        "label": "sort -> sort_lift -> pick_lift -> pick",
        "gripper_before": None,
        "gripper_after": None,
    },
    "pick_to_home": {
        "required_start": "pick",
        "targets": ["home_pick_turn", "home"],
        "label": "pick -> home_pick_turn -> home",
        "gripper_before": None,
        "gripper_after": None,
    },
    "sort_to_home": {
        "required_start": "sort",
        "targets": ["home_sort_turn", "home"],
        "label": "sort -> home_sort_turn -> home",
        "gripper_before": None,
        "gripper_after": None,
    },
    "unknown_to_home": {
        "required_start": "unknown",
        "targets": ["home"],
        "label": "unknown -> home",
        "gripper_before": None,
        "gripper_after": None,
    },
}


# ==========================================================
# Virtual arm stub
# ==========================================================

class VirtualArmAPI:
    """
    Simulates the Lite6 without hardware.

    All joints start at home (zero). Movement commands interpolate
    the joint positions over time so the live position display and
    the joint-state publisher see smooth motion.
    """

    def __init__(self) -> None:
        self._joints: List[float] = [0.0] * 6
        self._internal_lock = threading.Lock()
        self._move_thread: Optional[threading.Thread] = None
        self._gripper_open: Optional[bool] = None
        self._gripper_pos: float = float(_GRIPPER_OPEN_POS)

    def clean_warn(self) -> int:
        return 0

    def clean_error(self) -> int:
        return 0

    def motion_enable(self, enable: bool = True) -> int:
        return 0

    def set_mode(self, mode: int) -> int:
        return 0

    def set_state(self, state: int) -> int:
        return 0

    def disconnect(self) -> None:
        pass

    # -- Lite6 built-in gripper stubs ----------------------------------

    def open_lite6_gripper(self, sync: bool = True) -> int:
        with self._internal_lock:
            self._gripper_pos = float(_GRIPPER_OPEN_POS)
            self._gripper_open = True
        return 0

    def close_lite6_gripper(self, sync: bool = True) -> int:
        with self._internal_lock:
            self._gripper_pos = float(_GRIPPER_CLOSE_POS)
            self._gripper_open = False
        return 0

    def stop_lite6_gripper(self, sync: bool = True) -> int:
        return 0

    def get_gripper_position(self) -> Tuple[int, float]:
        with self._internal_lock:
            return 0, self._gripper_pos

    def get_servo_angle(self, is_radian: bool = True) -> Tuple[int, List[float]]:
        with self._internal_lock:
            # Return 7 values to match the real SDK's layout.
            return 0, list(self._joints) + [0.0]

    def set_servo_angle(
        self,
        angle: List[float],
        speed: float = DEFAULT_SPEED,
        mvacc: float = DEFAULT_ACCELERATION,
        wait: bool = True,
        is_radian: bool = True,
    ) -> int:
        target = [float(v) for v in angle[:6]]

        def _simulate() -> None:
            with self._internal_lock:
                start = list(self._joints)

            max_delta = max(abs(t - s) for t, s in zip(target, start))
            duration = max(max_delta / max(speed, 0.01), 0.3)
            steps = max(int(duration / 0.05), 10)

            for i in range(steps + 1):
                fraction = i / steps
                interpolated = [s + (t - s) * fraction for s, t in zip(start, target)]
                with self._internal_lock:
                    self._joints = interpolated
                if i < steps:
                    time.sleep(duration / steps)

        if wait:
            _simulate()
        else:
            if self._move_thread and self._move_thread.is_alive():
                self._move_thread.join(timeout=0.1)
            self._move_thread = threading.Thread(target=_simulate, daemon=True)
            self._move_thread.start()

        return 0


# ==========================================================
# Math helpers
# ==========================================================

def radians_to_degrees(values: List[float]) -> List[float]:
    return [math.degrees(v) for v in values]


def max_joint_error_degrees(
    current_rad: List[float],
    target_rad: List[float],
) -> float:
    return max(math.degrees(abs(c - t)) for c, t in zip(current_rad, target_rad))


# ==========================================================
# YAML loading
# ==========================================================

def load_named_positions() -> dict:
    config_path = Path(__file__).resolve().parent / "named_positions.yaml"

    if not config_path.exists():
        raise FileNotFoundError(f"Could not find named_positions.yaml at: {config_path}")

    with config_path.open("r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    if not isinstance(config, dict):
        raise ValueError("named_positions.yaml is not a valid YAML dictionary.")

    for key in ("joint_names", "positions"):
        if key not in config:
            raise ValueError(f"named_positions.yaml is missing: {key}")

    for name in (
        "home", "pick", "sort", "safe_crossing",
        "home_pick_turn", "home_sort_turn", "pick_lift", "sort_lift",
    ):
        if name not in config["positions"]:
            raise ValueError(f"named_positions.yaml is missing position: {name}")
        if "joints_rad" not in config["positions"][name]:
            raise ValueError(f"Position '{name}' is missing joints_rad.")

    return config


def get_positions(config: dict) -> Dict[str, List[float]]:
    positions = {}
    for name, data in config["positions"].items():
        joints_rad = data.get("joints_rad")
        if not isinstance(joints_rad, list):
            raise ValueError(f"Position '{name}' has invalid joints_rad.")
        if len(joints_rad) != 6:
            raise ValueError(
                f"Position '{name}' has {len(joints_rad)} joints, expected 6."
            )
        positions[name] = [float(v) for v in joints_rad]
    return positions


# ==========================================================
# Boot mode selection
# ==========================================================

def select_boot_mode() -> str:
    print("")
    print("Select boot mode:")
    print("  v = virtual  (simulated robot, no hardware required)")
    print(f"  p = physical (real Lite6 at {ROBOT_IP})")
    print("")

    while True:
        choice = input("Mode [v/p]: ").strip().lower()
        if choice == "v":
            print("\nVirtual mode selected. Simulation starting.")
            return "virtual"
        if choice == "p":
            print("\nPhysical mode selected. Connecting to real robot.")
            return "physical"
        print("Please enter 'v' or 'p'.")


# ==========================================================
# Robot connection
# ==========================================================

def check_code(code: int, action: str) -> None:
    if code != 0:
        raise RuntimeError(f"{action} failed. xArm code: {code}")


def connect_arm(mode: str):
    if mode == "virtual":
        print("Creating virtual arm (no hardware connection).")
        return VirtualArmAPI()

    if not XARM_SDK_AVAILABLE:
        raise RuntimeError(
            "xarm SDK not found. Run in virtual mode or install the SDK."
        )

    print(f"Connecting to Lite6 at {ROBOT_IP}...")
    arm = XArmAPI(ROBOT_IP, is_radian=True)

    # Clear error first, then warnings. Return codes of 1 here are normal
    # (nothing to clear, or controller settling) — do not raise on them.
    arm.clean_error()
    arm.clean_warn()
    time.sleep(0.2)

    check_code(arm.motion_enable(enable=True), "motion_enable")
    check_code(arm.set_mode(0), "set_mode(0)")
    check_code(arm.set_state(0), "set_state(0)")
    time.sleep(0.5)
    print("Connected to Lite6.")
    return arm


def read_current_joints(arm, sdk_lock: threading.Lock) -> List[float]:
    with sdk_lock:
        code, angles = arm.get_servo_angle(is_radian=True)
    check_code(code, "get_servo_angle")
    if angles is None:
        raise RuntimeError("Robot returned no joint angles.")
    if len(angles) < 6:
        raise RuntimeError(
            f"Robot returned only {len(angles)} joint values. Expected 6."
        )
    return [float(v) for v in angles[:6]]


# ==========================================================
# State detection
# ==========================================================

def find_nearest_named_state(
    current_joints: List[float],
    positions: Dict[str, List[float]],
) -> Tuple[str, float]:
    nearest_name = "unknown"
    nearest_error = 999999.0

    for name, target_joints in positions.items():
        error = max_joint_error_degrees(current_joints, target_joints)
        if error < nearest_error:
            nearest_name = name
            nearest_error = error

    if nearest_error > MAX_NAMED_STATE_ERROR_DEG:
        return "unknown", nearest_error

    return nearest_name, nearest_error


def print_current_position(
    current_joints: List[float],
    nearest_state: str,
    nearest_error: float,
) -> None:
    current_deg = radians_to_degrees(current_joints)

    print("")
    print("Current joint positions:")
    print("  degrees:")
    for i, v in enumerate(current_deg, start=1):
        print(f"    J{i}: {v:.3f}")
    print("  radians:")
    for v in current_joints:
        print(f"    - {v:.6f}")
    print("")
    print(f"  Nearest named state : {nearest_state}")
    print(f"  Max error from state: {nearest_error:.2f} deg")


# ==========================================================
# Movement
# ==========================================================

# ==========================================================
# Lite6 gripper control
# ==========================================================


def turn_on_gripper(
    arm,
    sdk_lock: threading.Lock,
    mode: str,
) -> bool:
    """
    Enable the Lite6 gripper.
    Opening once acts as the enable/recover step — matches UFactory Studio behaviour.
    Returns True when the gripper is ready.
    """
    if mode == "virtual":
        arm.open_lite6_gripper()
        print("Gripper: virtual mode — turned on (open).")
        return True

    print("")
    print("Turning on Lite6 gripper...")

    with sdk_lock:
        code = arm.open_lite6_gripper(sync=True)

    if code != 0:
        print(f"  ERROR: Failed to turn on gripper. xArm code: {code}")
        if code in (19, 0x13):
            print("  C19 — End module communication error.")
            print("  Hardware checks:")
            print("    1. Reseat the gripper cable at the robot wrist.")
            print("    2. Power-cycle the robot with the gripper connected.")
            print("    3. UFactory Studio → Settings → End Effector → Baud rate = 2000000.")
        with sdk_lock:
            arm.clean_error()
            arm.clean_warn()
        time.sleep(0.2)
        with sdk_lock:
            arm.motion_enable(enable=True)
            arm.set_mode(0)
            arm.set_state(0)
        return False

    # Re-assert arm ready state so movement commands work immediately after.
    time.sleep(0.1)
    with sdk_lock:
        arm.set_state(0)

    _gripper_raw_pos[0] = float(_GRIPPER_OPEN_POS)
    print("  Lite6 gripper is ON and open.")
    return True


def open_gripper(
    arm,
    sdk_lock: threading.Lock,
    mode: str,
    gripper_enabled: bool,
) -> Optional[bool]:
    """Open the Lite6 gripper. Returns True on success, None on error."""
    if not gripper_enabled:
        print("")
        print("Gripper command rejected — gripper is not turned on.")
        print("Press 't' first to turn on / enable the gripper.")
        return None

    if mode == "virtual":
        arm.open_lite6_gripper()
        _gripper_raw_pos[0] = float(_GRIPPER_OPEN_POS)
        print("  [VIRTUAL] Gripper opened.")
        return True

    print("  Opening gripper...")
    with sdk_lock:
        code = arm.open_lite6_gripper(sync=True)

    if code != 0:
        print(f"  WARNING: Failed to open gripper. xArm code: {code}")
        return None

    time.sleep(0.1)
    with sdk_lock:
        arm.set_state(0)

    _gripper_raw_pos[0] = float(_GRIPPER_OPEN_POS)
    print("  Gripper opened.")
    return True


def close_gripper(
    arm,
    sdk_lock: threading.Lock,
    mode: str,
    gripper_enabled: bool,
) -> Optional[bool]:
    """Close the Lite6 gripper. Returns False on success, None on error."""
    if not gripper_enabled:
        print("")
        print("Gripper command rejected — gripper is not turned on.")
        print("Press 't' first to turn on / enable the gripper.")
        return None

    if mode == "virtual":
        arm.close_lite6_gripper()
        _gripper_raw_pos[0] = float(_GRIPPER_CLOSE_POS)
        print("  [VIRTUAL] Gripper closed.")
        return False

    print("  Closing gripper...")
    with sdk_lock:
        code = arm.close_lite6_gripper(sync=True)

    if code != 0:
        print(f"  WARNING: Failed to close gripper. xArm code: {code}")
        return None

    time.sleep(0.1)
    with sdk_lock:
        arm.set_state(0)

    _gripper_raw_pos[0] = float(_GRIPPER_CLOSE_POS)
    print("  Gripper closed.")
    return False


def stop_gripper(
    arm,
    sdk_lock: threading.Lock,
    mode: str,
) -> bool:
    """
    Stop the Lite6 gripper.
    After stopping, press 't' before using open/close again.
    """
    if mode == "virtual":
        arm.stop_lite6_gripper()
        print("  [VIRTUAL] Gripper stopped.")
        return True

    print("  Stopping gripper...")
    with sdk_lock:
        code = arm.stop_lite6_gripper(sync=True)

    if code != 0:
        print(f"  WARNING: Failed to stop gripper. xArm code: {code}")
        return False

    time.sleep(0.1)
    with sdk_lock:
        arm.set_state(0)

    print("  Gripper stopped. Press 't' before using open/close again.")
    return True


def _try_clear_robot_error(arm, sdk_lock: threading.Lock) -> None:
    with sdk_lock:
        arm.clean_error()
        arm.clean_warn()
    time.sleep(0.2)
    with sdk_lock:
        arm.motion_enable(enable=True)
        arm.set_mode(0)
        arm.set_state(0)


def move_to_target(
    arm,
    sdk_lock: threading.Lock,
    target_name: str,
    positions: Dict[str, List[float]],
) -> None:
    if target_name not in positions:
        raise ValueError(f"Unknown target position: {target_name}")

    target_joints = positions[target_name]
    print(f"Moving to {target_name}...")

    # Clear any active C19 (gripper comm) or stale errors, then send the
    # movement command atomically inside the same lock so the background
    # publisher cannot re-trigger C19 between the clear and the send.
    with sdk_lock:
        arm.clean_error()
        arm.clean_warn()
        arm.set_state(0)
        code = arm.set_servo_angle(
            angle=target_joints,
            speed=DEFAULT_SPEED,
            mvacc=DEFAULT_ACCELERATION,
            wait=False,
            is_radian=True,
        )

    # If the command was still rejected (e.g. arm needed full re-enable),
    # do the heavy reset and retry once more.
    if code != 0:
        print(f"  set_servo_angle returned code {code}. Re-enabling robot and retrying...")
        _try_clear_robot_error(arm, sdk_lock)
        time.sleep(0.5)
        with sdk_lock:
            code = arm.set_servo_angle(
                angle=target_joints,
                speed=DEFAULT_SPEED,
                mvacc=DEFAULT_ACCELERATION,
                wait=False,
                is_radian=True,
            )

    check_code(code, f"move_to_target({target_name})")

    # Poll until the robot reaches the target, printing live joint values.
    start_time = time.monotonic()
    last_joints: List[float] = target_joints
    last_best_error = float("inf")
    last_progress_time = start_time
    failed = False

    while True:
        elapsed = time.monotonic() - start_time

        if elapsed > MOVE_TIMEOUT:
            print(f"\nWARNING: Move to {target_name} timed out after {MOVE_TIMEOUT:.0f}s.")
            failed = True
            break

        with sdk_lock:
            read_code, angles = arm.get_servo_angle(is_radian=True)

        # C19 = gripper end-module comm error: does not affect arm joints.
        # Clear it silently so the move continues uninterrupted.
        if read_code == 19:
            with sdk_lock:
                arm.clean_error()
                arm.clean_warn()
        elif read_code != 0:
            print(f"\nControllerError during move (SDK code {read_code}). Stopping.")
            failed = True
            break

        if angles is not None and len(angles) >= 6:
            current = [float(v) for v in angles[:6]]
            last_joints = current
            error = max_joint_error_degrees(current, target_joints)

            deg_str = "  ".join(
                f"J{i + 1}:{math.degrees(v):+7.2f}" for i, v in enumerate(current)
            )
            print(f"\r  [{deg_str}]  err:{error:5.2f}deg", end="", flush=True)

            if error < MOVE_DONE_THRESHOLD_DEG:
                break

            # Track whether the robot is making progress.
            if error < last_best_error - 0.5:
                last_best_error = error
                last_progress_time = time.monotonic()
            elif time.monotonic() - last_progress_time > 4.0:
                print(f"\nRobot stopped making progress toward {target_name}. "
                      "Possible controller error.")
                failed = True
                break

        time.sleep(MOVE_POLL_INTERVAL)

    # Overwrite the live line with a permanent final-position line.
    final_deg = radians_to_degrees(last_joints)
    deg_str = "  ".join(f"J{i + 1}:{v:+7.2f}" for i, v in enumerate(final_deg))

    if failed:
        print(f"\r  [{deg_str}]  <- STOPPED at {target_name} (error)")
        print("Attempting to clear robot error and re-enable...")
        _try_clear_robot_error(arm, sdk_lock)
        raise RuntimeError(
            f"Move to '{target_name}' failed. Robot error cleared — "
            "check position before retrying."
        )

    print(f"\r  [{deg_str}]  <- {target_name}")
    print(f"Reached: {target_name}")


def _run_gripper_action(
    action: Optional[str],
    arm,
    sdk_lock: threading.Lock,
    mode: str,
    gripper_is_open: Optional[bool],
    gripper_enabled: bool,
) -> Optional[bool]:
    """Execute 'open' or 'close' and return updated gripper_is_open."""
    if action == "close":
        result = close_gripper(arm, sdk_lock, mode, gripper_enabled)
        return result if result is not None else gripper_is_open
    if action == "open":
        result = open_gripper(arm, sdk_lock, mode, gripper_enabled)
        return result if result is not None else gripper_is_open
    return gripper_is_open


def run_sequence(
    arm,
    sdk_lock: threading.Lock,
    sequence_name: str,
    positions: Dict[str, List[float]],
    current_state: str,
    mode: str,
    gripper_is_open: Optional[bool] = None,
    gripper_enabled: bool = True,
) -> Tuple[str, Optional[bool]]:
    if sequence_name not in SEQUENCES:
        print(f"Unknown sequence: {sequence_name}")
        return current_state, gripper_is_open

    sequence = SEQUENCES[sequence_name]
    required_start = sequence["required_start"]
    targets = sequence["targets"]
    label = sequence["label"]
    gripper_before = sequence.get("gripper_before")
    gripper_after  = sequence.get("gripper_after")

    if current_state != required_start:
        print("")
        print("Command rejected.")
        print(f"  Current state  : {current_state}")
        print(f"  Required state : {required_start}")
        print(f"  Sequence       : {label}")
        return current_state, gripper_is_open

    gripper_note = ""
    if gripper_before:
        gripper_note += f"  Gripper BEFORE : {gripper_before}\n"
    if gripper_after:
        gripper_note += f"  Gripper AFTER  : {gripper_after}\n"

    print("")
    print(f"Sequence : {label}")
    print(f"Speed    : {DEFAULT_SPEED}   Acceleration: {DEFAULT_ACCELERATION}")
    if gripper_note:
        print(gripper_note, end="")

    if mode == "physical":
        confirm = input(
            "\nThis will move the REAL robot arm.\n"
            "Make sure:\n"
            "  - Emergency stop is reachable\n"
            "  - Nobody is inside the robot workspace\n"
            "  - UFactory Studio is closed\n"
            "  - The robot is physically clear\n\n"
            f"Type YES to execute '{label}': "
        ).strip()
        if confirm != "YES":
            print("Movement cancelled.")
            return current_state, gripper_is_open
    else:
        confirm = input(f"[VIRTUAL] Simulate '{label}'? [y/n]: ").strip().lower()
        if confirm != "y":
            print("Simulation cancelled.")
            return current_state, gripper_is_open

    if gripper_before and gripper_enabled:
        gripper_is_open = _run_gripper_action(
            gripper_before, arm, sdk_lock, mode, gripper_is_open, gripper_enabled
        )

    for target in targets:
        try:
            move_to_target(
                arm=arm,
                sdk_lock=sdk_lock,
                target_name=target,
                positions=positions,
            )
        except RuntimeError as exc:
            print(f"\nSequence aborted at step '{target}': {exc}")
            return "unknown", gripper_is_open

    if gripper_after and gripper_enabled:
        gripper_is_open = _run_gripper_action(
            gripper_after, arm, sdk_lock, mode, gripper_is_open, gripper_enabled
        )

    final_state = targets[-1]
    print(f"Sequence complete. New state: {final_state}")
    return final_state, gripper_is_open


# ==========================================================
# Universal home recovery
# ==========================================================

def go_home_from_any_state(
    arm,
    sdk_lock: threading.Lock,
    positions: Dict[str, List[float]],
    current_state: str,
    mode: str,
) -> str:
    print("")
    print("Universal home command.")
    print(f"Current state: {current_state}")

    if current_state == "home":
        print("Already at home.")
        return "home"

    if current_state == "pick":
        targets = ["home_pick_turn", "home"]
        label = "pick -> home_pick_turn -> home"
    elif current_state == "sort":
        targets = ["home_sort_turn", "home"]
        label = "sort -> home_sort_turn -> home"
    else:
        targets = ["home"]
        label = f"{current_state} -> home"

    if mode == "physical":
        confirm = input(
            "\nThis will move the REAL robot arm to home.\n"
            "Make sure the path is clear. Emergency stop must be reachable.\n\n"
            f"Path: {label}\n"
            "Type YES to go home: "
        ).strip()
        if confirm != "YES":
            print("Go-home cancelled.")
            return current_state
    else:
        confirm = input(
            f"[VIRTUAL] Simulate go-home via '{label}'? [y/n]: "
        ).strip().lower()
        if confirm != "y":
            print("Go-home cancelled.")
            return current_state

    for target in targets:
        try:
            move_to_target(
                arm=arm,
                sdk_lock=sdk_lock,
                target_name=target,
                positions=positions,
            )
        except RuntimeError as exc:
            print(f"\nGo-home aborted at step '{target}': {exc}")
            return "unknown"

    print("Arrived at home.")
    return "home"


# ==========================================================
# Menu
# ==========================================================

def print_menu(
    current_state: str,
    mode: str,
    gripper_is_open: Optional[bool],
    gripper_enabled: bool,
) -> None:
    mode_tag = "[VIRTUAL]" if mode == "virtual" else "[PHYSICAL]"
    gripper_state = {True: "OPEN", False: "CLOSED"}.get(gripper_is_open, "unknown")  # type: ignore[arg-type]
    gripper_power = "ON" if gripper_enabled else "OFF"
    print("")
    print(f"Guarded Fast Lite6 Controller  {mode_tag}")
    print("=" * 52)
    print(f"Detected state : {current_state}")
    print(f"Gripper power  : {gripper_power}")
    print(f"Gripper        : {gripper_state}")
    print("")
    print("r = read current position")
    print("h = go home from any state")
    print(f"t = turn on gripper   [power: {gripper_power}]")
    print(f"o = open gripper      [now: {gripper_state}]")
    print(f"c = close gripper     [now: {gripper_state}]")
    print(f"s = stop gripper")

    if current_state == "home":
        print("1 = home -> home_pick_turn -> pick")
        print("2 = home -> home_sort_turn -> sort")
    elif current_state == "pick":
        print("3 = pick -> pick_lift -> sort_lift -> sort")
        print("5 = pick -> home_pick_turn -> home")
    elif current_state == "sort":
        print("4 = sort -> sort_lift -> pick_lift -> pick")
        print("6 = sort -> home_sort_turn -> home")
    elif current_state == "safe_crossing":
        print("7 = safe_crossing -> pick")
        print("8 = safe_crossing -> sort")
        print("9 = safe_crossing -> home")
    else:
        print("Robot is not at a named state. Choose a recovery target:")
        print("0 = force move to home")
        print("1 = force move to pick")
        print("2 = force move to sort")
        print("  WARNING: only use if the path to the target is physically clear.")

    print("q = quit")


# ==========================================================
# RViz / collision visualization
# ==========================================================

def run_ros_background(command: str) -> subprocess.Popen:
    full_command = (
        f"cd {PROJECT_ROOT} && "
        "source /opt/ros/jazzy/setup.bash && "
        "source ~/xarm_ws/install/setup.bash && "
        "source install/setup.bash && "
        f"{command}"
    )
    print("\nStarting background ROS command:")
    print(full_command)
    return subprocess.Popen(
        ["bash", "-lc", full_command],
        preexec_fn=os.setsid,
    )


def run_ros_wait(command: str) -> int:
    full_command = (
        f"cd {PROJECT_ROOT} && "
        "source /opt/ros/jazzy/setup.bash && "
        "source ~/xarm_ws/install/setup.bash && "
        "source install/setup.bash && "
        f"{command}"
    )
    print("\nRunning ROS command:")
    print(full_command)
    result = subprocess.run(["bash", "-lc", full_command], text=True, check=False)
    return result.returncode


def start_rviz_and_collision() -> Optional[subprocess.Popen]:
    answer = input("\nStart RViz/workcell visualization? [y/n]: ").strip().lower()
    if answer != "y":
        print("RViz startup skipped.")
        return None

    print("\nStarting RViz...")
    rviz_process = run_ros_background(
        f"ros2 launch {PACKAGE_NAME} {RVIZ_LAUNCH_FILE}"
    )

    print("\nWaiting for scene to settle...")
    time.sleep(6.0)

    print("\nAdding collision objects...")
    rc = run_ros_wait(f"ros2 run {PACKAGE_NAME} {COLLISION_EXECUTABLE}")

    if rc == 0:
        print("Collision objects added.")
    else:
        print("WARNING: Collision object command failed.")
        print("If display_workcell.launch.py has no MoveIt planning scene,")
        print("change RVIZ_LAUNCH_FILE to lite6_workcell_bringup.launch.py.")

    return rviz_process


def stop_background_process(
    process: Optional[subprocess.Popen],
    name: str,
) -> None:
    if process is None:
        return
    print(f"\nStopping {name}...")
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        except Exception:
            pass
    except Exception:
        pass


# ==========================================================
# Joint state publisher for RViz
# ==========================================================

class RealRobotJointStatePublisher:
    """
    Publishes joint angles from the SDK (real or virtual) to /joint_states.

    Uses a non-blocking lock acquire so it never delays a movement command:
    if the SDK lock is held by a move dispatch, the last known position is
    republished instead of blocking.
    """

    def __init__(
        self,
        arm,
        sdk_lock: threading.Lock,
        joint_names: List[str],
    ) -> None:
        self.arm = arm
        self.sdk_lock = sdk_lock
        self.joint_names = joint_names
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self._last_msg = None

        if not rclpy.ok():
            rclpy.init(args=None)

        self.node = rclpy.create_node("sdk_real_joint_state_publisher")
        self.publisher = self.node.create_publisher(JointState, "/joint_states", 10)

    def start(self) -> None:
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        print("Publishing joint states to /joint_states.")

    def _loop(self) -> None:
        sleep_time = 1.0 / JOINT_STATE_RATE_HZ

        while self.running:
            try:
                # Non-blocking acquire: if the SDK lock is held by a move
                # command, skip the read and republish the last known position.
                acquired = self.sdk_lock.acquire(timeout=0.05)

                if acquired:
                    try:
                        code, angles = self.arm.get_servo_angle(is_radian=True)
                    finally:
                        self.sdk_lock.release()

                    if code == 0 and angles is not None and len(angles) >= 6:
                        # Use software-tracked gripper position — never poll
                        # get_gripper_position() on hardware as it re-triggers C19.
                        finger_val = _gripper_raw_pos[0] * _GRIPPER_FINGER_SCALE
                        msg = JointState()
                        msg.header.stamp = self.node.get_clock().now().to_msg()
                        msg.name = self.joint_names + ["finger_joint1"]
                        msg.position = [float(v) for v in angles[:6]] + [finger_val]
                        self._last_msg = msg
                        self.publisher.publish(msg)

                elif self._last_msg is not None:
                    # SDK busy with a command: republish last position to keep
                    # RViz alive without blocking the command.
                    self._last_msg.header.stamp = self.node.get_clock().now().to_msg()
                    self.publisher.publish(self._last_msg)

                rclpy.spin_once(self.node, timeout_sec=0.0)
                time.sleep(sleep_time)

            except Exception as e:
                print(f"WARNING: Joint state publisher error: {e}")
                time.sleep(1.0)

    def stop(self) -> None:
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=2.0)
        self.node.destroy_node()
        print("Stopped joint state publisher.")


# ==========================================================
# Main
# ==========================================================

def main() -> None:
    config = load_named_positions()
    positions = get_positions(config)
    joint_names = list(config["joint_names"])

    print("\nLoaded named positions:")
    for name in positions:
        print(f"  - {name}")

    mode = select_boot_mode()

    rviz_process = start_rviz_and_collision()

    arm = connect_arm(mode)
    sdk_lock = threading.Lock()

    gripper_enabled: bool = turn_on_gripper(arm, sdk_lock, mode)
    gripper_is_open: Optional[bool] = True if gripper_enabled else None

    joint_publisher = None

    if PUBLISH_JOINT_STATES_TO_RVIZ and ROS_AVAILABLE:
        try:
            joint_publisher = RealRobotJointStatePublisher(
                arm=arm,
                sdk_lock=sdk_lock,
                joint_names=joint_names,
            )
            joint_publisher.start()
        except Exception as e:
            print(f"WARNING: Could not start joint state publisher: {e}")

    sequence_map = {
        "1": "home_to_pick",
        "2": "home_to_sort",
        "3": "pick_to_sort",
        "4": "sort_to_pick",
        "5": "pick_to_home",
        "6": "sort_to_home",
    }

    try:
        while True:
            current_joints = read_current_joints(arm=arm, sdk_lock=sdk_lock)
            current_state, error = find_nearest_named_state(
                current_joints=current_joints,
                positions=positions,
            )

            print_current_position(
                current_joints=current_joints,
                nearest_state=current_state,
                nearest_error=error,
            )

            print_menu(current_state, mode, gripper_is_open, gripper_enabled)
            choice = input("Input: ").strip().lower()

            if choice == "q":
                break

            if choice == "r":
                continue

            if choice == "t":
                gripper_enabled = turn_on_gripper(arm, sdk_lock, mode)
                if gripper_enabled:
                    gripper_is_open = True
                continue

            if choice == "o":
                result = open_gripper(arm, sdk_lock, mode, gripper_enabled)
                if result is not None:
                    gripper_is_open = result
                continue

            if choice == "c":
                result = close_gripper(arm, sdk_lock, mode, gripper_enabled)
                if result is not None:
                    gripper_is_open = result
                continue

            if choice == "s":
                if stop_gripper(arm, sdk_lock, mode):
                    gripper_enabled = False
                    gripper_is_open = None
                continue

            if choice == "h":
                current_state = go_home_from_any_state(
                    arm=arm,
                    sdk_lock=sdk_lock,
                    positions=positions,
                    current_state=current_state,
                    mode=mode,
                )
                continue

            if choice in ("0", "1", "2") and current_state == "unknown":
                force_dest = {"0": "home", "1": "pick", "2": "sort"}[choice]

                if mode == "physical":
                    confirm = input(
                        f"\nWARNING: State is unknown.\n"
                        f"This moves the REAL robot directly to '{force_dest}'.\n"
                        f"Only use if the path is physically clear.\n\n"
                        f"Type YES to force move to {force_dest}: "
                    ).strip()
                    proceed = confirm == "YES"
                else:
                    proceed = (
                        input(
                            f"[VIRTUAL] Simulate force move to {force_dest}? [y/n]: "
                        ).strip().lower() == "y"
                    )

                if proceed:
                    try:
                        move_to_target(
                            arm=arm,
                            sdk_lock=sdk_lock,
                            target_name=force_dest,
                            positions=positions,
                        )
                        current_state = force_dest
                    except RuntimeError as exc:
                        print(f"\nForce move to {force_dest} failed: {exc}")
                else:
                    print("Force move cancelled.")
                continue

            if choice in sequence_map:
                current_state, gripper_is_open = run_sequence(
                    arm=arm,
                    sdk_lock=sdk_lock,
                    sequence_name=sequence_map[choice],
                    positions=positions,
                    current_state=current_state,
                    mode=mode,
                    gripper_is_open=gripper_is_open,
                    gripper_enabled=gripper_enabled,
                )
                continue

            if choice in ("7", "8", "9"):
                if current_state != "safe_crossing":
                    print(f"Command {choice} is only allowed from safe_crossing state.")
                    continue

                dest_map = {"7": "pick", "8": "sort", "9": "home"}
                dest = dest_map[choice]

                if mode == "physical":
                    confirm = input(
                        f"Type YES to move safe_crossing -> {dest}: "
                    ).strip()
                    proceed = confirm == "YES"
                else:
                    proceed = (
                        input(
                            f"[VIRTUAL] Simulate safe_crossing -> {dest}? [y/n]: "
                        ).strip().lower() == "y"
                    )

                if proceed:
                    move_to_target(
                        arm=arm,
                        sdk_lock=sdk_lock,
                        target_name=dest,
                        positions=positions,
                    )
                continue

            print("Invalid input.")

    finally:
        if joint_publisher is not None:
            joint_publisher.stop()

        arm.disconnect()
        if mode == "physical":
            print("Disconnected from Lite6.")
        else:
            print("Virtual session ended.")

        stop_background_process(rviz_process, "RViz/workcell visualization")

        if ROS_AVAILABLE and rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
