#!/usr/bin/env python3

"""
Reusable helper functions for the Lite 6 robot controller.

Final workflow:
- Start virtual or real environment
- Close old RViz windows first
- Add collision objects
- Start position monitor
- Move robot to home
- Use state-based menu:
  home -> pick
  home -> sort
  pick -> sort with Cartesian arch
  sort -> pick with Cartesian arch
"""

import os
import signal
import subprocess
import time
from typing import Optional


# ==========================================================
# Configuration
# ==========================================================

ROBOT_IP = "192.168.1.155"

PACKAGE_NAME = "a2_workcell_description"

VIRTUAL_BRINGUP_LAUNCH = "lite6_workcell_bringup.launch.py"
REAL_BRINGUP_LAUNCH = "lite6_workcell_real_bringup.launch.py"

COLLISION_EXECUTABLE = "add_workcell_collision_objects"
COMMANDER_EXECUTABLE = "named_position_commander.py"
MONITOR_EXECUTABLE = "robot_position_monitor.py"
CARTESIAN_ARCH_EXECUTABLE = "cartesian_sort_arch_commander.py"

CONTROLLER_MANAGER = "/controller_manager"
TRAJECTORY_CONTROLLER = "lite6_traj_controller"

CURRENT_STATE_TOPIC = "/robot_monitor/current_state_text"
PLANNED_STATE_TOPIC = "/robot_monitor/planned_state_text"

DEFAULT_ARCH_Z = 1.35

REFERENCE_FRAME = "environment_root"
END_EFFECTOR_FRAME = "link_eef"


# ==========================================================
# General command helpers
# ==========================================================

def run_command(command: list[str], wait: bool = True) -> Optional[subprocess.Popen]:
    """
    Run a terminal command.

    If wait=True, the function waits until the command finishes.
    If wait=False, the command keeps running in the background.

    Background processes are started in their own process group so they
    can be stopped cleanly later.
    """

    print("\n==================================================")
    print("Running:")
    print(" ".join(command))
    print("==================================================\n")

    if wait:
        subprocess.run(command, check=False)
        return None

    return subprocess.Popen(
        command,
        preexec_fn=os.setsid,
    )


def run_capture(command: list[str]) -> tuple[int, str, str]:
    """
    Run a command and capture stdout/stderr.
    """

    result = subprocess.run(
        command,
        text=True,
        capture_output=True,
        check=False,
    )

    return result.returncode, result.stdout, result.stderr


def wait_for_node(node_name: str, timeout_seconds: int = 60) -> bool:
    """
    Wait until a ROS 2 node appears in the graph.
    """

    print(f"Waiting for node: {node_name}")

    start_time = time.time()

    while time.time() - start_time < timeout_seconds:
        returncode, stdout, _ = run_capture(["ros2", "node", "list"])

        if returncode == 0 and node_name in stdout:
            print(f"Node found: {node_name}")
            return True

        time.sleep(1.0)

    print(f"WARNING: Node not found within timeout: {node_name}")
    return False


def wait_for_service(service_name: str, timeout_seconds: int = 60) -> bool:
    """
    Wait until a ROS 2 service appears in the graph.
    """

    print(f"Waiting for service: {service_name}")

    start_time = time.time()

    while time.time() - start_time < timeout_seconds:
        returncode, stdout, _ = run_capture(["ros2", "service", "list"])

        if returncode == 0 and service_name in stdout:
            print(f"Service found: {service_name}")
            return True

        time.sleep(1.0)

    print(f"WARNING: Service not found within timeout: {service_name}")
    return False


def wait_for_tf_transform(
    reference_frame: str = REFERENCE_FRAME,
    end_effector_frame: str = END_EFFECTOR_FRAME,
    timeout_seconds: int = 20,
) -> bool:
    """
    Wait until TF can provide environment_root -> link_eef.

    This prevents planning from an unknown position.
    """

    print(
        f"\nChecking TF transform: "
        f"{reference_frame} -> {end_effector_frame}"
    )

    command = [
        "timeout",
        str(timeout_seconds),
        "ros2",
        "run",
        "tf2_ros",
        "tf2_echo",
        reference_frame,
        end_effector_frame,
    ]

    result = subprocess.run(
        command,
        text=True,
        capture_output=True,
        check=False,
    )

    output = result.stdout + result.stderr

    if "Translation:" in output or "At time" in output:
        print("TF transform is available.")
        return True

    print("ERROR: TF transform is not available.")
    print(output)
    return False


def topic_echo_once(topic_name: str, timeout_seconds: int = 5) -> bool:
    """
    Echo one message from a topic.
    """

    command = [
        "timeout",
        str(timeout_seconds),
        "ros2",
        "topic",
        "echo",
        "--once",
        topic_name,
        "--field",
        "data",
    ]

    result = subprocess.run(command, check=False)

    if result.returncode != 0:
        print(f"WARNING: Could not read topic: {topic_name}")
        return False

    return True


# ==========================================================
# RViz / startup / shutdown helpers
# ==========================================================

def close_existing_rviz() -> None:
    """
    Close existing RViz windows before launching the environment.

    This prevents multiple RViz instances from stacking up when the
    controller is restarted.
    """

    print("\nChecking for existing RViz windows...")

    result = subprocess.run(
        ["pgrep", "-af", "rviz2"],
        text=True,
        capture_output=True,
        check=False,
    )

    if result.returncode != 0:
        print("No existing RViz window found.")
        return

    print("Existing RViz process found:")
    print(result.stdout)

    print("Closing existing RViz windows...")
    subprocess.run(["pkill", "-f", "rviz2"], check=False)

    time.sleep(2.0)


def launch_bringup(mode: str) -> subprocess.Popen:
    """
    Start either the virtual/fake robot or the real robot bringup.

    The launch files themselves handle RViz.
    This function does not start rviz2 manually.
    """

    if mode == "virtual":
        command = [
            "ros2",
            "launch",
            PACKAGE_NAME,
            VIRTUAL_BRINGUP_LAUNCH,
        ]

        return run_command(command, wait=False)

    if mode == "real":
        command = [
            "ros2",
            "launch",
            PACKAGE_NAME,
            REAL_BRINGUP_LAUNCH,
            f"robot_ip:={ROBOT_IP}",
        ]

        return run_command(command, wait=False)

    raise ValueError(f"Unknown mode: {mode}")


def start_position_monitor() -> subprocess.Popen:
    """
    Start robot_position_monitor.py in the background.
    """

    print("\nStarting robot position monitor...")

    return subprocess.Popen(
        [
            "ros2",
            "run",
            PACKAGE_NAME,
            MONITOR_EXECUTABLE,
        ],
        preexec_fn=os.setsid,
    )


def stop_process(process: Optional[subprocess.Popen], name: str) -> None:
    """
    Stop a background process safely.
    """

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


def add_collision_objects() -> bool:
    """
    Add workcell collision objects to MoveIt's planning scene.
    """

    print("\nAdding workcell collision objects...")

    result = subprocess.run(
        [
            "ros2",
            "run",
            PACKAGE_NAME,
            COLLISION_EXECUTABLE,
        ],
        check=False,
    )

    if result.returncode == 0:
        print("Collision object command finished.")
        return True

    print("ERROR: Collision object command failed.")
    return False


def setup_environment(mode: str) -> tuple[
    Optional[subprocess.Popen],
    Optional[subprocess.Popen],
    str,
]:
    """
    Start the selected environment and prepare the robot controller workflow.

    Returns:
    - bringup_process
    - monitor_process
    - current logical robot state
    """

    close_existing_rviz()

    print(f"\nLaunching {mode} bringup...")
    bringup_process = launch_bringup(mode)

    print("\nWaiting for MoveIt to start...")
    wait_for_node("/move_group", timeout_seconds=60)

    if mode == "real":
        wait_for_service(
            "/controller_manager/switch_controller",
            timeout_seconds=60,
        )

    print("\nWaiting for RViz, TF and MoveIt to settle...")
    time.sleep(5.0)

    monitor_process = start_position_monitor()
    time.sleep(2.0)

    add_collision_objects()

    if not ensure_controller_active(mode):
        print("Controller is not active. Continuing is not recommended.")
        answer = input("Continue anyway? y/n: ").strip().lower()

        if answer != "y":
            return bringup_process, monitor_process, "unknown"

    print("\nSetup complete.")

    print("\nStartup movement: moving robot to home position.")

    moved_home = safe_plan_and_execute_named_position("home", mode)

    if not moved_home:
        print("WARNING: Robot did not reach home during startup.")
        print("Logical robot state is unknown.")
        return bringup_process, monitor_process, "unknown"

    print("Robot is now at home position.")
    return bringup_process, monitor_process, "home"


# ==========================================================
# Position readings
# ==========================================================

def print_current_robot_pose_once() -> None:
    """
    Print the current end-effector pose and joint positions.
    """

    print("\nCurrent robot end-effector position:")
    topic_echo_once(CURRENT_STATE_TOPIC)


def print_planned_robot_pose_once() -> None:
    """
    Print the latest planned final end-effector pose and joint positions.
    """

    print("\nLatest planned final robot position:")
    topic_echo_once(PLANNED_STATE_TOPIC)


# ==========================================================
# Controller helpers
# ==========================================================

def get_controller_state() -> Optional[str]:
    """
    Read the state of the Lite 6 trajectory controller.

    Returns:
    - active
    - inactive
    - missing
    - None if the command failed
    """

    returncode, stdout, stderr = run_capture(
        [
            "ros2",
            "control",
            "list_controllers",
            "--controller-manager",
            CONTROLLER_MANAGER,
        ]
    )

    if returncode != 0:
        print("WARNING: Could not list controllers.")
        print(stderr)
        return None

    print("\nController list:")
    print(stdout)

    for line in stdout.splitlines():
        if line.startswith(TRAJECTORY_CONTROLLER):
            if " active" in line:
                return "active"

            if " inactive" in line:
                return "inactive"

    return "missing"


def activate_controller() -> bool:
    """
    Activate lite6_traj_controller through controller_manager.
    """

    print("\nActivating Lite 6 trajectory controller...")

    command = [
        "ros2",
        "service",
        "call",
        "/controller_manager/switch_controller",
        "controller_manager_msgs/srv/SwitchController",
        "{activate_controllers: ['lite6_traj_controller'], deactivate_controllers: [], strictness: 2, activate_asap: true, timeout: {sec: 5, nanosec: 0}}",
    ]

    result = subprocess.run(command, check=False)

    if result.returncode == 0:
        print("Controller activation service call finished.")
        return True

    print("ERROR: Controller activation service call failed.")
    return False


def ensure_controller_active(mode: str) -> bool:
    """
    In real mode, make sure the physical robot trajectory controller is active.
    In virtual mode, this check is skipped.
    """

    if mode == "virtual":
        print("Virtual mode selected. Skipping real-controller activation check.")
        return True

    state = get_controller_state()

    if state == "active":
        print("Controller is active.")
        return True

    if state == "inactive":
        print("Controller is inactive. Trying to activate it...")
        activate_controller()
        time.sleep(1.0)

        state = get_controller_state()

        if state == "active":
            print("Controller is now active.")
            return True

        print("ERROR: Controller is still not active.")
        return False

    print(f"ERROR: Controller state is invalid: {state}")
    return False


# ==========================================================
# Named MoveIt command helpers
# ==========================================================

def plan_named_position(target: str) -> bool:
    """
    Plan to a predefined named position.

    This does not move the physical robot.
    """

    print(f"\nPlanning target: {target}")
    print("This should NOT move the physical robot.")

    command = [
        "ros2",
        "run",
        PACKAGE_NAME,
        COMMANDER_EXECUTABLE,
        "--ros-args",
        "-p",
        f"target:={target}",
        "-p",
        "execute:=false",
    ]

    result = subprocess.run(command, check=False)

    if result.returncode == 0:
        print(f"Planning command for '{target}' finished.")
        return True

    print(f"ERROR: Planning command for '{target}' failed.")
    return False


def execute_named_position(target: str, mode: str) -> bool:
    """
    Execute movement to a predefined named position.
    """

    print(f"\nExecuting target: {target}")

    if mode == "real":
        print("\nWARNING: Real robot mode is active.")
        print("This command can move the physical Lite 6 robot arm.")

        confirm = input(
            "\nBefore executing, make sure:\n"
            "  - Emergency stop is reachable\n"
            "  - Nobody is inside the robot workspace\n"
            "  - The planned path in RViz is safe\n"
            "  - The physical robot matches the RViz start position\n\n"
            f"Type YES to execute movement to '{target}': "
        ).strip()

        if confirm != "YES":
            print("Execution cancelled.")
            return False

    else:
        print("Virtual mode: execution only moves the simulated/fake robot state.")

    command = [
        "ros2",
        "run",
        PACKAGE_NAME,
        COMMANDER_EXECUTABLE,
        "--ros-args",
        "-p",
        f"target:={target}",
        "-p",
        "execute:=true",
    ]

    result = subprocess.run(command, check=False)

    if result.returncode == 0:
        print(f"Execution command for '{target}' finished.")
        return True

    print(f"ERROR: Execution command for '{target}' failed.")
    return False


def safe_plan_and_execute_named_position(target: str, mode: str) -> bool:
    """
    Plan to a named position first.
    If planning succeeds, execute the named position.
    """

    print(f"\nPreparing movement to: {target}")

    if not wait_for_tf_transform():
        print("Movement cancelled because current robot TF is unknown.")
        return False

    planned = plan_named_position(target)

    if not planned:
        print(f"Planning to '{target}' failed. Execution cancelled.")
        return False

    print(f"Planning to '{target}' succeeded.")

    executed = execute_named_position(target, mode)

    if not executed:
        print(f"Execution to '{target}' failed.")
        return False

    print(f"Movement to '{target}' completed.")
    return True


# ==========================================================
# Cartesian arch movement
# ==========================================================

def run_cartesian_arch(direction: str, mode: str) -> bool:
    """
    Run Cartesian arch movement between pick and sort.

    direction:
    - pick_to_sort
    - sort_to_pick

    This function treats low Cartesian fraction as failure.
    """

    if direction not in ["pick_to_sort", "sort_to_pick"]:
        print(f"Invalid Cartesian direction: {direction}")
        return False

    print(f"\nPreparing Cartesian arch movement: {direction}")

    if not wait_for_tf_transform():
        print("Cartesian movement cancelled because current robot TF is unknown.")
        return False

    if mode == "real":
        print("\nWARNING: Real robot mode is active.")
        print("This Cartesian movement can move the physical robot.")
        print("Make sure the emergency stop is reachable.")
        print("Only continue if the robot is already on the correct side.")

        confirm = input(
            f"Type YES to execute Cartesian movement '{direction}': "
        ).strip()

        if confirm != "YES":
            print("Cartesian execution cancelled.")
            return False

    command = [
        "ros2",
        "run",
        PACKAGE_NAME,
        CARTESIAN_ARCH_EXECUTABLE,
        "--ros-args",
        "-p",
        f"direction:={direction}",
        "-p",
        f"arch_z:={DEFAULT_ARCH_Z}",
        "-p",
        "execute:=true",
    ]

    result = subprocess.run(
        command,
        text=True,
        capture_output=True,
        check=False,
    )

    output = result.stdout + result.stderr
    print(output)

    failed_messages = [
        "Cartesian path fraction is too low",
        "The path is incomplete",
        "Cartesian arch execution failed",
        "Execution goal rejected",
        "Could not get current pose",
        "Could not lookup transform",
        "Action server /execute_trajectory is not available",
    ]

    for message in failed_messages:
        if message in output:
            print(f"Cartesian movement '{direction}' failed.")
            return False

    if result.returncode != 0:
        print(f"Cartesian movement '{direction}' failed.")
        return False

    print(f"Cartesian movement '{direction}' completed.")
    return True


# ==========================================================
# Menu helpers
# ==========================================================

def choose_mode() -> Optional[str]:
    """
    Let the user choose virtual mode or real robot mode.
    """

    while True:
        print("\nChoose startup mode:")
        print("  1 = Virtual environment / fake robot")
        print("  2 = RealRobot / physical robot manipulation")
        print("  q = quit")

        choice = input("Input: ").strip().lower()

        if choice == "1":
            return "virtual"

        if choice == "2":
            return "real"

        if choice == "q":
            return None

        print("Invalid input. Choose 1, 2, or q.")


def command_menu(mode: str, robot_state: str) -> bool:
    """
    State-based main menu.

    robot_state:
    - unknown
    - home
    - pick
    - sort

    Only valid commands are shown.
    """

    while True:
        print("\nRobot command menu")
        print("==================")
        print(f"Current mode: {mode}")
        print(f"Logical robot state: {robot_state}")
        print("")
        print("  r = read current robot position")

        if robot_state == "unknown":
            print("  1 = move to home")

        elif robot_state == "home":
            print("  1 = move home -> pick")
            print("  2 = move home -> sort")

        elif robot_state == "pick":
            print("  1 = move pick -> home")
            print("  2 = Cartesian pick -> sort")

        elif robot_state == "sort":
            print("  1 = move sort -> home")
            print("  2 = Cartesian sort -> pick")

        print("  c = check controller")
        print("  a = activate controller")
        print("  q = quit program")

        choice = input("Input: ").strip().lower()

        if choice == "q":
            return False

        if choice == "r":
            print_current_robot_pose_once()
            continue

        if choice == "c":
            get_controller_state()
            continue

        if choice == "a":
            if mode == "real":
                activate_controller()
            else:
                print("Virtual mode: controller activation is not required.")
            continue

        if robot_state == "unknown":
            if choice == "1":
                if safe_plan_and_execute_named_position("home", mode):
                    robot_state = "home"
            else:
                print("Invalid option for current state.")
            continue

        if robot_state == "home":
            if choice == "1":
                if safe_plan_and_execute_named_position("pick", mode):
                    robot_state = "pick"

            elif choice == "2":
                if safe_plan_and_execute_named_position("sort", mode):
                    robot_state = "sort"

            else:
                print("Invalid option for current state.")
            continue

        if robot_state == "pick":
            if choice == "1":
                if safe_plan_and_execute_named_position("home", mode):
                    robot_state = "home"

            elif choice == "2":
                if run_cartesian_arch("pick_to_sort", mode):
                    robot_state = "sort"

            else:
                print("Invalid option for current state.")
            continue

        if robot_state == "sort":
            if choice == "1":
                if safe_plan_and_execute_named_position("home", mode):
                    robot_state = "home"

            elif choice == "2":
                if run_cartesian_arch("sort_to_pick", mode):
                    robot_state = "pick"

            else:
                print("Invalid option for current state.")
            continue