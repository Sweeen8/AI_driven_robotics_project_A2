#!/usr/bin/env python3

import sys

from functions import (
    choose_mode,
    command_menu,
    setup_environment,
    stop_process,
)


def main() -> int:
    print("\nLite 6 Robot Controller")
    print("=======================")
    print("This controller starts either the virtual robot or the real robot.")
    print("It uses a state-based workflow:")
    print("  home -> pick")
    print("  home -> sort")
    print("  pick -> sort with Cartesian arch")
    print("  sort -> pick with Cartesian arch")

    mode = choose_mode()

    if mode is None:
        print("Program cancelled.")
        return 0

    bringup_process = None
    monitor_process = None
    robot_state = "unknown"

    try:
        bringup_process, monitor_process, robot_state = setup_environment(mode)

        print("\nYou can now use the robot command menu.")
        command_menu(mode, robot_state)

    except KeyboardInterrupt:
        print("\nRobot controller interrupted by user.")

    finally:
        stop_process(monitor_process, "robot position monitor")
        stop_process(bringup_process, "bringup")

        print("\nRobot controller finished.")

    return 0


if __name__ == "__main__":
    sys.exit(main())