#!/usr/bin/env python3
"""Thin CLI wrapper for ArmController.

Usage:
    ros2 run robotic_arm_bringup arm pick blue_cube
    ros2 run robotic_arm_bringup arm place blue_cube 0.4 0.1
    ros2 run robotic_arm_bringup arm place blue_cube 0.4 0.1 0.35
    ros2 run robotic_arm_bringup arm home
    ros2 run robotic_arm_bringup arm open-gripper
    ros2 run robotic_arm_bringup arm close-gripper
    ros2 run robotic_arm_bringup arm reset
    ros2 run robotic_arm_bringup arm move-to 0.5 0.0 0.4
"""

import argparse
import sys

import rclpy
from rclpy.node import Node

from robotic_arm_bringup.arm_controller import ArmController


def main(args=None):
    parser = argparse.ArgumentParser(description="Robotic arm control CLI")
    sub = parser.add_subparsers(dest="command", required=True)

    # pick <object>
    p = sub.add_parser("pick", help="Pick up an object")
    p.add_argument("object", type=str, help="Object name (e.g. blue_cube)")

    # place <object> <x> <y> [z]
    p = sub.add_parser("place", help="Pick and place an object")
    p.add_argument("object", type=str, help="Object name")
    p.add_argument("x", type=float, help="Target X position")
    p.add_argument("y", type=float, help="Target Y position")
    p.add_argument("z", type=float, nargs="?", default=None, help="Target Z position (default: same as source)")

    # home
    sub.add_parser("home", help="Return arm to home position")

    # open-gripper
    sub.add_parser("open-gripper", help="Open gripper")

    # close-gripper
    sub.add_parser("close-gripper", help="Close gripper")

    # reset
    sub.add_parser("reset", help="Full recovery: home + reset scene to original state")

    # list-objects
    sub.add_parser("list-objects", help="List all objects currently in the MoveIt planning scene")

    # move-to <x> <y> <z>
    p = sub.add_parser("move-to", help="Move end-effector to position")
    p.add_argument("x", type=float, help="X position")
    p.add_argument("y", type=float, help="Y position")
    p.add_argument("z", type=float, help="Z position")

    parsed = parser.parse_args()

    rclpy.init(args=args)
    node = Node("arm_controller_node")
    logger = node.get_logger()

    try:
        controller = ArmController(node)
        success = False

        if parsed.command == "pick":
            success = controller.pick(parsed.object)
        elif parsed.command == "place":
            success = controller.place(parsed.object, parsed.x, parsed.y, parsed.z)
        elif parsed.command == "home":
            success = controller.home()
        elif parsed.command == "open-gripper":
            success = controller.open_gripper()
        elif parsed.command == "close-gripper":
            success = controller.close_gripper()
        elif parsed.command == "reset":
            success = controller.reset()
        elif parsed.command == "list-objects":
            success = controller.list_objects()
        elif parsed.command == "move-to":
            success = controller.move_to(parsed.x, parsed.y, parsed.z)

        if success:
            logger.info("Done.")
            exit_code = 0
        else:
            logger.error("Failed.")
            exit_code = 1

    except KeyboardInterrupt:
        logger.info("Interrupted")
        exit_code = 130
    except Exception as e:
        logger.error(f"Error: {e}")
        import traceback
        traceback.print_exc()
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
