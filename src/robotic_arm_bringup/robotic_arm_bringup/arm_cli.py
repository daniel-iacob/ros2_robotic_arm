#!/usr/bin/env python3
"""Thin CLI client for the motion_server action server.

Sends action goals and prints feedback/results. The actual arm control
logic runs in the persistent motion_server node.

Usage:
    ros2 run robotic_arm_bringup arm pick blue_cube
    ros2 run robotic_arm_bringup arm place blue_cube
    ros2 run robotic_arm_bringup arm place blue_cube 0.4 0.1 0.35
    ros2 run robotic_arm_bringup arm move-to 0.4 0.1 0.3
    ros2 run robotic_arm_bringup arm home
    ros2 run robotic_arm_bringup arm open-gripper
    ros2 run robotic_arm_bringup arm close-gripper
    ros2 run robotic_arm_bringup arm reset
    ros2 run robotic_arm_bringup arm list-objects
"""

import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from robotic_arm_interfaces.action import (
    CloseGripper,
    Home,
    MoveTo,
    OpenGripper,
    Pick,
    Place,
    Reset,
)
from robotic_arm_interfaces.srv import ListObjects, MoveObject


def _send_action_goal(node, action_type, action_name, goal_msg, timeout=60.0):
    """Send an action goal, print feedback, return (success, message)."""
    logger = node.get_logger()
    client = ActionClient(node, action_type, action_name)

    if not client.wait_for_server(timeout_sec=10.0):
        logger.error(f"Action server '{action_name}' not available. Is motion_server running?")
        return False, "Server not available"

    def feedback_callback(feedback_msg):
        fb = feedback_msg.feedback
        logger.info(f"  [{fb.progress:.0%}] {fb.step}")

    send_future = client.send_goal_async(goal_msg, feedback_callback=feedback_callback)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=10.0)

    if not send_future.done():
        logger.error("Failed to send goal (timeout)")
        return False, "Send timeout"

    goal_handle = send_future.result()
    if not goal_handle.accepted:
        logger.error("Goal rejected by server")
        return False, "Goal rejected"

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=timeout)

    if not result_future.done():
        logger.error("Action timeout")
        return False, "Execution timeout"

    result = result_future.result().result
    return result.success, result.message


def _call_list_objects(node):
    """Call the list_objects service and print results."""
    logger = node.get_logger()
    client = node.create_client(ListObjects, "/list_objects")

    if not client.wait_for_service(timeout_sec=10.0):
        logger.error("ListObjects service not available. Is motion_server running?")
        return False

    future = client.call_async(ListObjects.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

    if not future.done() or future.result() is None:
        logger.error("Failed to query objects")
        return False

    objects = future.result().objects
    if not objects:
        logger.info("No objects in scene")
        return True

    logger.info(f"Objects in scene ({len(objects)}):")
    for obj in objects:
        if obj.held:
            logger.info(f"  {obj.name}: (held)")
        else:
            logger.info(f"  {obj.name}: ({obj.x:.3f}, {obj.y:.3f}, {obj.z:.3f})")
    return True


def _call_move_object(node, name, x, y, z):
    """Call the move_object service."""
    logger = node.get_logger()
    client = node.create_client(MoveObject, "/move_object")

    if not client.wait_for_service(timeout_sec=10.0):
        logger.error("MoveObject service not available. Is motion_server running?")
        return False, "Service not available"

    req = MoveObject.Request()
    req.name = name
    req.x = x
    req.y = y
    req.z = z

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

    if not future.done() or future.result() is None:
        return False, "Service call failed"

    result = future.result()
    return result.success, result.message


def main(args=None):
    parser = argparse.ArgumentParser(description="Robotic arm control CLI")
    sub = parser.add_subparsers(dest="command", required=True)

    # pick <object>
    p = sub.add_parser("pick", help="Pick up an object")
    p.add_argument("object", type=str, help="Object name (e.g. blue_cube)")

    # place <object> [x y [z]]
    p = sub.add_parser("place", help="Release object at position")
    p.add_argument("object", type=str, help="Object name")
    p.add_argument("x", type=float, nargs="?", default=None, help="X position")
    p.add_argument("y", type=float, nargs="?", default=None, help="Y position")
    p.add_argument("z", type=float, nargs="?", default=None, help="Z position")

    # home
    sub.add_parser("home", help="Return arm to home position")

    # open-gripper
    sub.add_parser("open-gripper", help="Open gripper")

    # close-gripper
    sub.add_parser("close-gripper", help="Close gripper")

    # reset
    sub.add_parser("reset", help="Full recovery: home + reset scene to original state")

    # list-objects
    sub.add_parser("list-objects", help="List all objects in the planning scene")

    # move-to <x> <y> <z>
    p = sub.add_parser("move-to", help="Move end-effector to position")
    p.add_argument("x", type=float, help="X position")
    p.add_argument("y", type=float, help="Y position")
    p.add_argument("z", type=float, help="Z position")

    # move-object <name> <x> <y> <z>
    p = sub.add_parser("move-object", help="Move an object in the planning scene")
    p.add_argument("object", type=str, help="Object name")
    p.add_argument("x", type=float, help="X position")
    p.add_argument("y", type=float, help="Y position")
    p.add_argument("z", type=float, help="Z position")

    parsed = parser.parse_args()

    rclpy.init(args=args)
    node = Node("arm_cli")
    logger = node.get_logger()

    try:
        if parsed.command == "list-objects":
            success = _call_list_objects(node)
            exit_code = 0 if success else 1
        elif parsed.command == "move-object":
            success, msg = _call_move_object(
                node, parsed.object, parsed.x, parsed.y, parsed.z
            )
            if success:
                logger.info(f"Done: {msg}")
                exit_code = 0
            else:
                logger.error(f"Failed: {msg}")
                exit_code = 1
        else:
            if parsed.command == "pick":
                goal = Pick.Goal(object_name=parsed.object)
                success, msg = _send_action_goal(node, Pick, "/pick", goal)

            elif parsed.command == "place":
                goal = Place.Goal(object_name=parsed.object)
                if parsed.x is not None and parsed.y is not None:
                    goal.x = parsed.x
                    goal.y = parsed.y
                    goal.z = parsed.z if parsed.z is not None else 0.0
                success, msg = _send_action_goal(node, Place, "/place", goal)

            elif parsed.command == "move-to":
                goal = MoveTo.Goal(x=parsed.x, y=parsed.y, z=parsed.z)
                success, msg = _send_action_goal(node, MoveTo, "/move_to", goal)

            elif parsed.command == "home":
                success, msg = _send_action_goal(node, Home, "/home", Home.Goal())

            elif parsed.command == "reset":
                success, msg = _send_action_goal(node, Reset, "/reset", Reset.Goal())

            elif parsed.command == "open-gripper":
                success, msg = _send_action_goal(
                    node, OpenGripper, "/open_gripper", OpenGripper.Goal(),
                )

            elif parsed.command == "close-gripper":
                success, msg = _send_action_goal(
                    node, CloseGripper, "/close_gripper", CloseGripper.Goal(),
                )

            if success:
                logger.info(f"Done: {msg}")
                exit_code = 0
            else:
                logger.error(f"Failed: {msg}")
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
