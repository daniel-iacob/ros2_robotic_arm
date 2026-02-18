import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA


class SceneManager(Node):
    def __init__(self):
        super().__init__("scene_manager")
        # We must use /planning_scene to send colors
        self.pub = self.create_publisher(PlanningScene, "/planning_scene", 10)

        # Wait longer for MoveIt to be ready (increased from 2 to 5 seconds)
        self.get_logger().info("Waiting for MoveIt to initialize...")
        time.sleep(5.0)

    def spawn_blue_cube(self):
        # 1. Define the Cube
        cube = CollisionObject()
        cube.header.frame_id = "base_link"
        cube.id = "blue_piece"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.05, 0.05, 0.05]

        pose = Pose()
        pose.position.x = 0.5
        pose.position.y = 0.0
        pose.position.z = 0.3
        pose.orientation.w = 1.0

        cube.primitives.append(primitive)
        cube.primitive_poses.append(pose)
        cube.operation = CollisionObject.ADD

        # 2. Define the Color (The "Colorizer" part)
        color = ObjectColor()
        color.id = "blue_piece"  # MUST match the cube ID exactly
        color.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)  # Pure Blue, Alpha 1.0

        # 3. Wrap everything into the Scene message
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.world.collision_objects.append(cube)
        scene_msg.object_colors.append(color)  # This "paints" the object

        # 4. Send it
        self.pub.publish(scene_msg)
        self.get_logger().info("Published BLUE cube to the planning scene!")

    def spawn_red_cube(self):
        # 1. Define the Cube
        cube = CollisionObject()
        cube.header.frame_id = "base_link"
        cube.id = "red_piece"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.05, 0.05, 0.05]

        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.5
        pose.position.z = 0.3
        pose.orientation.w = 1.0

        cube.primitives.append(primitive)
        cube.primitive_poses.append(pose)
        cube.operation = CollisionObject.ADD

        # 2. Define the Color (The "Colorizer" part)
        color = ObjectColor()
        color.id = "red_piece"  # MUST match the cube ID exactly
        color.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Pure Red, Alpha 1.0

        # 3. Wrap everything into the Scene message
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.world.collision_objects.append(cube)
        scene_msg.object_colors.append(color)  # This "paints" the object

        # 4. Send it
        self.pub.publish(scene_msg)
        self.get_logger().info("Published RED cube to the planning scene!")


def main():
    rclpy.init()
    node = SceneManager()

    # Publish multiple times to ensure MoveIt receives them
    node.get_logger().info("Publishing cubes to planning scene...")
    for i in range(3):
        node.spawn_blue_cube()
        node.spawn_red_cube()
        time.sleep(0.5)

    node.get_logger().info("Scene setup complete!")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
