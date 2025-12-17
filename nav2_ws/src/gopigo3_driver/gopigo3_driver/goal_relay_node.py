#!/usr/bin/env python3
"""
Goal Relay Node - Ensures all goal poses use Time(0) timestamp.

This node intercepts goal poses and republishes them with Time(0) timestamp,
which tells Nav2 to use the latest available TF transform. This prevents
TF extrapolation errors during replanning.

The node subscribes to goals from Foxglove and republishes to Nav2's internal topic.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
import tf2_ros
import tf2_geometry_msgs


class GoalRelayNode(Node):
    def __init__(self):
        super().__init__('goal_relay_node')

        # Flag to prevent infinite loop (we publish to a topic we also subscribe to)
        self._publishing = False

        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribe to goals from Foxglove (both topics)
        self.goal_sub_simple = self.create_subscription(
            PoseStamped,
            '/move_base_simple/goal',
            self.goal_callback_simple,
            10
        )

        # Subscribe to /goal_pose to intercept Foxglove direct publishes
        self.goal_sub_pose = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback_pose,
            10
        )

        # Publish to Nav2's internal goal topic (bt_navigator subscribes to this)
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose_internal',
            10
        )

        self.get_logger().info('Goal Relay Node started')
        self.get_logger().info('  Subscribing to: /move_base_simple/goal, /goal_pose')
        self.get_logger().info('  Publishing to: /goal_pose_internal (with Time(0))')

    def goal_callback_simple(self, msg: PoseStamped):
        """Handle goals from /move_base_simple/goal."""
        self._process_goal(msg, '/move_base_simple/goal')

    def goal_callback_pose(self, msg: PoseStamped):
        """Handle goals from /goal_pose (intercept Foxglove direct publishes)."""
        # Skip if this is our own publish or if timestamp is already 0
        if self._publishing:
            return
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            return
        self._process_goal(msg, '/goal_pose')

    def _process_goal(self, msg: PoseStamped, source_topic: str):
        """Process goal and republish with Time(0) timestamp."""
        source_frame = msg.header.frame_id
        target_frame = 'map'

        # Use Time(0) to tell Nav2 to use latest transform
        zero_stamp = Time(sec=0, nanosec=0)

        # If frame_id is empty or 'map', republish directly with zero timestamp
        if source_frame == '' or source_frame == 'map':
            new_msg = PoseStamped()
            new_msg.header.frame_id = 'map'
            new_msg.header.stamp = zero_stamp
            new_msg.pose = msg.pose

            self._publishing = True
            self.goal_pub.publish(new_msg)
            self._publishing = False

            self.get_logger().info(
                f'Goal from {source_topic} relayed with timestamp=0: '
                f'({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
            )
            return

        try:
            # Get transform from source frame to map
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # Transform the pose
            transformed_pose = tf2_geometry_msgs.do_transform_pose_stamped(msg, transform)
            transformed_pose.header.frame_id = 'map'
            transformed_pose.header.stamp = zero_stamp

            self._publishing = True
            self.goal_pub.publish(transformed_pose)
            self._publishing = False

            self.get_logger().info(
                f'Goal from {source_topic} transformed: {source_frame} -> map, timestamp=0: '
                f'({transformed_pose.pose.position.x:.2f}, {transformed_pose.pose.position.y:.2f})'
            )

        except tf2_ros.LookupException as e:
            self.get_logger().error(f'Transform lookup failed: {e}')
        except tf2_ros.ConnectivityException as e:
            self.get_logger().error(f'Transform connectivity error: {e}')
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().error(f'Transform extrapolation error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = GoalRelayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
