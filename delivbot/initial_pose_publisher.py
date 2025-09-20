#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class InitialPosePublisher(Node):
    """Publish a single PoseStamped on a configurable topic after a short delay."""

    def __init__(self) -> None:
        super().__init__('initial_pose_publisher')

        self.declare_parameter('topic', 'initial_pose')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('qx', 0.0)
        self.declare_parameter('qy', 0.0)
        self.declare_parameter('qz', 0.0)
        self.declare_parameter('qw', 1.0)
        self.declare_parameter('delay', 0.5)

        topic = self.get_parameter('topic').get_parameter_value().string_value or 'initial_pose'
        self._frame_id = self.get_parameter('frame_id').get_parameter_value().string_value or 'map'
        self._position = (
            float(self.get_parameter('x').value),
            float(self.get_parameter('y').value),
            float(self.get_parameter('z').value),
        )
        self._orientation = (
            float(self.get_parameter('qx').value),
            float(self.get_parameter('qy').value),
            float(self.get_parameter('qz').value),
            float(self.get_parameter('qw').value),
        )
        delay = float(self.get_parameter('delay').value)

        self._publisher = self.create_publisher(PoseStamped, topic, 1)
        self._published = False
        # Publish once after the requested delay to give other nodes time to start.
        self._timer = self.create_timer(delay, self._publish_once)

    def _publish_once(self) -> None:
        if self._published:
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.pose.position.x = self._position[0]
        msg.pose.position.y = self._position[1]
        msg.pose.position.z = self._position[2]
        msg.pose.orientation.x = self._orientation[0]
        msg.pose.orientation.y = self._orientation[1]
        msg.pose.orientation.z = self._orientation[2]
        msg.pose.orientation.w = self._orientation[3]

        self._publisher.publish(msg)
        self._published = True
        self._timer.cancel()
        self.get_logger().info('Published initial pose', throttle_duration_sec=5.0)
        # Allow the message to flush, then shutdown this helper node.
        self.create_timer(0.2, self._shutdown)

    def _shutdown(self) -> None:
        self.get_logger().info('Initial pose publisher exiting')
        self.destroy_node()
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

