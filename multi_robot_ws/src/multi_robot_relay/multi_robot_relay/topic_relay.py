#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
import socket

class TopicRelay(Node):
    def __init__(self):
        super().__init__('topic_relay')

        # Get robot configuration from parameters
        self.declare_parameter('robot_id', 'robot1')
        self.declare_parameter('robot_ip', '192.168.1.100')

        self.robot_id = self.get_parameter('robot_id').value
        self.robot_ip = self.get_parameter('robot_ip').value
        self.namespace = f'/{self.robot_id}'

        self.get_logger().info(f'Starting relay for {self.robot_id} at {self.robot_ip}')

        # Publishers with namespace
        self.odom_pub = self.create_publisher(
            Odometry,
            f'{self.namespace}/odom',
            10
        )
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            f'{self.namespace}/map',
            10
        )

        # Subscribers to robot topics (via ROS2 domain bridge or network)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Goal subscriber with namespace (from React UI)
        self.goal_sub = self.create_subscription(
            PoseStamped,
            f'{self.namespace}/goal_pose',
            self.goal_callback,
            10
        )

        # Goal publisher to robot
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

    def odom_callback(self, msg):
        self.odom_pub.publish(msg)

    def map_callback(self, msg):
        self.map_pub.publish(msg)

    def goal_callback(self, msg):
        self.get_logger().info(f'Forwarding goal to {self.robot_id}')
        self.goal_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    relay = TopicRelay()
    rclpy.spin(relay)
    relay.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
