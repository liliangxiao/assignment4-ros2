#!/usr/bin/env python3

# Modified from Gemini output

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__("obstacle_avoidance")

        # Configure QoS profile for publishing and subscribing
        # Quality of Service (qos) policies that allow you to tune communication between nodes.
        # QoS policies for subscriber must match the publisher
        # For more info, see: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # BEST_EFFORT: attempt to deliver samples, but may lose them if the network is not robust.
            durability=DurabilityPolicy.VOLATILE,  # VOLATILE: no attempt is made to persist samples.
            history=HistoryPolicy.KEEP_LAST,  # KEEP_LAST: only store up to N samples, configurable via the queue depth option.
            depth=10,  # a queue size of 10 to buffer messages if they arrive faster than they can be processed
        )

        self.scan_subscription = self.create_subscription(
            LaserScan,
            "scan",
            self.scan_callback,
            qos_profile=qos_profile,  # Replace with your lidar topic
        )
        self.velocity_publisher = self.create_publisher(
            Twist, "cmd_vel", 10  # Replace with your robot's velocity command topic
        )
        self.threshold_distance = 0.4  # Distance threshold for obstacle detection
        self.scan_range = math.pi / 2  # Scan range to check (90 degrees in front)
        self.turning = False
        self.turn_start_time = 0.0
        self.linear_speed = 0.2
        self.angular_speed = 0.2  # radians per second
        self.turn_duration = (
            math.pi / self.angular_speed
        )  # seconds to turn 180 degrees.

    def scan_callback(self, msg):
        if self.turning:
            self.turn_robot()
            return

        min_angle = -self.scan_range / 2
        max_angle = self.scan_range / 2

        ranges = []
        for i, angle in enumerate(
            (msg.angle_min + i * msg.angle_increment) for i in range(len(msg.ranges))
        ):
            if min_angle <= angle <= max_angle:
                ranges.append(msg.ranges[i])

        # self.get_logger().info(f"ranges length: {len(ranges)}")

        obstacle_detected = False
        for range_val in ranges:
            if (
                0.1 < range_val < self.threshold_distance
            ):  # filter out very close values, and far values.
                obstacle_detected = True
                break

        twist = Twist()
        if obstacle_detected:
            self.get_logger().info("Obstacle detected!")
            self.turning = True
            self.get_logger().info("Turning in progress!")
            self.turn_start_time = self.get_clock().now().nanoseconds / 1e9

        else:
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
            self.velocity_publisher.publish(twist)

    def turn_robot(self):
        twist = Twist()
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_time = current_time - self.turn_start_time

        # print(f"Elapsed Time: {elapsed_time} - Turn duration: {self.turn_duration}")

        if elapsed_time < self.turn_duration:
            twist.angular.z = self.angular_speed
            self.velocity_publisher.publish(twist)
        else:
            self.turning = False
            self.get_logger().info("Turning Complete")
            twist.angular.z = 0.0
            self.velocity_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance = ObstacleAvoidance()
    rclpy.spin(obstacle_avoidance)
    obstacle_avoidance.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
