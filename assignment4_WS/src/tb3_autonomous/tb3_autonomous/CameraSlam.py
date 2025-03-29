#!/usr/bin/env python3

import rclpy
import cv2
import math
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory 
   
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class IntegratedRobotController(Node):
    def __init__(self):
        super().__init__("integrated_robot_controller")
        
        # Initialize movement parameters and state machine
        self.state = "move_forward"
        self.start_time = self.get_clock().now()
        self.duration_move = 2.0  # Move forward duration
        self.duration_turn = math.pi/2 / 0.2  # 90 degree turn time
        self.linear_speed = 0.2
        self.angular_speed = 0.2
        
        # Obstacle avoidance parameters
        self.obstacle_detected = False
        self.turning_start_time = 0.0
        self.turn_duration = math.pi / self.angular_speed  # 180 degrees
        self.threshold_distance = 0.4
        self.scan_range = math.pi/2  # 90 degree frontal scan
        
        # Initialize CV components
        self.bridge = CvBridge()
        self.templates = self.load_templates()
        self.current_arrow = None
        
        # Configure QoS profiles
        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Set up publishers/subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(LaserScan, "/scan", self.scan_callback, lidar_qos)
        self.create_subscription(Image, "/camera/color/image_raw", self.image_callback, lidar_qos)
        self.create_timer(0.1, self.state_machine_update)


    def load_templates(self):
        """Load arrow templates for detection"""
        templates = {}
        # Get the directory of the current script
        script_dir = os.path.dirname(os.path.abspath(__file__))
        
        for i in range(4):
            templates = {}
            package_name = 'tb3_autonomous'  # Define package_name here!
            #template_path = os.path.join('lib/python3.10/site-packages/tb3_autonomous', f'output{i}.jpg')
            package_share_directory = get_package_share_directory(package_name)
            template_path = os.path.join(package_share_directory, f'output{i}.jpg')
            try:
                templates[f'arguco{i+1}'] = cv2.imread(template_path, 0)
            except Exception as e:
                self.get_logger().error(f"Failed to load template: {str(e)}")
        return templates


    def scan_callback(self, msg):
        """Process LiDAR data for obstacle detection"""
        if self.obstacle_detected:
            return  # Already handling obstacle
            
        min_angle = -self.scan_range/2
        max_angle = self.scan_range/2
        
        # Analyze scan ranges
        ranges = [msg.ranges[i] for i, angle in enumerate(
            (msg.angle_min + i * msg.angle_increment) for i in range(len(msg.ranges))
        ) if min_angle <= angle <= max_angle]
        
        # Check for obstacles
        self.obstacle_detected = any(
            0.1 < r < self.threshold_distance for r in ranges if not math.isnan(r)
        )
        
        if self.obstacle_detected:
            self.get_logger().info("OBSTACLE DETECTED! Initiating avoidance maneuver")
            self.turning_start_time = self.get_clock().now().nanoseconds / 1e9

    def image_callback(self, msg):
        """Process camera images for arrow detection"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            processed = self.detect_arrows(cv_image)
            cv2.imshow("Camera Feed", processed)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Image processing error: {str(e)}")

    def detect_arrows(self, frame):
        """Arrow detection processing pipeline"""
        # Preprocessing
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5,5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        
        # Contour detection and processing
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            if cv2.contourArea(cnt) < 500:
                continue
                
            approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
            if len(approx) == 4:
                # Perspective transform and template matching
                warped = self.perspective_transform(gray, approx.reshape(4,2))
                self.current_arrow = self.template_match(warped)
                
                if self.current_arrow:
                    cv2.drawContours(frame, [approx], -1, (0,255,0), 2)
                    cv2.putText(frame, self.current_arrow, 
                               tuple(approx[0][0]), cv2.FONT_HERSHEY_SIMPLEX, 
                               0.8, (0,0,255), 2)
        return frame

    def state_machine_update(self):
        """Main control logic for movement states"""
        twist = Twist()
        
        # Priority 1: Obstacle avoidance
        if self.obstacle_detected:
            current_time = self.get_clock().now().nanoseconds / 1e9
            elapsed = current_time - self.turning_start_time
            
            if elapsed < self.turn_duration:
                twist.angular.z = self.angular_speed
            else:
                self.obstacle_detected = False
                self.get_logger().info("Obstacle avoidance complete")
            
            self.cmd_vel_pub.publish(twist)
            return
        
        # Priority 2: Normal square movement
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
        
        if self.state == "move_forward" and elapsed >= self.duration_move:
            self.state = "turn"
            self.start_time = self.get_clock().now()
        elif self.state == "turn" and elapsed >= self.duration_turn:
            self.state = "move_forward"
            self.start_time = self.get_clock().now()
        
        # Set movement commands
        if self.state == "move_forward":
            twist.linear.x = self.linear_speed
        else:
            twist.angular.z = -self.angular_speed  # Clockwise turn
        
        self.cmd_vel_pub.publish(twist)

    # Helper functions for arrow detection
    def perspective_transform(self, img, pts):
        """Warp perspective to match template"""
        rect = self.order_points(pts)
        (tl, tr, br, bl) = rect
        
        width = max(int(np.linalg.norm(br - bl)), int(np.linalg.norm(tr - tl)))
        height = max(int(np.linalg.norm(tr - br)), int(np.linalg.norm(tl - bl)))
        
        dst = np.array([[0,0], [width-1,0], [width-1,height-1], [0,height-1]], dtype="float32")
        M = cv2.getPerspectiveTransform(rect, dst)
        return cv2.warpPerspective(img, M, (width, height))

    def template_match(self, warped_img):
        """Match template against known arrows"""
        for label, template in self.templates.items():
            resized = cv2.resize(warped_img, (template.shape[1], template.shape[0]))
            res = cv2.matchTemplate(resized, template, cv2.TM_CCOEFF_NORMED)
            if np.max(res) > 0.6:
                return label
        return None

    @staticmethod
    def order_points(pts):
        """Order points for perspective transform"""
        rect = np.zeros((4,2), dtype="float32")
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]
        diff = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]
        return rect

def main(args=None):
    rclpy.init(args=args)
    controller = IntegratedRobotController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
