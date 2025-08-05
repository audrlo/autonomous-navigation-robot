#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Twist
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import math

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        
        # Parameters
        self.declare_parameter('min_distance', 0.6)  # 2 feet in meters
        self.declare_parameter('max_linear_velocity', 0.5)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('safety_factor', 1.2)
        self.declare_parameter('use_lidar', False)  # Use RealSense depth by default
        
        self.min_distance = self.get_parameter('min_distance').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.safety_factor = self.get_parameter('safety_factor').value
        self.use_lidar = self.get_parameter('use_lidar').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_out', 10)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel_in', self.cmd_vel_callback, 10)
        
        if self.use_lidar:
            self.scan_sub = self.create_subscription(
                LaserScan, 'scan', self.laser_callback, 10)
        else:
            # Subscribe to RealSense depth point cloud
            self.pointcloud_sub = self.create_subscription(
                PointCloud2, '/camera/camera/depth/color/points', 
                self.pointcloud_callback, 10)
        
        # State variables
        self.last_cmd_vel = Twist()
        self.obstacle_detected = False
        self.obstacle_direction = 0.0  # -1 = left, 0 = center, 1 = right
        
        self.get_logger().info('Obstacle avoidance node started')
        self.get_logger().info(f'Using {"LiDAR" if self.use_lidar else "RealSense depth"} for obstacle detection')
    
    def cmd_vel_callback(self, msg):
        """Process incoming velocity commands and apply obstacle avoidance."""
        self.last_cmd_vel = msg
        
        if self.obstacle_detected:
            # Apply obstacle avoidance
            modified_cmd = self.apply_obstacle_avoidance(msg)
            self.cmd_vel_pub.publish(modified_cmd)
        else:
            # No obstacles, pass through the command
            self.cmd_vel_pub.publish(msg)
    
    def laser_callback(self, msg):
        """Process laser scan data for obstacle detection."""
        if len(msg.ranges) == 0:
            return
        
        # Check for obstacles in front arc (±45 degrees)
        angle_increment = msg.angle_increment
        total_points = len(msg.ranges)
        center_index = total_points // 2
        
        # Calculate indices for ±45 degrees
        quarter_points = total_points // 8  # 45 degrees
        start_index = max(0, center_index - quarter_points)
        end_index = min(total_points, center_index + quarter_points)
        
        # Find minimum distance in front arc
        front_ranges = msg.ranges[start_index:end_index]
        valid_ranges = [r for r in front_ranges if msg.range_min <= r <= msg.range_max]
        
        if valid_ranges:
            min_distance = min(valid_ranges)
            min_index = front_ranges.index(min_distance) + start_index
            
            # Determine obstacle direction
            if min_index < center_index - quarter_points // 2:
                self.obstacle_direction = -1.0  # Left
            elif min_index > center_index + quarter_points // 2:
                self.obstacle_direction = 1.0   # Right
            else:
                self.obstacle_direction = 0.0   # Center
            
            self.obstacle_detected = min_distance < (self.min_distance * self.safety_factor)
        else:
            self.obstacle_detected = False
    
    def pointcloud_callback(self, msg):
        """Process RealSense depth point cloud for obstacle detection."""
        try:
            # Convert point cloud to numpy array
            points = []
            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append([point[0], point[1], point[2]])
            
            if len(points) == 0:
                self.obstacle_detected = False
                return
            
            points = np.array(points)
            
            # Filter points in front of robot (positive x direction)
            # and within reasonable height range
            front_mask = (points[:, 0] > 0) & (points[:, 0] < 3.0)  # 3m range
            height_mask = (points[:, 2] > -0.3) & (points[:, 2] < 1.5)  # Ground to head height
            width_mask = abs(points[:, 1]) < 1.0  # ±1m width
            
            valid_points = points[front_mask & height_mask & width_mask]
            
            if len(valid_points) == 0:
                self.obstacle_detected = False
                return
            
            # Find closest point
            distances = np.sqrt(valid_points[:, 0]**2 + valid_points[:, 1]**2)
            min_distance = np.min(distances)
            closest_point_idx = np.argmin(distances)
            closest_point = valid_points[closest_point_idx]
            
            # Determine obstacle direction based on y-coordinate
            if closest_point[1] < -0.2:
                self.obstacle_direction = -1.0  # Left
            elif closest_point[1] > 0.2:
                self.obstacle_direction = 1.0   # Right
            else:
                self.obstacle_direction = 0.0   # Center
            
            self.obstacle_detected = min_distance < (self.min_distance * self.safety_factor)
            
            if self.obstacle_detected:
                self.get_logger().debug(
                    f'Obstacle detected at {min_distance:.2f}m, direction: {self.obstacle_direction}')
                
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')
            self.obstacle_detected = False
    
    def apply_obstacle_avoidance(self, cmd_vel):
        """Apply obstacle avoidance behavior."""
        modified_cmd = Twist()
        
        if cmd_vel.linear.x > 0:  # Moving forward
            if self.obstacle_direction == 0.0:  # Obstacle dead ahead
                # Stop forward motion and turn
                modified_cmd.linear.x = 0.0
                modified_cmd.angular.z = self.max_angular_vel * (1.0 if np.random.random() > 0.5 else -1.0)
            else:
                # Slow down and turn away from obstacle
                modified_cmd.linear.x = cmd_vel.linear.x * 0.3  # Reduce speed
                # Turn away from obstacle
                turn_direction = -1.0 if self.obstacle_direction > 0 else 1.0
                modified_cmd.angular.z = self.max_angular_vel * 0.7 * turn_direction
        else:
            # Backing up or turning - allow the motion
            modified_cmd = cmd_vel
        
        # Clamp velocities
        modified_cmd.linear.x = max(-self.max_linear_vel, 
                                   min(self.max_linear_vel, modified_cmd.linear.x))
        modified_cmd.angular.z = max(-self.max_angular_vel, 
                                    min(self.max_angular_vel, modified_cmd.angular.z))
        
        return modified_cmd

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()