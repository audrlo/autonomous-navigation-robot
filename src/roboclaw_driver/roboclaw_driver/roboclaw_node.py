#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import serial
import struct
import math
import time
from threading import Lock

class RoboClawNode(Node):
    def __init__(self):
        super().__init__('roboclaw_node')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 38400)
        self.declare_parameter('address', 0x80)
        self.declare_parameter('wheel_separation', 0.5)  # meters
        self.declare_parameter('wheel_radius', 0.1)      # meters
        self.declare_parameter('max_speed', 1.0)         # m/s
        self.declare_parameter('ticks_per_meter', 1000)  # encoder ticks per meter
        
        self.port = self.get_parameter('port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.address = self.get_parameter('address').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_speed = self.get_parameter('max_speed').value
        self.ticks_per_meter = self.get_parameter('ticks_per_meter').value
        
        # Initialize serial connection
        self.serial_lock = Lock()
        try:
            self.serial_conn = serial.Serial(self.port, self.baud_rate, timeout=0.1)
            self.get_logger().info(f'Connected to RoboClaw on {self.port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to RoboClaw: {e}')
            self.serial_conn = None
        
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.left_encoder_prev = 0
        self.right_encoder_prev = 0
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timers
        self.create_timer(0.05, self.update_odometry)  # 20 Hz
        
        self.get_logger().info('RoboClaw driver node started')
    
    def cmd_vel_callback(self, msg):
        """Convert twist command to motor speeds and send to RoboClaw."""
        if not self.serial_conn:
            return
            
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        # Differential drive kinematics
        left_vel = linear_vel - (angular_vel * self.wheel_separation / 2.0)
        right_vel = linear_vel + (angular_vel * self.wheel_separation / 2.0)
        
        # Convert to motor commands (-127 to 127)
        max_motor_speed = 127
        left_motor = int(max_motor_speed * left_vel / self.max_speed)
        right_motor = int(max_motor_speed * right_vel / self.max_speed)
        
        # Clamp values
        left_motor = max(-127, min(127, left_motor))
        right_motor = max(-127, min(127, right_motor))
        
        # Send commands to RoboClaw
        self.drive_motors(left_motor, right_motor)
    
    def drive_motors(self, left_speed, right_speed):
        """Send motor speed commands to RoboClaw."""
        with self.serial_lock:
            try:
                # Command 9: Drive Both Motors with Signed Speed
                if left_speed >= 0:
                    left_cmd = 0  # Forward
                    left_val = left_speed
                else:
                    left_cmd = 1  # Backward  
                    left_val = -left_speed
                    
                if right_speed >= 0:
                    right_cmd = 0  # Forward
                    right_val = right_speed
                else:
                    right_cmd = 1  # Backward
                    right_val = -right_speed
                
                # Send drive command
                cmd = [self.address, 9, left_val, right_val]
                checksum = sum(cmd) & 0x7F
                cmd.append(checksum)
                
                self.serial_conn.write(bytes(cmd))
                self.serial_conn.flush()
                
            except Exception as e:
                self.get_logger().error(f'Error sending motor command: {e}')
    
    def read_encoders(self):
        """Read encoder values from RoboClaw."""
        if not self.serial_conn:
            return None, None
            
        with self.serial_lock:
            try:
                # Command 16: Read Encoder 1
                cmd = [self.address, 16]
                checksum = sum(cmd) & 0x7F
                cmd.append(checksum)
                
                self.serial_conn.write(bytes(cmd))
                response = self.serial_conn.read(6)
                
                if len(response) == 6:
                    left_encoder = struct.unpack('>I', response[:4])[0]
                    
                    # Command 17: Read Encoder 2
                    cmd = [self.address, 17]
                    checksum = sum(cmd) & 0x7F
                    cmd.append(checksum)
                    
                    self.serial_conn.write(bytes(cmd))
                    response = self.serial_conn.read(6)
                    
                    if len(response) == 6:
                        right_encoder = struct.unpack('>I', response[:4])[0]
                        return left_encoder, right_encoder
                        
            except Exception as e:
                self.get_logger().error(f'Error reading encoders: {e}')
                
        return None, None
    
    def update_odometry(self):
        """Update robot odometry based on encoder readings."""
        left_encoder, right_encoder = self.read_encoders()
        
        if left_encoder is None or right_encoder is None:
            return
        
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # Calculate distance traveled by each wheel
        left_diff = left_encoder - self.left_encoder_prev
        right_diff = right_encoder - self.right_encoder_prev
        
        left_distance = left_diff / self.ticks_per_meter
        right_distance = right_diff / self.ticks_per_meter
        
        # Update previous encoder values
        self.left_encoder_prev = left_encoder
        self.right_encoder_prev = right_encoder
        
        # Calculate robot motion
        distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_separation
        
        # Update robot pose
        delta_x = distance * math.cos(self.theta + delta_theta / 2.0)
        delta_y = distance * math.sin(self.theta + delta_theta / 2.0)
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Calculate velocities
        linear_vel = distance / dt if dt > 0 else 0.0
        angular_vel = delta_theta / dt if dt > 0 else 0.0
        
        # Publish odometry
        self.publish_odometry(current_time, linear_vel, angular_vel)
        
        # Publish joint states
        self.publish_joint_states(current_time, left_distance/dt, right_distance/dt)
        
        self.last_time = current_time
    
    def publish_odometry(self, current_time, linear_vel, angular_vel):
        """Publish odometry message."""
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from yaw)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Velocity
        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = angular_vel
        
        self.odom_pub.publish(odom)
        
        # Publish TF transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)
    
    def publish_joint_states(self, current_time, left_vel, right_vel):
        """Publish joint states for the wheels."""
        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.velocity = [left_vel / self.wheel_radius, right_vel / self.wheel_radius]
        
        self.joint_state_pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = RoboClawNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()