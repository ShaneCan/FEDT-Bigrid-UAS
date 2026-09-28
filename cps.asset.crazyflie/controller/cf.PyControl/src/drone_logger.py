#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import csv
import os
from datetime import datetime
import time

class DroneLogger(Node):
    def __init__(self):
        super().__init__('drone_logger')
        
        # Create the subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            '/cf231/odom',
            self.odom_callback,
            10)
        
        # Create the log directory
        self.log_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'drone_logs')
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
            
        # Create the log file
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_file = os.path.join(self.log_dir, f'drone_log_{timestamp}.csv')
        
        # Initialise the CSV file
        with open(self.log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'x', 'y', 'z', 'roll', 'pitch', 'yaw'])
            
        self.get_logger().info(f'Drone logger initialized. Logging to {self.log_file}')
        
    def quaternion_to_euler(self, x, y, z, w):
        """Converts a quaternion to Euler angles."""
        # Compute roll (rotation about the x axis)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Compute pitch (rotation about the y axis)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # use 90 degrees
        else:
            pitch = np.arcsin(sinp)
            
        # Compute yaw (rotation about the z axis)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
        
    def odom_callback(self, msg):
        """Handles odometry data."""
        # Get the position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        # Get the orientation as a quaternion
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Convert to Euler angles
        roll, pitch, yaw = self.quaternion_to_euler(qx, qy, qz, qw)
        
        # Get the timestamp
        timestamp = time.time()
        
        # Log the data
        print(f'Position: x={x:.3f}, y={y:.3f}, z={z:.3f}')
        print(f'Attitude: roll={np.degrees(roll):.3f}°, pitch={np.degrees(pitch):.3f}°, yaw={np.degrees(yaw):.3f}°')
        print('-' * 50)
        
        # Write to the CSV file
        with open(self.log_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, x, y, z, roll, pitch, yaw])

def main():
    rclpy.init()
    node = DroneLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()