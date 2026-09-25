#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import requests
import json
from datetime import datetime
import socket
import base64

import bispace_pb2
from google.protobuf import text_format

class DroneVertiport(Node):
    def __init__(self):
        super().__init__('drone_vertiport')
        
        # 存储无人机位置和高度数据
        self.drone_positions = {}
        self.drone_heights = {}
        
        # 创建订阅者列表
        self.odom_subs = []
        self.drone_names = ['cf231', 'cf232', 'cf233', 'cf234', 'cf235']  # 无人机名称列表
        
        # 为每架无人机创建订阅者
        for drone_name in self.drone_names:
            sub = self.create_subscription(
                Odometry,
                f'/{drone_name}/odom',
                lambda msg, name=drone_name: self.odom_callback(msg, name),
                10)
            self.odom_subs.append(sub)
            self.drone_positions[drone_name] = {'x': 0.0, 'y': 0.0}
            self.drone_heights[drone_name] = 0.0
        
        # 创建定时器，每秒发送一次数据
        self.timer = self.create_timer(1.0, self.get_bigrid_and_locate_drones)
        
        self.get_logger().info('Drone vertiport node initialized')
        
    def odom_callback(self, msg, drone_name):
        """Process odometry data"""
        # Get position and height
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        # Update drone position and height
        self.drone_positions[drone_name] = {'x': x, 'y': y}
        self.drone_heights[drone_name] = z
        
    def _decode_bigrid(self, response_text):
        """Parse BiGrid protobuf from Java service JSON response"""
        try:
            payload = json.loads(response_text)
        except Exception as e:
            raise ValueError(f'Failed to parse JSON response: {e}')

        mime_type = payload.get('mimeType')
        if mime_type != 'application/x-protobuf':
            raise ValueError(f'Unsupported mimeType: {mime_type}')

        content_b64 = payload.get('content')
        if not content_b64:
            raise ValueError('Response missing content field')

        try:
            content_bytes = base64.b64decode(content_b64)
        except Exception as e:
            raise ValueError(f'Base64 decode failed: {e}')

        grid = bispace_pb2.BiGrid()
        try:
            grid.ParseFromString(content_bytes)
        except Exception as e:
            raise ValueError(f'Protobuf parse failed: {e}')

        return grid

    @staticmethod
    def _point_in_cell(x, y, cell):
        """Check if point (x,y) is within the given cell rectangle bounds (pose as center, size as width/length)."""
        try:
            cx = float(cell.pose.x)
            cy = float(cell.pose.y)
            width = float(cell.size.width)
            length = float(cell.size.length)
        except Exception:
            return False

        if width <= 0.0 or length <= 0.0:
            return False

        half_w = width / 2.0
        half_l = length / 2.0

        in_x = (cx - half_w) <= x <= (cx + half_w)
        in_y = (cy - half_l) <= y <= (cy + half_l)
        return in_x and in_y

    def _find_cell_for_point(self, x, y, grid):
        """Find the first cell in BiGrid that contains point (x,y), returns cell or None."""
        for cell in grid.items:
            if self._point_in_cell(x, y, cell):
                return cell
        return None

    def _find_drone_by_position(self, x, y, tolerance=0.01):
        """Find drone name by coordinates"""
        for drone_name, pos in self.drone_positions.items():
            if abs(pos['x'] - x) < tolerance and abs(pos['y'] - y) < tolerance:
                return drone_name
        return None

    def _detect_collisions(self, grid):
        """Detect if multiple drones are in the same cell and determine which one should leave"""
        cell_occupancy = {}
        collisions = []
        
        # Check which drones are in which cells
        for drone_name, pos in self.drone_positions.items():
            cell = self._find_cell_for_point(pos['x'], pos['y'], grid)
            if cell is not None:
                cell_id = cell.id
                if cell_id not in cell_occupancy:
                    cell_occupancy[cell_id] = []
                cell_occupancy[cell_id].append(drone_name)
        
        # Check for cells with multiple drones
        for cell_id, drones in cell_occupancy.items():
            if len(drones) > 1:
                # Find the drone with highest altitude (should leave)
                highest_drone = max(drones, key=lambda drone: self.drone_heights[drone])
                collisions.append({
                    'cell_id': cell_id,
                    'drones': drones,
                    'highest_drone': highest_drone,
                    'highest_altitude': self.drone_heights[highest_drone]
                })
        
        return collisions

    def get_bigrid_and_locate_drones(self):
        """Get BiGrid and locate drones in cells"""
        try:
            # Print current drone positions and heights
            self.get_logger().info('Current drone positions and heights:')
            for drone_name, pos in self.drone_positions.items():
                height = self.drone_heights[drone_name]
                self.get_logger().info(f'{drone_name}: x={pos["x"]:.3f}, y={pos["y"]:.3f}, z={height:.3f}')
            
            # Send POST request to Windows host to get BiGrid
            bigrid_data = {
                "x": 0.0,
                "y": -2.0,
                "stepSizeX": 1.0,
                "stepSizeY": 1.0
            }
            
            self.get_logger().info('Requesting BiGrid from Java service:')
            self.get_logger().info(json.dumps(bigrid_data, indent=2))
            
            response = requests.post(
                "http://172.29.224.1:8080/generate/bigrid?rows=1&cols=5&format=protobuf",
                headers={"Content-Type": "application/json"},
                data=json.dumps(bigrid_data)
            )
            
            if response.status_code == 200:
                self.get_logger().info('Successfully received BiGrid from Java service')
                self.get_logger().info('Java service response:')
                self.get_logger().info(response.text)

                # 尝试解析并定位无人机所在单元格
                try:
                    grid = self._decode_bigrid(response.text)
                    self.get_logger().info(f'BiGrid parsed successfully, cell count: {len(grid.items)}')
                    
                    # Output BiGrid complete content (textproto format)
                    try:
                        grid_text = text_format.MessageToString(grid)
                        self.get_logger().info('BiGrid content:')
                        self.get_logger().info('\n' + grid_text)
                    except Exception as e:
                        self.get_logger().error(f'BiGrid content output failed: {e}')

                    # Detect collision situations
                    collisions = self._detect_collisions(grid)
                    if collisions:
                        self.get_logger().warn('COLLISION RISK DETECTED! Multiple drones in same vertiport cell!')
                        for collision in collisions:
                            self.get_logger().warn(
                                f'Cell {collision["cell_id"]} contains multiple drones: {collision["drones"]}'
                            )
                            self.get_logger().warn(
                                f'ALERT: {collision["highest_drone"]} (altitude: {collision["highest_altitude"]:.3f}m) - GO TO OTHER VERTIPORT!'
                            )
                            self.get_logger().warn(
                                f'Reason: {collision["highest_drone"]} has the highest altitude and should leave this cell'
                            )
                    else:
                        self.get_logger().info('No collision risk detected - vertiport cells are safe')

                    # Locate each drone in its cell
                    for drone_name, pos in self.drone_positions.items():
                        cell = self._find_cell_for_point(pos['x'], pos['y'], grid)
                        height = self.drone_heights[drone_name]
                        if cell is not None:
                            self.get_logger().info(
                                f'{drone_name} position=({pos["x"]:.3f},{pos["y"]:.3f},z={height:.3f}) -> cell id="{cell.id}", center=({cell.pose.x:.3f},{cell.pose.y:.3f}), size=({cell.size.width:.3f},{cell.size.length:.3f})'
                            )
                        else:
                            self.get_logger().info(
                                f'{drone_name} position=({pos["x"]:.3f},{pos["y"]:.3f},z={height:.3f}) -> not within any cell bounds'
                            )
                except Exception as e:
                    self.get_logger().error(f'Failed to parse Java service protobuf: {e}')
            else:
                self.get_logger().error(f'Failed to get BiGrid: {response.status_code}')
                self.get_logger().error(f'Error response: {response.text}')
                
        except Exception as e:
            self.get_logger().error(f'Error getting BiGrid: {str(e)}')

def main():
    rclpy.init()
    node = DroneVertiport()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
