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
import math

import bispace_pb2
from google.protobuf import text_format

class DroneQuadtree(Node):
    def __init__(self):
        super().__init__('drone_quadtree')
        
        # 存储无人机位置数据
        self.drone_positions = {}
        
        # 创建订阅者列表
        self.odom_subs = []
        self.drone_names = ['cf231', 'cf232', 'cf233']  # 无人机名称列表
        
        # 为每架无人机创建订阅者
        for drone_name in self.drone_names:
            sub = self.create_subscription(
                Odometry,
                f'/{drone_name}/odom',
                lambda msg, name=drone_name: self.odom_callback(msg, name),
                10)
            self.odom_subs.append(sub)
            self.drone_positions[drone_name] = {'x': 0.0, 'y': 0.0}
        
        # 创建定时器，每秒发送一次数据
        self.timer = self.create_timer(1.0, self.send_positions)
        
        self.get_logger().info('Drone quadtree node initialized')
        
    def odom_callback(self, msg, drone_name):
        """处理里程计数据"""
        # 获取位置
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # 更新无人机位置
        self.drone_positions[drone_name] = {'x': x, 'y': y}
        
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

        # Parse pointsOmitted and pointsAdded
        points_omitted = payload.get('pointsOmitted', [])
        points_added = payload.get('pointsAdded', [])
        
        return grid, points_omitted, points_added

    @staticmethod
    def _point_in_cell(x, y, cell):
        """判断点(x,y)是否位于给定cell矩形范围内（以pose为中心，size为宽长）。"""
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
        """在BiGrid中查找包含点(x,y)的第一个单元格，返回cell或None。"""
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

    @staticmethod
    def _point_in_boundary(x, y, boundary):
        """判断点(x,y)是否处于 boundary 范围内。boundary: {x, y, width, height}."""
        try:
            bx = float(boundary["x"])  # 左下角x
            by = float(boundary["y"])  # 左下角y
            bw = float(boundary["width"])  # 宽
            bh = float(boundary["height"])  # 高
        except Exception:
            return False

        if bw <= 0.0 or bh <= 0.0:
            return False

        in_x = bx <= x <= (bx + bw)
        in_y = by <= y <= (by + bh)
        return in_x and in_y

    def _detect_collisions_by_omitted_and_boundary(self, points_omitted, boundary):
        """points_omitted 且位于 boundary 内即为发生冲突的无人机。"""
        collisions = []
        for omitted_point in points_omitted:
            x, y = float(omitted_point["x"]), float(omitted_point["y"])
            if not self._point_in_boundary(x, y, boundary):
                continue

            drone_name = self._find_drone_by_position(x, y)
            if drone_name is None:
                continue

            collisions.append({
                'drone': drone_name,
                'position': (x, y)
            })
        return collisions

    def _find_nearest_empty_cell(self, x, y, grid):
        """寻找距离 (x, y) 最近、且当前没有无人机处于该 cell 的 cell 中心作为避障坐标。若不存在返回 None。"""
        # 先构建一个集合，包含当前所有无人机所在的 cell id
        occupied_cell_ids = set()
        for _, pos in self.drone_positions.items():
            cell = self._find_cell_for_point(pos['x'], pos['y'], grid)
            if cell is not None and getattr(cell, 'id', None) is not None:
                occupied_cell_ids.add(cell.id)

        best_cell = None
        best_dist = float('inf')
        for cell in grid.items:
            # 跳过已被占用的 cell
            if getattr(cell, 'id', None) in occupied_cell_ids:
                continue

            cx = float(cell.pose.x)
            cy = float(cell.pose.y)
            dist = math.hypot(cx - x, cy - y)
            if dist < best_dist:
                best_dist = dist
                best_cell = cell

        if best_cell is None:
            return None
        return (float(best_cell.pose.x), float(best_cell.pose.y), getattr(best_cell, 'id', None))

    def send_positions(self):
        """Send drone positions to Java service"""
        # Prepare request data
        data = {
            "boundary": {
                "x": -2.0,
                "y": -2.0,
                "width": 4.0,
                "height": 4.0
            },
            "pointData": {
                "points": list(self.drone_positions.values())
            }
        }
        
        try:
            # Print current drone positions
            self.get_logger().info('Current drone positions:')
            for drone_name, pos in self.drone_positions.items():
                self.get_logger().info(f'{drone_name}: x={pos["x"]:.3f}, y={pos["y"]:.3f}')
            
            # Print data being sent
            self.get_logger().info('Sending data to Java service:')
            self.get_logger().info(json.dumps(data, indent=2))
            
            # Send POST request to Windows host
            response = requests.post(
                "http://172.29.224.1:8080/generate/quadtree?marginPoint=0.5&format=protobuf",
                headers={"Content-Type": "application/json"},
                data=json.dumps(data)
            )
            
            if response.status_code == 200:
                self.get_logger().info('Successfully sent positions to Java service')
                self.get_logger().info('Java service response:')
                self.get_logger().info(response.text)

                # Try to parse and locate drone cells
                try:
                    grid, points_omitted, points_added = self._decode_bigrid(response.text)
                    self.get_logger().info(f'BiGrid parsed successfully, cell count: {len(grid.items)}')
                    # Output BiGrid complete content (textproto format)
                    try:
                        grid_text = text_format.MessageToString(grid)
                        self.get_logger().info('BiGrid content:')
                        self.get_logger().info('\n' + grid_text)
                    except Exception as e:
                        self.get_logger().error(f'BiGrid content output failed: {e}')

                    # 基于新版服务检测碰撞：points_omitted 且位于 boundary
                    collisions = self._detect_collisions_by_omitted_and_boundary(points_omitted, data["boundary"])
                    if collisions:
                        self.get_logger().warn('Collsion risks detected!')
                        for c in collisions:
                            dx, dy = c['position']
                            self.get_logger().warn(
                                f'Drone {c["drone"]} position=({dx:.3f},{dy:.3f}) have risk of collision')

                            # 为该无人机寻找最近的空 cell 作为避障坐标
                            suggestion = self._find_nearest_empty_cell(dx, dy, grid)
                            if suggestion is not None:
                                sx, sy, sid = suggestion
                                self.get_logger().warn(
                                    f'Suggestion for avoiding collision -> cell id="{sid}", center=({sx:.3f},{sy:.3f})')
                            else:
                                self.get_logger().warn('No available empty cell found for avoiding collision')
                    else:
                        self.get_logger().warn('No collision risk detected')

                    for drone_name, pos in self.drone_positions.items():
                        cell = self._find_cell_for_point(pos['x'], pos['y'], grid)
                        if cell is not None:
                            self.get_logger().info(
                                f'{drone_name} position=({pos["x"]:.3f},{pos["y"]:.3f}) -> cell id="{cell.id}", center=({cell.pose.x:.3f},{cell.pose.y:.3f}), size=({cell.size.width:.3f},{cell.size.length:.3f})'
                            )
                        else:
                            self.get_logger().info(
                                f'{drone_name} position=({pos["x"]:.3f},{pos["y"]:.3f}) -> not within any cell bounds'
                            )
                    self.get_logger().info('================================')
                except Exception as e:
                    self.get_logger().error(f'Failed to parse Java service protobuf: {e}')
            else:
                self.get_logger().error(f'Failed to send positions: {response.status_code}')
                self.get_logger().error(f'Error response: {response.text}')
                self.get_logger().info('================================')
                
        except Exception as e:
            self.get_logger().error(f'Error sending positions: {str(e)}')

def main():
    rclpy.init()
    node = DroneQuadtree()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()