from rclpy.node import Node
from rclpy.time import Time, Duration

from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.msg import PoseStampedArray
import threading

from typing import Dict, List, Optional
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy


class CfPositionBuffer:
    def __init__(self, node: Node, decay_time: float = 1.0):
        """_summary_

        Args:
            node (Node): A node to attach a decay_time timer to.
            decay_time (float, optional): Time a position stays valid. Defaults to 1.0.
        """
        self.node = node
        self.decay_time = decay_time

        self.positions: Dict[str, PoseStamped] = (
            {}
        )  # Would it be good to maintain the parent id? Is it always world?
        self.positions_lock: threading.Lock = threading.Lock()

    def set_position(self, pose: PoseStamped, parent_frame_id: str):
        with self.positions_lock:
            self.positions[pose.header.frame_id] = pose

    def get_position(self, frame: str) -> Optional[PoseStamped]:
        with self.positions_lock:
            if frame in self.positions.keys():
                now = self.node.get_clock().now()
                decay_duration = Duration(
                    seconds=int(self.decay_time),
                    nanoseconds=int((self.decay_time % 1) * 1e9),
                )
                try:  # Fails if time is use_sim_time and we just started.
                    decay_time = now - decay_duration
                except:
                    return None

                pose: PoseStamped = self.positions[frame]
                time = Time.from_msg(pose.header.stamp)

                if time < decay_time:
                    del self.positions[frame]
                    return None

                return self.positions[frame]
        return None

    def get_all_positions(self) -> List[PoseStamped]:
        poses = []
        for frame in list(self.positions.keys()):
            pose = self.get_position(frame)
            if pose is not None:
                poses.append(pose)
        return poses


class CfPositionListener:
    def __init__(self, buffer: CfPositionBuffer, node: Node):
        self.buffer = buffer
        self.node = node

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.cf_position_sub = node.create_subscription(
            msg_type=PoseStampedArray,
            topic="/cf_positions",
            callback=self.callback,
            qos_profile=qos,
        )

    def __del__(self) -> None:
        self.node.destroy_subscription(self.cf_position_sub)

    def callback(self, msg: PoseStampedArray):
        pose_stamped: PoseStamped
        for pose_stamped in msg.poses:
            self.buffer.set_position(pose_stamped, msg.header.frame_id)
