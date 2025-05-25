from typing import List, Any, Protocol, Optional
import time


class Publisher(Protocol):
    def send(self, message: dict) -> None:
        ...


def to_float(value: Any) -> float:
    try:
        return float(value)
    except (ValueError, TypeError):
        raise ValueError(f"Invalid float value: {value}")


class PoseArray:
    def __init__(self, publisher: Publisher, topic: str = "/waypoint_follower/waypoints"):
        self.publisher = publisher
        self.topic = topic

    def publish(self, waypoints: List[dict], frame_id: str = "map"):
        """
        发布PoseArray消息到Nav2 waypoint follower
        
        Args:
            waypoints: 航点列表，每个航点包含 {'x': float, 'y': float, 'yaw': float}
            frame_id: 坐标系ID，默认为"map"
        """
        import math
        
        # 获取当前时间戳
        current_time = time.time()
        secs = int(current_time)
        nsecs = int((current_time - secs) * 1e9)

        poses = []
        for waypoint in waypoints:
            x = to_float(waypoint.get('x', 0.0))
            y = to_float(waypoint.get('y', 0.0))
            yaw = to_float(waypoint.get('yaw', 0.0))
            
            # 将yaw角度转换为四元数
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            
            pose = {
                "position": {
                    "x": x,
                    "y": y,
                    "z": 0.0
                },
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": qz,
                    "w": qw
                }
            }
            poses.append(pose)

        msg = {
            "op": "publish",
            "topic": self.topic,
            "msg": {
                "header": {
                    "stamp": {
                        "sec": secs,
                        "nanosec": nsecs
                    },
                    "frame_id": frame_id
                },
                "poses": poses
            }
        }
        
        self.publisher.send(msg)
        return msg

    def publish_simple(self, waypoints: List[List[float]], frame_id: str = "map"):
        """
        简化版本：接受 [[x1, y1, yaw1], [x2, y2, yaw2], ...] 格式
        
        Args:
            waypoints: 航点列表，格式为 [[x, y, yaw], ...]
            frame_id: 坐标系ID，默认为"map"
        """
        waypoint_dicts = []
        for wp in waypoints:
            if len(wp) >= 2:
                waypoint_dict = {
                    'x': wp[0],
                    'y': wp[1],
                    'yaw': wp[2] if len(wp) > 2 else 0.0
                }
                waypoint_dicts.append(waypoint_dict)
        
        return self.publish(waypoint_dicts, frame_id) 