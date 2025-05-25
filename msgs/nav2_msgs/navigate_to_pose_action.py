from typing import Any, Protocol
import time
import math


class Publisher(Protocol):
    def send(self, message: dict) -> None:
        ...


def to_float(value: Any) -> float:
    try:
        return float(value)
    except (ValueError, TypeError):
        raise ValueError(f"Invalid float value: {value}")


class NavigateToPoseAction:
    def __init__(self, publisher: Publisher, action_name: str = "/navigate_to_pose"):
        self.publisher = publisher
        self.action_name = action_name

    def send_goal(self, x: float, y: float, z: float = 0.0, 
                  qx: float = 0.0, qy: float = 0.0, qz: float = 0.0, qw: float = 1.0,
                  frame_id: str = "map"):
        """
        发送navigate to pose action goal
        
        Args:
            x, y, z: 目标位置坐标
            qx, qy, qz, qw: 目标方向的四元数表示
            frame_id: 坐标系ID，默认为"map"
        """
        # 获取当前时间戳
        current_time = time.time()
        secs = int(current_time)
        nsecs = int((current_time - secs) * 1e9)

        # 构建action goal消息
        goal_msg = {
            "op": "send_action_goal",
            "action": self.action_name,
            "action_type": "nav2_msgs/action/NavigateToPose",
            "goal": {
                "pose": {
                    "header": {
                        "stamp": {
                            "sec": secs,
                            "nanosec": nsecs
                        },
                        "frame_id": frame_id
                    },
                    "pose": {
                        "position": {
                            "x": to_float(x),
                            "y": to_float(y),
                            "z": to_float(z)
                        },
                        "orientation": {
                            "x": to_float(qx),
                            "y": to_float(qy),
                            "z": to_float(qz),
                            "w": to_float(qw)
                        }
                    }
                }
            }
        }
        
        self.publisher.send(goal_msg)
        return goal_msg

    def send_goal_simple(self, x: float, y: float, yaw: float = 0.0, frame_id: str = "map"):
        """
        简化版本：使用yaw角度而不是四元数
        
        Args:
            x, y: 目标位置坐标
            yaw: 目标朝向角度（弧度）
            frame_id: 坐标系ID，默认为"map"
        """
        # 将yaw角度转换为四元数
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        
        return self.send_goal(x, y, 0.0, 0.0, 0.0, qz, qw, frame_id)

    def cancel_goal(self):
        """取消当前的导航目标"""
        cancel_msg = {
            "op": "cancel_action_goal",
            "action": self.action_name
        }
        
        self.publisher.send(cancel_msg)
        return cancel_msg 