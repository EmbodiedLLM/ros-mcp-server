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


class PoseStamped:
    def __init__(self, publisher: Publisher, topic: str = "/goal_pose"):
        self.publisher = publisher
        self.topic = topic

    def publish(self, x: float, y: float, z: float = 0.0, 
                qx: float = 0.0, qy: float = 0.0, qz: float = 0.0, qw: float = 1.0,
                frame_id: str = "map"):
        """
        发布PoseStamped消息到nav2
        
        Args:
            x, y, z: 位置坐标
            qx, qy, qz, qw: 四元数表示的方向
            frame_id: 坐标系ID，默认为"map"
        """
        # 转换为浮点数
        x_f = to_float(x)
        y_f = to_float(y)
        z_f = to_float(z)
        qx_f = to_float(qx)
        qy_f = to_float(qy)
        qz_f = to_float(qz)
        qw_f = to_float(qw)

        # 获取当前时间戳
        current_time = time.time()
        secs = int(current_time)
        nsecs = int((current_time - secs) * 1e9)

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
                "pose": {
                    "position": {
                        "x": x_f,
                        "y": y_f,
                        "z": z_f
                    },
                    "orientation": {
                        "x": qx_f,
                        "y": qy_f,
                        "z": qz_f,
                        "w": qw_f
                    }
                }
            }
        }
        
        self.publisher.send(msg)
        return msg

    def publish_simple(self, x: float, y: float, yaw: float = 0.0, frame_id: str = "map"):
        """
        简化版本：只需要x, y坐标和yaw角度
        
        Args:
            x, y: 位置坐标
            yaw: 偏航角（弧度）
            frame_id: 坐标系ID，默认为"map"
        """
        import math
        
        # 将yaw角度转换为四元数
        yaw_f = to_float(yaw)
        qz = math.sin(yaw_f / 2.0)
        qw = math.cos(yaw_f / 2.0)
        
        return self.publish(x, y, 0.0, 0.0, 0.0, qz, qw, frame_id) 