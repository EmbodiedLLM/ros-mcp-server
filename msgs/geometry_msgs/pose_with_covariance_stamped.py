from typing import Any, Protocol, Optional
import json
import math


class Publisher(Protocol):
    def send(self, message: dict) -> None:
        ...

    def subscribe_once(self, topic: str, timeout: float = 5.0) -> Optional[dict]:
        ...


class PoseWithCovarianceStamped:
    """处理geometry_msgs/PoseWithCovarianceStamped消息，通常用于AMCL定位结果"""
    
    def __init__(self, publisher: Publisher, topic: str = "/amcl_pose"):
        self.publisher = publisher
        self.topic = topic

    def subscribe(self, timeout: float = 5.0) -> Optional[dict]:
        """
        订阅位姿信息
        
        Args:
            timeout: 超时时间（秒）
            
        Returns:
            包含位置和协方差信息的字典，如果失败返回None
        """
        try:
            msg = self.publisher.subscribe_once(self.topic, timeout)
            if msg:
                return self._parse_pose_message(msg)
            return None
        except Exception as e:
            print(f"订阅位姿信息失败: {e}")
            return None

    def get_position(self, timeout: float = 5.0) -> Optional[dict]:
        """
        获取当前位置信息
        
        Args:
            timeout: 超时时间（秒）
            
        Returns:
            位置信息字典: {
                'x': float, 'y': float, 'z': float,
                'yaw': float,  # 朝向角度（弧度）
                'frame_id': str
            }
        """
        try:
            msg = self.publisher.subscribe_once(self.topic, timeout)
            if msg:
                return self._extract_position(msg)
            return None
        except Exception as e:
            print(f"获取位置信息失败: {e}")
            return None

    def _parse_pose_message(self, msg: dict) -> dict:
        """解析完整的位姿消息"""
        try:
            # 提取位置信息
            position = self._extract_position(msg)
            # 提取协方差信息
            covariance = self._extract_covariance(msg)
            
            # 提取时间戳
            header = msg.get('msg', {}).get('header', {})
            timestamp = header.get('stamp', {})
            
            return {
                'position': position,
                'covariance': covariance,
                'timestamp': timestamp,
                'frame_id': header.get('frame_id', 'unknown')
            }
        except Exception as e:
            print(f"解析位姿消息失败: {e}")
            return {}

    def _extract_position(self, msg: dict) -> Optional[dict]:
        """从消息中提取位置信息"""
        try:
            pose = msg.get('msg', {}).get('pose', {}).get('pose', {})
            position = pose.get('position', {})
            orientation = pose.get('orientation', {})
            
            # 提取位置坐标
            x = float(position.get('x', 0.0))
            y = float(position.get('y', 0.0))
            z = float(position.get('z', 0.0))
            
            # 将四元数转换为yaw角度
            qx = float(orientation.get('x', 0.0))
            qy = float(orientation.get('y', 0.0))
            qz = float(orientation.get('z', 0.0))
            qw = float(orientation.get('w', 1.0))
            
            # 计算yaw角度（绕z轴旋转）
            yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            
            # 获取frame_id
            frame_id = msg.get('msg', {}).get('header', {}).get('frame_id', 'map')
            
            return {
                'x': x,
                'y': y,
                'z': z,
                'yaw': yaw,
                'yaw_degrees': math.degrees(yaw),  # 同时提供度数
                'quaternion': {
                    'x': qx, 'y': qy, 'z': qz, 'w': qw
                },
                'frame_id': frame_id
            }
        except Exception as e:
            print(f"提取位置信息失败: {e}")
            return None

    def _extract_covariance(self, msg: dict) -> Optional[list]:
        """从消息中提取协方差信息"""
        try:
            covariance = msg.get('msg', {}).get('pose', {}).get('covariance', [])
            return list(covariance) if covariance else None
        except Exception as e:
            print(f"提取协方差信息失败: {e}")
            return None 