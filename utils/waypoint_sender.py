from typing import List, Any, Protocol, Optional
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


class UniversalWaypointSender:
    """通用的waypoint发送器，支持多种Nav2实现方式"""
    
    def __init__(self, publisher: Publisher):
        self.publisher = publisher

    def send_waypoints_via_action(self, waypoints: List[List[float]], 
                                 action_name: str = "/follow_waypoints",
                                 frame_id: str = "map"):
        """
        通过action发送waypoints（推荐方式）
        """
        # 获取当前时间戳
        current_time = time.time()
        secs = int(current_time)
        nsecs = int((current_time - secs) * 1e9)

        poses = []
        for wp in waypoints:
            if len(wp) >= 2:
                x = to_float(wp[0])
                y = to_float(wp[1])
                yaw = to_float(wp[2] if len(wp) > 2 else 0.0)
                
                # 将yaw角度转换为四元数
                qz = math.sin(yaw / 2.0)
                qw = math.cos(yaw / 2.0)
                
                pose = {
                    "header": {
                        "stamp": {
                            "sec": secs,
                            "nanosec": nsecs
                        },
                        "frame_id": frame_id
                    },
                    "pose": {
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
                }
                poses.append(pose)

        # 构建action goal消息
        goal_msg = {
            "op": "send_action_goal",
            "action": action_name,
            "action_type": "nav2_msgs/FollowWaypoints",
            "goal": {
                "poses": poses
            }
        }
        
        self.publisher.send(goal_msg)
        return goal_msg

    def send_waypoints_via_topic(self, waypoints: List[List[float]], 
                                topic_name: str = "/waypoints",
                                frame_id: str = "map"):
        """
        通过话题发送waypoints（兼容某些实现）
        """
        # 获取当前时间戳
        current_time = time.time()
        secs = int(current_time)
        nsecs = int((current_time - secs) * 1e9)

        poses = []
        for wp in waypoints:
            if len(wp) >= 2:
                x = to_float(wp[0])
                y = to_float(wp[1])
                yaw = to_float(wp[2] if len(wp) > 2 else 0.0)
                
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

        # 发送PoseArray消息
        msg = {
            "op": "publish",
            "topic": topic_name,
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

    def send_sequential_goals(self, waypoints: List[List[float]], 
                             goal_topic: str = "/goal_pose",
                             frame_id: str = "map",
                             delay: float = 0.5):
        """
        逐个发送单独的导航目标（备用方案）
        """
        results = []
        
        for i, wp in enumerate(waypoints):
            if len(wp) >= 2:
                x = to_float(wp[0])
                y = to_float(wp[1])
                yaw = to_float(wp[2] if len(wp) > 2 else 0.0)
                
                # 将yaw角度转换为四元数
                qz = math.sin(yaw / 2.0)
                qw = math.cos(yaw / 2.0)
                
                # 获取当前时间戳
                current_time = time.time()
                secs = int(current_time)
                nsecs = int((current_time - secs) * 1e9)
                
                msg = {
                    "op": "publish",
                    "topic": goal_topic,
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
                    }
                }
                
                self.publisher.send(msg)
                results.append(msg)
                
                # 如果不是最后一个航点，添加延迟
                if i < len(waypoints) - 1:
                    time.sleep(delay)
        
        return results

    def cancel_waypoint_following(self, action_name: str = "/follow_waypoints"):
        """取消waypoint following"""
        cancel_msg = {
            "op": "cancel_action_goal",
            "action": action_name
        }
        
        self.publisher.send(cancel_msg)
        return cancel_msg 