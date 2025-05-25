from typing import Protocol, Optional, Any
import json


class Publisher(Protocol):
    def send(self, message: dict) -> None:
        ...


class ServiceCaller:
    def __init__(self, publisher: Publisher):
        self.publisher = publisher

    def call_service(self, service_name: str, service_type: str, request_data: dict = None):
        """
        调用ROS服务
        
        Args:
            service_name: 服务名称
            service_type: 服务类型
            request_data: 请求数据
        """
        if request_data is None:
            request_data = {}
            
        msg = {
            "op": "call_service",
            "service": service_name,
            "type": service_type,
            "args": request_data
        }
        
        self.publisher.send(msg)
        return msg

    def follow_waypoints(self):
        """
        调用Nav2的waypoint following服务
        """
        return self.call_service(
            service_name="/waypoint_follower/follow_waypoints",
            service_type="nav2_msgs/FollowWaypoints",
            request_data={}
        )

    def stop_waypoint_following(self):
        """
        停止waypoint following
        """
        return self.call_service(
            service_name="/waypoint_follower/stop_waypoint_following", 
            service_type="std_srvs/Empty",
            request_data={}
        )

    def pause_waypoint_following(self):
        """
        暂停waypoint following
        """
        return self.call_service(
            service_name="/waypoint_follower/pause_waypoint_following",
            service_type="std_srvs/Empty", 
            request_data={}
        )

    def resume_waypoint_following(self):
        """
        恢复waypoint following
        """
        return self.call_service(
            service_name="/waypoint_follower/resume_waypoint_following",
            service_type="std_srvs/Empty",
            request_data={}
        ) 