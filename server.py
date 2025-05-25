from mcp.server.fastmcp import FastMCP
from typing import List, Any, Optional
from pathlib import Path
import json
from utils.websocket_manager import WebSocketManager
from utils.service_caller import ServiceCaller

from msgs.geometry_msgs import Twist, PoseWithCovarianceStamped
from msgs.sensor_msgs import Image, JointState
from msgs.nav_msgs import Odometry
from msgs.nav2_msgs.navigate_to_pose_action import NavigateToPoseAction
from msgs.nav2_msgs.follow_waypoints_action import FollowWaypointsAction

LOCAL_IP = "10.90.0.101"
ROSBRIDGE_IP = "10.90.0.101"
ROSBRIDGE_PORT = 9090

mcp = FastMCP("ros-mcp-server")
ws_manager = WebSocketManager(ROSBRIDGE_IP, ROSBRIDGE_PORT, LOCAL_IP)
twist = Twist(ws_manager, topic="/cmd_vel")
image = Image(ws_manager, topic="/camera/image_raw")
jointstate = JointState(ws_manager, topic="/joint_states")
service_caller = ServiceCaller(ws_manager)

# Action clients for Nav2 (推荐方式)
navigate_action = NavigateToPoseAction(ws_manager, "/navigate_to_pose")
follow_waypoints_action = FollowWaypointsAction(ws_manager, "/follow_waypoints")

# 位置获取相关实例
odometry = Odometry(ws_manager, topic="/Odometry")  # 使用实际可用的里程计话题
amcl_pose = PoseWithCovarianceStamped(ws_manager, topic="/pose")  # 使用实际可用的话题

@mcp.tool()
def get_topics():
    topic_info = ws_manager.get_topics()

    if topic_info:
        topics, types = zip(*topic_info)
        result = {
            "topics": list(topics),
            "types": list(types)
        }
        ws_manager.close()
        return result
    else:
        ws_manager.close()
        return "No topics found"

@mcp.tool()
def pub_twist(linear: List[Any], angular: List[Any]):
    msg = twist.publish(linear, angular)
    ws_manager.close()
    
    if msg is not None:
        return "Twist message published successfully"
    else:
        return "No message published"

@mcp.tool()
def pub_twist_seq(linear: List[Any], angular: List[Any], duration: List[Any]):
    twist.publish_sequence(linear, angular, duration)

@mcp.tool()
def sub_image():
    msg = image.subscribe()
    ws_manager.close()
    
    if msg is not None:
        return "Image data received and downloaded successfully"
    else:
        return "No image data received"

@mcp.tool()
def pub_jointstate(name: list[str], position: list[float], velocity: list[float], effort: list[float]):
    msg = jointstate.publish(name, position, velocity, effort)
    ws_manager.close()
    if msg is not None:
        return "JointState message published successfully"
    else:
        return "No message published"

@mcp.tool()
def sub_jointstate():
    msg = jointstate.subscribe()
    ws_manager.close()
    if msg is not None:
        return msg
    else:
        return "No JointState data received"

@mcp.tool()
def send_nav2_goal_action(x: float, y: float, yaw: float = 0.0, frame_id: str = "map"):
    """
    通过action发送导航目标点给nav2（推荐方式）
    
    Args:
        x: 目标点x坐标
        y: 目标点y坐标  
        yaw: 目标朝向角度（弧度），默认为0
        frame_id: 坐标系ID，默认为"map"
    """
    msg = navigate_action.send_goal_simple(x, y, yaw, frame_id)
    ws_manager.close()
    
    if msg is not None:
        return f"Nav2 goal sent via action successfully to ({x}, {y}) with yaw {yaw} in frame {frame_id}"
    else:
        return "Failed to send nav2 goal via action"

@mcp.tool()
def send_nav2_goal(x: float, y: float, yaw: float = 0.0, frame_id: str = "map"):
    """
    发送导航目标点给nav2
    
    Args:
        x: 目标点x坐标
        y: 目标点y坐标  
        yaw: 目标朝向角度（弧度），默认为0
        frame_id: 坐标系ID，默认为"map"
    """
    # 现在使用action方式发送
    msg = navigate_action.send_goal_simple(x, y, yaw, frame_id)
    ws_manager.close()
    
    if msg is not None:
        return f"Nav2 goal sent successfully to ({x}, {y}) with yaw {yaw} in frame {frame_id}"
    else:
        return "Failed to send nav2 goal"

@mcp.tool()
def send_nav2_goal_detailed(x: float, y: float, z: float = 0.0,
                           qx: float = 0.0, qy: float = 0.0, qz: float = 0.0, qw: float = 1.0,
                           frame_id: str = "map"):
    """
    发送详细的导航目标点给nav2（包含完整的位置和四元数方向）
    
    Args:
        x, y, z: 目标位置坐标
        qx, qy, qz, qw: 目标方向的四元数表示
        frame_id: 坐标系ID，默认为"map"
    """
    # 现在使用action方式发送
    msg = navigate_action.send_goal(x, y, z, qx, qy, qz, qw, frame_id)
    ws_manager.close()
    
    if msg is not None:
        return f"Detailed nav2 goal sent successfully to ({x}, {y}, {z}) with quaternion ({qx}, {qy}, {qz}, {qw}) in frame {frame_id}"
    else:
        return "Failed to send detailed nav2 goal"

@mcp.tool()
def send_waypoints_action_direct(waypoint_list: List[List[float]], frame_id: str = "map"):
    """
    直接通过action发送多个航点给Nav2（新的推荐方式）
    
    Args:
        waypoint_list: 航点列表，格式为 [[x1, y1, yaw1], [x2, y2, yaw2], ...]
        frame_id: 坐标系ID，默认为"map"
    """
    msg = follow_waypoints_action.send_goal_simple(waypoint_list, frame_id)
    ws_manager.close()
    
    if msg is not None:
        return f"Waypoints sent via action directly: {len(waypoint_list)} waypoints in frame {frame_id}"
    else:
        return "Failed to send waypoints via action directly"

@mcp.tool()
def send_waypoints(waypoint_list: List[List[float]], frame_id: str = "map"):
    """
    发送多个航点给Nav2 waypoint follower
    
    Args:
        waypoint_list: 航点列表，格式为 [[x1, y1, yaw1], [x2, y2, yaw2], ...]
        frame_id: 坐标系ID，默认为"map"
    """
    # 现在使用action方式发送
    msg = follow_waypoints_action.send_goal_simple(waypoint_list, frame_id)
    ws_manager.close()
    
    if msg is not None:
        return f"Waypoints sent successfully: {len(waypoint_list)} waypoints in frame {frame_id}"
    else:
        return "Failed to send waypoints"

@mcp.tool()
def start_waypoint_following():
    """
    开始执行waypoint following（相当于RViz2中的"Start"按钮）
    """
    msg = service_caller.follow_waypoints()
    ws_manager.close()
    
    if msg is not None:
        return "Waypoint following started successfully"
    else:
        return "Failed to start waypoint following"

@mcp.tool()
def stop_waypoint_following():
    """
    停止waypoint following
    """
    msg = service_caller.stop_waypoint_following()
    ws_manager.close()
    
    if msg is not None:
        return "Waypoint following stopped successfully"
    else:
        return "Failed to stop waypoint following"

@mcp.tool()
def pause_waypoint_following():
    """
    暂停waypoint following
    """
    msg = service_caller.pause_waypoint_following()
    ws_manager.close()
    
    if msg is not None:
        return "Waypoint following paused successfully"
    else:
        return "Failed to pause waypoint following"

@mcp.tool()
def resume_waypoint_following():
    """
    恢复waypoint following
    """
    msg = service_caller.resume_waypoint_following()
    ws_manager.close()
    
    if msg is not None:
        return "Waypoint following resumed successfully"
    else:
        return "Failed to resume waypoint following"

@mcp.tool()
def send_and_start_waypoints(waypoint_list: List[List[float]], frame_id: str = "map"):
    """
    发送航点并立即开始执行（组合功能）
    
    Args:
        waypoint_list: 航点列表，格式为 [[x1, y1, yaw1], [x2, y2, yaw2], ...]
        frame_id: 坐标系ID，默认为"map"
    """
    # 现在直接使用action方式发送，无需额外启动
    waypoints_msg = follow_waypoints_action.send_goal_simple(waypoint_list, frame_id)
    ws_manager.close()
    
    if waypoints_msg is not None:
        return f"Waypoints sent and started via action: {len(waypoint_list)} waypoints in frame {frame_id}"
    else:
        return "Failed to send waypoints via action"









@mcp.tool()
def cancel_waypoint_following():
    """
    取消当前的waypoint following
    """
    msg = follow_waypoints_action.cancel_goal()
    ws_manager.close()
    
    if msg is not None:
        return "Waypoint following cancelled successfully"
    else:
        return "Failed to cancel waypoint following"

@mcp.tool()
def get_robot_position(source: str = "auto", timeout: float = 5.0):
    """
    获取机器人当前位置
    
    Args:
        source: 位置数据源，可选值：
               - "auto": 自动选择最佳数据源（默认）
               - "odom": 使用里程计数据（/Odometry话题）
               - "amcl": 使用AMCL定位数据（/pose话题）
        timeout: 超时时间（秒），默认5.0秒
    
    Returns:
        位置信息字典，包含x, y, z坐标和yaw角度
    """
    # 为每次调用创建新的WebSocket管理器
    local_ws_manager = WebSocketManager(ROSBRIDGE_IP, ROSBRIDGE_PORT, LOCAL_IP)
    
    if source == "auto":
        # 首先尝试AMCL定位数据（更准确）
        try:
            local_amcl_pose = PoseWithCovarianceStamped(local_ws_manager, topic="/pose")
            position = local_amcl_pose.get_position(timeout=timeout)
            if position:
                local_ws_manager.close()
                return {
                    "source": "amcl",
                    "position": position,
                    "status": "success"
                }
        except Exception as e:
            print(f"AMCL位置获取失败: {e}")
        
        # 如果AMCL失败，尝试里程计数据
        try:
            local_odometry = Odometry(local_ws_manager, topic="/Odometry")
            position = local_odometry.get_position(timeout=timeout)
            if position:
                local_ws_manager.close()
                return {
                    "source": "odometry", 
                    "position": position,
                    "status": "success"
                }
        except Exception as e:
            print(f"里程计位置获取失败: {e}")
            
        local_ws_manager.close()
        return {
            "source": "none",
            "position": None,
            "status": "failed",
            "error": "All position sources failed"
        }
    
    elif source == "amcl":
        try:
            local_amcl_pose = PoseWithCovarianceStamped(local_ws_manager, topic="/pose")
            position = local_amcl_pose.get_position(timeout=timeout)
            local_ws_manager.close()
            
            if position:
                return {
                    "source": "amcl",
                    "position": position,
                    "status": "success"
                }
            else:
                return {
                    "source": "amcl",
                    "position": None,
                    "status": "failed",
                    "error": "Failed to get AMCL position"
                }
        except Exception as e:
            local_ws_manager.close()
            return {
                "source": "amcl",
                "position": None,
                "status": "failed",
                "error": f"AMCL position error: {e}"
            }
    
    elif source == "odom":
        try:
            local_odometry = Odometry(local_ws_manager, topic="/Odometry")
            position = local_odometry.get_position(timeout=timeout)
            local_ws_manager.close()
            
            if position:
                return {
                    "source": "odometry",
                    "position": position,
                    "status": "success"
                }
            else:
                return {
                    "source": "odometry", 
                    "position": None,
                    "status": "failed",
                    "error": "Failed to get odometry position"
                }
        except Exception as e:
            local_ws_manager.close()
            return {
                "source": "odometry",
                "position": None,
                "status": "failed",
                "error": f"Odometry position error: {e}"
            }
    
    else:
        local_ws_manager.close()
        return {
            "source": "invalid",
            "position": None,
            "status": "failed",
            "error": f"Invalid source: {source}. Use 'auto', 'amcl', or 'odom'"
        }

@mcp.tool()
def get_robot_velocity(timeout: float = 5.0):
    """
    获取机器人当前速度
    
    Args:
        timeout: 超时时间（秒），默认5.0秒
    
    Returns:
        速度信息字典，包含线速度和角速度
    """
    local_ws_manager = WebSocketManager(ROSBRIDGE_IP, ROSBRIDGE_PORT, LOCAL_IP)
    
    try:
        local_odometry = Odometry(local_ws_manager, topic="/Odometry")
        velocity = local_odometry.get_velocity(timeout=timeout)
        local_ws_manager.close()
        
        if velocity:
            return {
                "velocity": velocity,
                "status": "success"
            }
        else:
            return {
                "velocity": None,
                "status": "failed",
                "error": "Failed to get robot velocity"
            }
    except Exception as e:
        local_ws_manager.close()
        return {
            "velocity": None,
            "status": "failed",
            "error": f"Velocity error: {e}"
        }

@mcp.tool()
def get_robot_odometry(timeout: float = 5.0):
    """
    获取完整的机器人里程计信息（位置+速度）
    
    Args:
        timeout: 超时时间（秒），默认5.0秒
    
    Returns:
        完整里程计信息字典
    """
    local_ws_manager = WebSocketManager(ROSBRIDGE_IP, ROSBRIDGE_PORT, LOCAL_IP)
    
    try:
        local_odometry = Odometry(local_ws_manager, topic="/Odometry")
        odom_data = local_odometry.subscribe(timeout=timeout)
        local_ws_manager.close()
        
        if odom_data:
            return {
                "odometry": odom_data,
                "status": "success"
            }
        else:
            return {
                "odometry": None,
                "status": "failed", 
                "error": "Failed to get robot odometry"
            }
    except Exception as e:
        local_ws_manager.close()
        return {
            "odometry": None,
            "status": "failed",
            "error": f"Odometry error: {e}"
        }

@mcp.tool()
def get_robot_amcl_pose(timeout: float = 5.0):
    """
    获取AMCL定位结果（包含协方差信息）
    
    Args:
        timeout: 超时时间（秒），默认5.0秒
    
    Returns:
        AMCL位姿信息字典，包含位置和协方差
    """
    local_ws_manager = WebSocketManager(ROSBRIDGE_IP, ROSBRIDGE_PORT, LOCAL_IP)
    
    try:
        local_amcl_pose = PoseWithCovarianceStamped(local_ws_manager, topic="/pose")
        amcl_data = local_amcl_pose.subscribe(timeout=timeout)
        local_ws_manager.close()
        
        if amcl_data:
            return {
                "amcl_pose": amcl_data,
                "status": "success"
            }
        else:
            return {
                "amcl_pose": None,
                "status": "failed",
                "error": "Failed to get AMCL pose"
            }
    except Exception as e:
        local_ws_manager.close()
        return {
            "amcl_pose": None,
            "status": "failed",
            "error": f"AMCL pose error: {e}"
        }

if __name__ == "__main__":
    mcp.run(transport="stdio")
