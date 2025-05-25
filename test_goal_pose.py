#!/usr/bin/env python3

from utils.websocket_manager import WebSocketManager
from msgs.geometry_msgs import PoseStamped

# 配置
ROSBRIDGE_IP = "10.90.0.101"
ROSBRIDGE_PORT = 9090
LOCAL_IP = "10.90.0.101"

def test_goal_pose():
    # 创建WebSocket管理器
    ws_manager = WebSocketManager(ROSBRIDGE_IP, ROSBRIDGE_PORT, LOCAL_IP)
    
    # 创建PoseStamped发布器
    goal_pose = PoseStamped(ws_manager, topic="/goal_pose")
    
    # 发送目标点
    print("发送目标点到 /goal_pose...")
    msg = goal_pose.publish_simple(1.0, 1.0, 0.0, "map")
    
    if msg:
        print("目标点发送成功!")
        print(f"消息内容: {msg}")
    else:
        print("目标点发送失败!")
    
    # 关闭连接
    ws_manager.close()

if __name__ == "__main__":
    test_goal_pose() 