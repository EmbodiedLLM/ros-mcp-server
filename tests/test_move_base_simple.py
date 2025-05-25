#!/usr/bin/env python3

from utils.websocket_manager import WebSocketManager
import time

# 配置
ROSBRIDGE_IP = "10.90.0.101"
ROSBRIDGE_PORT = 9090
LOCAL_IP = "10.90.0.101"

def test_move_base_simple():
    ws_manager = WebSocketManager(ROSBRIDGE_IP, ROSBRIDGE_PORT, LOCAL_IP)
    
    # 构建move_base_simple/goal消息
    current_time = time.time()
    secs = int(current_time)
    nsecs = int((current_time - secs) * 1e9)
    
    goal_msg = {
        "op": "publish",
        "topic": "/move_base_simple/goal",
        "msg": {
            "header": {
                "stamp": {
                    "sec": secs,
                    "nanosec": nsecs
                },
                "frame_id": "map"
            },
            "pose": {
                "position": {
                    "x": 1.0,
                    "y": 0.0,
                    "z": 0.0
                },
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                    "w": 1.0
                }
            }
        }
    }
    
    print("发送目标到 /move_base_simple/goal...")
    ws_manager.send(goal_msg)
    print("目标发送完成!")
    print(f"消息内容: {goal_msg}")
    
    ws_manager.close()

if __name__ == "__main__":
    test_move_base_simple() 