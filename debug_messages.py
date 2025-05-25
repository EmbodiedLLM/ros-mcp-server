#!/usr/bin/env python3

from utils.websocket_manager import WebSocketManager
import json

# 配置
LOCAL_IP = "10.90.0.101"
ROSBRIDGE_IP = "10.90.0.101"
ROSBRIDGE_PORT = 9090

def debug_message_structure():
    """调试消息结构"""
    ws_manager = WebSocketManager(ROSBRIDGE_IP, ROSBRIDGE_PORT, LOCAL_IP)
    
    print("=== 调试消息结构 ===\n")
    
    # 测试 /pose 话题
    print("1. 调试 /pose 话题消息结构:")
    try:
        result = ws_manager.subscribe_once("/pose", timeout=5.0)
        if result:
            print("成功接收到 /pose 消息:")
            print(json.dumps(result, indent=2))
        else:
            print("未接收到 /pose 消息")
    except Exception as e:
        print(f"/pose 订阅失败: {e}")
    
    print("\n" + "="*50 + "\n")
    
    # 测试 /Odometry 话题
    print("2. 调试 /Odometry 话题消息结构:")
    try:
        result = ws_manager.subscribe_once("/Odometry", timeout=5.0)
        if result:
            print("成功接收到 /Odometry 消息:")
            print(json.dumps(result, indent=2))
        else:
            print("未接收到 /Odometry 消息")
    except Exception as e:
        print(f"/Odometry 订阅失败: {e}")
    
    ws_manager.close()

if __name__ == "__main__":
    debug_message_structure() 