#!/usr/bin/env python3
"""
测试机器人位置获取功能
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from utils.websocket_manager import WebSocketManager
from msgs.nav_msgs import Odometry
from msgs.geometry_msgs import PoseWithCovarianceStamped

# 配置
LOCAL_IP = "10.90.0.101"
ROSBRIDGE_IP = "10.90.0.101"
ROSBRIDGE_PORT = 9090

def test_direct_subscription():
    """直接测试话题订阅"""
    ws_manager = WebSocketManager(ROSBRIDGE_IP, ROSBRIDGE_PORT, LOCAL_IP)
    
    print("测试直接订阅 /odom 话题...")
    try:
        result = ws_manager.subscribe_once("/odom", timeout=5.0)
        if result:
            print("成功接收到 /odom 消息:")
            print(f"消息类型: {result.get('op')}")
            print(f"话题: {result.get('topic')}")
            print(f"消息内容: {result.get('msg', {}).keys()}")
        else:
            print("未接收到 /odom 消息")
    except Exception as e:
        print(f"/odom 订阅失败: {e}")
    
    print("\n测试直接订阅 /pose 话题...")
    try:
        result = ws_manager.subscribe_once("/pose", timeout=5.0)
        if result:
            print("成功接收到 /pose 消息:")
            print(f"消息类型: {result.get('op')}")
            print(f"话题: {result.get('topic')}")
            print(f"消息内容: {result.get('msg', {}).keys()}")
        else:
            print("未接收到 /pose 消息")
    except Exception as e:
        print(f"/pose 订阅失败: {e}")
    
    ws_manager.close()

def test_odometry_class():
    """测试Odometry类"""
    ws_manager = WebSocketManager(ROSBRIDGE_IP, ROSBRIDGE_PORT, LOCAL_IP)
    odometry = Odometry(ws_manager, topic="/odom")
    
    print("\n测试Odometry类的位置获取...")
    try:
        position = odometry.get_position(timeout=5.0)
        if position:
            print("成功获取位置信息:")
            print(f"位置: x={position.get('x')}, y={position.get('y')}, z={position.get('z')}")
            print(f"朝向: yaw={position.get('yaw')} 弧度 ({position.get('yaw_degrees')} 度)")
            print(f"坐标系: {position.get('frame_id')}")
        else:
            print("未能获取位置信息")
    except Exception as e:
        print(f"位置获取失败: {e}")
    
    ws_manager.close()

def test_topics():
    """测试话题可用性"""
    print("=== 检查相关话题 ===\n")
    
    ws_manager = WebSocketManager(ROSBRIDGE_IP, ROSBRIDGE_PORT, LOCAL_IP)
    
    try:
        topics = ws_manager.get_topics()
        if topics:
            print("可用话题:")
            position_topics = []
            for topic, msg_type in topics:
                if any(keyword in topic.lower() for keyword in ['odom', 'pose', 'amcl']):
                    position_topics.append((topic, msg_type))
                    print(f"  {topic} ({msg_type})")
            
            if not position_topics:
                print("  未找到位置相关话题")
        else:
            print("无法获取话题列表")
    except Exception as e:
        print(f"获取话题列表失败: {e}")
    
    ws_manager.close()
    print()

if __name__ == "__main__":
    print("开始测试位置获取功能...")
    test_direct_subscription()
    test_odometry_class()
    print("测试完成")

    # 首先检查话题
    test_topics() 