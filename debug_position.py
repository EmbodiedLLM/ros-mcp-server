#!/usr/bin/env python3
"""
调试位置获取功能
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from utils.websocket_manager import WebSocketManager
from msgs.nav_msgs import Odometry
from msgs.geometry_msgs import PoseWithCovarianceStamped
import json

def debug_position_topics():
    """调试位置相关话题"""
    
    # 连接配置
    ROSBRIDGE_IP = "10.90.0.101"
    ROSBRIDGE_PORT = 9090
    LOCAL_IP = "10.90.0.101"
    
    print("=== 调试位置获取功能 ===\n")
    
    # 创建WebSocket管理器
    ws_manager = WebSocketManager(ROSBRIDGE_IP, ROSBRIDGE_PORT, LOCAL_IP)
    
    # 测试不同的里程计话题
    odom_topics = ["/odom", "/Odometry"]
    pose_topics = ["/pose", "/initialpose", "/amcl_pose"]
    
    print("1. 测试里程计话题...")
    for topic in odom_topics:
        print(f"\n测试话题: {topic}")
        try:
            odometry = Odometry(ws_manager, topic=topic)
            
            # 尝试订阅一次消息
            print(f"  尝试订阅 {topic}...")
            msg = ws_manager.subscribe_once(topic, timeout=2.0)
            
            if msg:
                print(f"  ✓ 成功接收到消息")
                print(f"  消息类型: {msg.get('op', 'unknown')}")
                
                # 尝试解析位置
                try:
                    position = odometry._extract_position(msg)
                    if position:
                        print(f"  ✓ 位置解析成功:")
                        print(f"    位置: ({position['x']:.3f}, {position['y']:.3f}, {position['z']:.3f})")
                        print(f"    朝向: {position['yaw']:.3f} 弧度 ({position['yaw_degrees']:.1f}°)")
                        print(f"    坐标系: {position['frame_id']}")
                    else:
                        print(f"  ✗ 位置解析失败")
                except Exception as e:
                    print(f"  ✗ 位置解析异常: {e}")
                
                # 尝试解析速度
                try:
                    velocity = odometry._extract_velocity(msg)
                    if velocity:
                        print(f"  ✓ 速度解析成功:")
                        linear = velocity['linear']
                        angular = velocity['angular']
                        print(f"    线速度: ({linear['x']:.3f}, {linear['y']:.3f}, {linear['z']:.3f}) m/s")
                        print(f"    角速度: ({angular['x']:.3f}, {angular['y']:.3f}, {angular['z']:.3f}) rad/s")
                    else:
                        print(f"  ✗ 速度解析失败")
                except Exception as e:
                    print(f"  ✗ 速度解析异常: {e}")
                    
            else:
                print(f"  ✗ 未接收到消息 (超时)")
                
        except Exception as e:
            print(f"  ✗ 话题测试异常: {e}")
    
    print("\n2. 测试位姿话题...")
    for topic in pose_topics:
        print(f"\n测试话题: {topic}")
        try:
            pose = PoseWithCovarianceStamped(ws_manager, topic=topic)
            
            # 尝试订阅一次消息
            print(f"  尝试订阅 {topic}...")
            msg = ws_manager.subscribe_once(topic, timeout=2.0)
            
            if msg:
                print(f"  ✓ 成功接收到消息")
                print(f"  消息类型: {msg.get('op', 'unknown')}")
                
                # 尝试解析位置
                try:
                    position = pose._extract_position(msg)
                    if position:
                        print(f"  ✓ 位置解析成功:")
                        print(f"    位置: ({position['x']:.3f}, {position['y']:.3f}, {position['z']:.3f})")
                        print(f"    朝向: {position['yaw']:.3f} 弧度 ({position['yaw_degrees']:.1f}°)")
                        print(f"    坐标系: {position['frame_id']}")
                    else:
                        print(f"  ✗ 位置解析失败")
                except Exception as e:
                    print(f"  ✗ 位置解析异常: {e}")
                    
            else:
                print(f"  ✗ 未接收到消息 (超时)")
                
        except Exception as e:
            print(f"  ✗ 话题测试异常: {e}")
    
    print("\n3. 原始消息检查...")
    # 检查原始消息结构
    for topic in ["/odom", "/pose"]:
        print(f"\n检查话题 {topic} 的原始消息结构:")
        try:
            msg = ws_manager.subscribe_once(topic, timeout=2.0)
            if msg:
                print(f"  原始消息结构:")
                print(f"  {json.dumps(msg, indent=2)[:500]}...")  # 只显示前500字符
            else:
                print(f"  未接收到消息")
        except Exception as e:
            print(f"  检查异常: {e}")
    
    # 关闭连接
    ws_manager.close()
    print("\n=== 调试完成 ===")

if __name__ == "__main__":
    debug_position_topics() 