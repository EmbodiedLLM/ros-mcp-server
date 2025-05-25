#!/usr/bin/env python3

from utils.websocket_manager import WebSocketManager
from msgs.nav_msgs import Odometry
from msgs.geometry_msgs import PoseWithCovarianceStamped
import json

# 配置
LOCAL_IP = "10.90.0.101"
ROSBRIDGE_IP = "10.90.0.101"
ROSBRIDGE_PORT = 9090

def test_parsing():
    """测试消息解析功能"""
    ws_manager = WebSocketManager(ROSBRIDGE_IP, ROSBRIDGE_PORT, LOCAL_IP)
    
    print("=== 测试消息解析功能 ===\n")
    
    # 测试AMCL位置解析
    print("1. 测试AMCL位置解析:")
    amcl_pose = PoseWithCovarianceStamped(ws_manager, topic="/pose")
    try:
        # 直接获取原始消息
        raw_msg = ws_manager.subscribe_once("/pose", timeout=5.0)
        if raw_msg:
            print("原始消息接收成功")
            # 测试解析
            position = amcl_pose._extract_position(raw_msg)
            if position:
                print("✓ AMCL位置解析成功:")
                print(f"  位置: ({position['x']:.3f}, {position['y']:.3f}, {position['z']:.3f})")
                print(f"  朝向: {position['yaw']:.3f} 弧度 ({position['yaw_degrees']:.1f}°)")
                print(f"  坐标系: {position['frame_id']}")
            else:
                print("✗ AMCL位置解析失败")
        else:
            print("✗ 未接收到AMCL消息")
    except Exception as e:
        print(f"✗ AMCL测试异常: {e}")
    
    print("\n" + "="*50 + "\n")
    
    # 测试里程计位置解析
    print("2. 测试里程计位置解析:")
    odometry = Odometry(ws_manager, topic="/Odometry")
    try:
        # 直接获取原始消息
        raw_msg = ws_manager.subscribe_once("/Odometry", timeout=5.0)
        if raw_msg:
            print("原始消息接收成功")
            # 测试解析
            position = odometry._extract_position(raw_msg)
            if position:
                print("✓ 里程计位置解析成功:")
                print(f"  位置: ({position['x']:.3f}, {position['y']:.3f}, {position['z']:.3f})")
                print(f"  朝向: {position['yaw']:.3f} 弧度 ({position['yaw_degrees']:.1f}°)")
                print(f"  坐标系: {position['frame_id']}")
            else:
                print("✗ 里程计位置解析失败")
                
            # 测试速度解析
            velocity = odometry._extract_velocity(raw_msg)
            if velocity:
                print("✓ 里程计速度解析成功:")
                linear = velocity['linear']
                angular = velocity['angular']
                print(f"  线速度: ({linear['x']:.3f}, {linear['y']:.3f}, {linear['z']:.3f}) m/s")
                print(f"  角速度: ({angular['x']:.3f}, {angular['y']:.3f}, {angular['z']:.3f}) rad/s")
            else:
                print("✗ 里程计速度解析失败")
        else:
            print("✗ 未接收到里程计消息")
    except Exception as e:
        print(f"✗ 里程计测试异常: {e}")
    
    print("\n" + "="*50 + "\n")
    
    # 测试高级接口
    print("3. 测试高级接口:")
    try:
        print("测试AMCL get_position():")
        amcl_position = amcl_pose.get_position(timeout=3.0)
        if amcl_position:
            print("✓ AMCL get_position() 成功:")
            print(f"  位置: ({amcl_position['x']:.3f}, {amcl_position['y']:.3f}, {amcl_position['z']:.3f})")
            print(f"  朝向: {amcl_position['yaw']:.3f} 弧度 ({amcl_position['yaw_degrees']:.1f}°)")
        else:
            print("✗ AMCL get_position() 失败")
    except Exception as e:
        print(f"✗ AMCL get_position() 异常: {e}")
    
    try:
        print("\n测试里程计 get_position():")
        odom_position = odometry.get_position(timeout=3.0)
        if odom_position:
            print("✓ 里程计 get_position() 成功:")
            print(f"  位置: ({odom_position['x']:.3f}, {odom_position['y']:.3f}, {odom_position['z']:.3f})")
            print(f"  朝向: {odom_position['yaw']:.3f} 弧度 ({odom_position['yaw_degrees']:.1f}°)")
        else:
            print("✗ 里程计 get_position() 失败")
    except Exception as e:
        print(f"✗ 里程计 get_position() 异常: {e}")
    
    ws_manager.close()

if __name__ == "__main__":
    test_parsing() 