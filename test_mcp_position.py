#!/usr/bin/env python3
"""
测试MCP位置获取功能
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# 导入server中的函数
from server import get_robot_position, get_robot_velocity, get_robot_odometry

def test_mcp_position_functions():
    """测试MCP位置获取功能"""
    
    print("=== 测试MCP位置获取功能 ===\n")
    
    print("1. 测试自动位置获取...")
    try:
        result = get_robot_position()
        print(f"结果: {result}")
    except Exception as e:
        print(f"异常: {e}")
    
    print("\n2. 测试里程计位置获取...")
    try:
        result = get_robot_position(source="odom")
        print(f"结果: {result}")
    except Exception as e:
        print(f"异常: {e}")
    
    print("\n3. 测试AMCL位置获取...")
    try:
        result = get_robot_position(source="amcl")
        print(f"结果: {result}")
    except Exception as e:
        print(f"异常: {e}")
    
    print("\n4. 测试速度获取...")
    try:
        result = get_robot_velocity()
        print(f"结果: {result}")
    except Exception as e:
        print(f"异常: {e}")
    
    print("\n5. 测试完整里程计信息获取...")
    try:
        result = get_robot_odometry()
        print(f"结果: {result}")
    except Exception as e:
        print(f"异常: {e}")
    
    print("\n=== 测试完成 ===")

if __name__ == "__main__":
    test_mcp_position_functions() 