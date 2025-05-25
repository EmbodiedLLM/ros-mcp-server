#!/usr/bin/env python3

from utils.websocket_manager import WebSocketManager
from msgs.nav_msgs import Odometry
from msgs.geometry_msgs import PoseWithCovarianceStamped

# 配置
LOCAL_IP = "10.90.0.101"
ROSBRIDGE_IP = "10.90.0.101"
ROSBRIDGE_PORT = 9090

def test_mcp_logic():
    """测试MCP工具的逻辑"""
    print("=== 测试MCP工具逻辑 ===\n")
    
    # 模拟get_robot_position(source="amcl")的逻辑
    print("1. 测试AMCL位置获取逻辑:")
    local_ws_manager = WebSocketManager(ROSBRIDGE_IP, ROSBRIDGE_PORT, LOCAL_IP)
    
    try:
        local_amcl_pose = PoseWithCovarianceStamped(local_ws_manager, topic="/pose")
        position = local_amcl_pose.get_position(timeout=5.0)
        local_ws_manager.close()
        
        if position:
            result = {
                "source": "amcl",
                "position": position,
                "status": "success"
            }
            print("✓ AMCL位置获取成功:")
            print(f"  结果: {result}")
        else:
            result = {
                "source": "amcl",
                "position": None,
                "status": "failed",
                "error": "Failed to get AMCL position"
            }
            print("✗ AMCL位置获取失败:")
            print(f"  结果: {result}")
    except Exception as e:
        local_ws_manager.close()
        result = {
            "source": "amcl",
            "position": None,
            "status": "failed",
            "error": f"AMCL position error: {e}"
        }
        print(f"✗ AMCL位置获取异常: {e}")
        print(f"  结果: {result}")
    
    print("\n" + "="*50 + "\n")
    
    # 模拟get_robot_position(source="odom")的逻辑
    print("2. 测试里程计位置获取逻辑:")
    local_ws_manager = WebSocketManager(ROSBRIDGE_IP, ROSBRIDGE_PORT, LOCAL_IP)
    
    try:
        local_odometry = Odometry(local_ws_manager, topic="/Odometry")
        position = local_odometry.get_position(timeout=5.0)
        local_ws_manager.close()
        
        if position:
            result = {
                "source": "odometry",
                "position": position,
                "status": "success"
            }
            print("✓ 里程计位置获取成功:")
            print(f"  结果: {result}")
        else:
            result = {
                "source": "odometry", 
                "position": None,
                "status": "failed",
                "error": "Failed to get odometry position"
            }
            print("✗ 里程计位置获取失败:")
            print(f"  结果: {result}")
    except Exception as e:
        local_ws_manager.close()
        result = {
            "source": "odometry",
            "position": None,
            "status": "failed",
            "error": f"Odometry position error: {e}"
        }
        print(f"✗ 里程计位置获取异常: {e}")
        print(f"  结果: {result}")

if __name__ == "__main__":
    test_mcp_logic() 