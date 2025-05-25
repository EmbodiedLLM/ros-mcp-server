#!/usr/bin/env python3
"""
真实测试Nav2 waypoints功能
"""

import sys
import time
import json
from utils.websocket_manager import WebSocketManager
from utils.waypoint_sender import UniversalWaypointSender

def test_real_waypoints():
    """真实测试waypoints功能"""
    
    print("🚀 开始真实测试Nav2 waypoints功能...")
    
    # 使用你的配置
    LOCAL_IP = "0.0.0.0"
    ROSBRIDGE_IP = "0.0.0.0"
    ROSBRIDGE_PORT = 9090
    
    # 创建连接
    ws_manager = WebSocketManager(ROSBRIDGE_IP, ROSBRIDGE_PORT, LOCAL_IP)
    waypoint_sender = UniversalWaypointSender(ws_manager)
    
    # 定义测试航点（请根据你的地图调整坐标）
    test_waypoints = [
        [1.0, 1.0, 0.0],      # 第一个航点：x=1.0, y=1.0, 朝向0度
        [2.0, 1.0, 1.57],     # 第二个航点：x=2.0, y=1.0, 朝向90度
        [2.0, 2.0, 3.14],     # 第三个航点：x=2.0, y=2.0, 朝向180度
        [1.0, 2.0, -1.57]     # 第四个航点：x=1.0, y=2.0, 朝向-90度
    ]
    
    print(f"📍 测试航点: {test_waypoints}")
    
    # 测试1: 智能自动选择方式
    print("\n=== 测试1: 智能自动选择方式 ===")
    try:
        # 首先尝试action方式
        print("🔄 尝试action方式...")
        result = waypoint_sender.send_waypoints_via_action(test_waypoints)
        if result:
            print("✅ Action方式成功!")
            print(f"发送的消息: {json.dumps(result, indent=2, ensure_ascii=False)}")
            ws_manager.close()
            return "action"
        else:
            print("❌ Action方式失败")
    except Exception as e:
        print(f"❌ Action方式异常: {e}")
    
    # 测试2: Topic方式
    print("\n=== 测试2: Topic方式 ===")
    try:
        print("🔄 尝试topic方式...")
        result = waypoint_sender.send_waypoints_via_topic(test_waypoints)
        if result:
            print("✅ Topic方式成功!")
            print(f"发送的消息: {json.dumps(result, indent=2, ensure_ascii=False)}")
            ws_manager.close()
            return "topic"
        else:
            print("❌ Topic方式失败")
    except Exception as e:
        print(f"❌ Topic方式异常: {e}")
    
    # 测试3: Sequential方式
    print("\n=== 测试3: Sequential方式 ===")
    try:
        print("🔄 尝试sequential方式...")
        print("⚠️  注意：这将逐个发送目标，每个间隔2秒")
        
        # 只发送第一个航点作为测试
        single_waypoint = [test_waypoints[0]]
        result = waypoint_sender.send_sequential_goals(single_waypoint, delay=0.5)
        if result:
            print("✅ Sequential方式成功!")
            print(f"发送的消息数量: {len(result)}")
            print(f"第一个消息: {json.dumps(result[0], indent=2, ensure_ascii=False)}")
            ws_manager.close()
            return "sequential"
        else:
            print("❌ Sequential方式失败")
    except Exception as e:
        print(f"❌ Sequential方式异常: {e}")
    
    ws_manager.close()
    return None

def test_cancel_waypoints():
    """测试取消waypoints功能"""
    print("\n=== 测试取消waypoints功能 ===")
    
    LOCAL_IP = "0.0.0.0"
    ROSBRIDGE_IP = "0.0.0.0"
    ROSBRIDGE_PORT = 9090
    
    ws_manager = WebSocketManager(ROSBRIDGE_IP, ROSBRIDGE_PORT, LOCAL_IP)
    waypoint_sender = UniversalWaypointSender(ws_manager)
    
    try:
        result = waypoint_sender.cancel_waypoint_following()
        if result:
            print("✅ 取消waypoints成功!")
            print(f"发送的消息: {json.dumps(result, indent=2, ensure_ascii=False)}")
        else:
            print("❌ 取消waypoints失败")
    except Exception as e:
        print(f"❌ 取消waypoints异常: {e}")
    
    ws_manager.close()

def interactive_test():
    """交互式测试"""
    print("\n🎮 交互式测试模式")
    print("请输入你想测试的航点坐标（格式：x,y,yaw）")
    print("例如：1.0,2.0,0.0")
    print("输入'quit'退出")
    
    LOCAL_IP = "0.0.0.0"
    ROSBRIDGE_IP = "0.0.0.0"
    ROSBRIDGE_PORT = 9090
    
    waypoints = []
    
    while True:
        user_input = input(f"\n航点 {len(waypoints)+1} (或'quit'退出, 'send'发送): ").strip()
        
        if user_input.lower() == 'quit':
            break
        elif user_input.lower() == 'send':
            if waypoints:
                print(f"\n📤 发送 {len(waypoints)} 个航点...")
                
                ws_manager = WebSocketManager(ROSBRIDGE_IP, ROSBRIDGE_PORT, LOCAL_IP)
                waypoint_sender = UniversalWaypointSender(ws_manager)
                
                # 尝试智能发送
                try:
                    result = waypoint_sender.send_waypoints_via_action(waypoints)
                    if result:
                        print("✅ 航点发送成功!")
                    else:
                        print("❌ 航点发送失败，尝试其他方式...")
                        result = waypoint_sender.send_waypoints_via_topic(waypoints)
                        if result:
                            print("✅ 通过topic发送成功!")
                        else:
                            print("❌ 所有方式都失败")
                except Exception as e:
                    print(f"❌ 发送异常: {e}")
                
                ws_manager.close()
                waypoints = []
            else:
                print("❌ 没有航点可发送")
        else:
            try:
                parts = user_input.split(',')
                if len(parts) >= 2:
                    x = float(parts[0])
                    y = float(parts[1])
                    yaw = float(parts[2]) if len(parts) > 2 else 0.0
                    waypoints.append([x, y, yaw])
                    print(f"✅ 添加航点: ({x}, {y}, {yaw})")
                else:
                    print("❌ 格式错误，请使用 x,y,yaw 格式")
            except ValueError:
                print("❌ 坐标格式错误，请输入数字")

def main():
    """主测试函数"""
    print("🧪 Nav2 Waypoints 真实功能测试")
    print("=" * 50)
    
    # 检查连接
    print("🔗 检查ROS连接...")
    try:
        ws_manager = WebSocketManager("0.0.0.0", 9090, "0.0.0.0")
        topic_info = ws_manager.get_topics()
        if topic_info:
            print(f"✅ ROS连接正常，找到 {len(topic_info)} 个话题")
        else:
            print("❌ 无法获取话题列表，请检查rosbridge是否运行")
            return 1
        ws_manager.close()
    except Exception as e:
        print(f"❌ ROS连接失败: {e}")
        print("请确保rosbridge_server正在运行:")
        print("ros2 launch rosbridge_server rosbridge_websocket_launch.xml")
        return 1
    
    # 运行自动测试
    print("\n🤖 自动测试...")
    successful_method = test_real_waypoints()
    
    if successful_method:
        print(f"\n🎉 测试成功！推荐使用 {successful_method} 方式")
        
        # 测试取消功能
        time.sleep(1)
        test_cancel_waypoints()
        
        # 询问是否进行交互式测试
        user_choice = input("\n是否进行交互式测试？(y/n): ").strip().lower()
        if user_choice == 'y':
            interactive_test()
    else:
        print("\n❌ 所有自动测试都失败了")
        print("可能的原因:")
        print("1. Nav2没有正确配置waypoint following")
        print("2. 需要手动启动waypoint follower节点")
        print("3. 使用了非标准的Nav2配置")
        
        # 仍然提供交互式测试
        user_choice = input("\n是否尝试交互式测试？(y/n): ").strip().lower()
        if user_choice == 'y':
            interactive_test()
    
    print("\n✨ 测试完成!")
    return 0

if __name__ == "__main__":
    sys.exit(main()) 