#!/usr/bin/env python3

from utils.websocket_manager import WebSocketManager
from msgs.geometry_msgs import PoseStamped

# 配置
ROSBRIDGE_IP = "10.90.0.101"
ROSBRIDGE_PORT = 9090
LOCAL_IP = "10.90.0.101"

def test_different_topics():
    ws_manager = WebSocketManager(ROSBRIDGE_IP, ROSBRIDGE_PORT, LOCAL_IP)
    
    # 测试不同的可能话题
    topics_to_test = [
        "/goal_pose",
        "/move_base_simple/goal", 
        "/clicked_point",
        "/nav2_goal",
        "/navigation_goal"
    ]
    
    for topic in topics_to_test:
        try:
            print(f"\n测试话题: {topic}")
            goal_pose = PoseStamped(ws_manager, topic=topic)
            msg = goal_pose.publish_simple(1.0, 0.0, 0.0, "map")
            if msg:
                print(f"✅ {topic} 发送成功")
            else:
                print(f"❌ {topic} 发送失败")
        except Exception as e:
            print(f"❌ {topic} 异常: {e}")
    
    ws_manager.close()

def test_different_actions():
    ws_manager = WebSocketManager(ROSBRIDGE_IP, ROSBRIDGE_PORT, LOCAL_IP)
    
    # 测试不同的可能action
    actions_to_test = [
        "/navigate_to_pose",
        "/bt_navigator/navigate_to_pose",
        "/nav2/navigate_to_pose", 
        "/NavigateToPose",
        "/move_base"
    ]
    
    for action in actions_to_test:
        try:
            print(f"\n测试Action: {action}")
            
            goal_msg = {
                "op": "send_action_goal",
                "action": action,
                "action_type": "nav2_msgs/action/NavigateToPose",
                "goal": {
                    "pose": {
                        "header": {
                            "stamp": {"sec": 0, "nanosec": 0},
                            "frame_id": "map"
                        },
                        "pose": {
                            "position": {"x": 1.0, "y": 0.0, "z": 0.0},
                            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                        }
                    }
                }
            }
            
            ws_manager.send(goal_msg)
            print(f"✅ {action} 发送成功")
            
        except Exception as e:
            print(f"❌ {action} 异常: {e}")
    
    ws_manager.close()

if __name__ == "__main__":
    print("=== 测试话题发布 ===")
    test_different_topics()
    
    print("\n=== 测试Action发布 ===") 
    test_different_actions() 