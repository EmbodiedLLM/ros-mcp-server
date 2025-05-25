#!/usr/bin/env python3
"""
测试新的stdio MCP客户端 - 连接远程服务器
"""
import asyncio
import json
import subprocess
import sys
import os
from typing import Dict, Any

async def test_stdio_client():
    """测试stdio客户端的MCP协议实现"""
    print("🚀 开始测试stdio MCP客户端 (连接远程服务器)\n")
    
    # 设置环境变量 - 连接到远程服务器
    env = os.environ.copy()
    env["MCP_SERVER_URL"] = "http://10.90.0.101:8000/mcp"
    env["DEBUG"] = "true"
    
    try:
        # 启动stdio客户端进程
        process = subprocess.Popen(
            [sys.executable, "stdio_wrapper.py"],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            env=env
        )
        
        print("✅ stdio客户端进程已启动")
        
        # 等待一下让进程启动
        await asyncio.sleep(2)
        
        # 检查进程是否还在运行
        if process.poll() is not None:
            stderr_output = process.stderr.read()
            print(f"❌ 进程已退出，错误信息: {stderr_output}")
            return False
        
        # 测试1: 初始化
        print("\n🔧 测试1: 初始化")
        init_request = {
            "jsonrpc": "2.0",
            "id": 1,
            "method": "initialize",
            "params": {
                "protocolVersion": "2024-11-05",
                "capabilities": {
                    "roots": {
                        "listChanged": True
                    },
                    "sampling": {}
                },
                "clientInfo": {
                    "name": "test-client",
                    "version": "1.0.0"
                }
            }
        }
        
        await send_request(process, init_request)
        response = await read_response(process, timeout=10)
        
        if response and "result" in response:
            print("✅ 初始化成功")
            print(f"   服务器信息: {response['result'].get('serverInfo', {})}")
        else:
            print(f"❌ 初始化失败: {response}")
            # 检查stderr
            stderr_output = await read_stderr(process)
            if stderr_output:
                print(f"   错误信息: {stderr_output}")
            return False
        
        # 发送初始化完成通知
        init_notification = {
            "jsonrpc": "2.0",
            "method": "notifications/initialized"
        }
        await send_request(process, init_notification)
        
        # 测试2: 获取工具列表
        print("\n🛠️  测试2: 获取工具列表")
        tools_request = {
            "jsonrpc": "2.0",
            "id": 2,
            "method": "tools/list"
        }
        
        await send_request(process, tools_request)
        response = await read_response(process, timeout=10)
        
        if response and "result" in response:
            tools = response["result"].get("tools", [])
            print(f"✅ 获取到 {len(tools)} 个工具")
            for i, tool in enumerate(tools[:5]):  # 显示前5个工具
                print(f"   {i+1}. {tool['name']}: {tool['description'][:50]}...")
            if len(tools) > 5:
                print(f"   ... 还有 {len(tools) - 5} 个工具")
        else:
            print(f"❌ 获取工具列表失败: {response}")
            stderr_output = await read_stderr(process)
            if stderr_output:
                print(f"   错误信息: {stderr_output}")
            return False
        
        # 测试3: 调用工具
        print("\n🧪 测试3: 调用工具")
        if tools:
            # 尝试调用第一个工具
            first_tool = tools[0]["name"]
            call_request = {
                "jsonrpc": "2.0",
                "id": 3,
                "method": "tools/call",
                "params": {
                    "name": first_tool,
                    "arguments": {}
                }
            }
            
            await send_request(process, call_request)
            response = await read_response(process, timeout=15)
            
            if response and "result" in response:
                content = response["result"].get("content", [])
                is_error = response["result"].get("isError", False)
                
                if is_error:
                    print(f"⚠️  工具调用返回错误 (可能是ROS未连接): {content[0]['text'] if content else 'Unknown error'}")
                else:
                    print(f"✅ 工具调用成功: {first_tool}")
                    if content:
                        result_text = content[0].get("text", "")
                        print(f"   结果: {result_text[:100]}...")
            else:
                print(f"❌ 工具调用失败: {response}")
                stderr_output = await read_stderr(process)
                if stderr_output:
                    print(f"   错误信息: {stderr_output}")
        
        # 关闭进程
        process.terminate()
        try:
            await asyncio.wait_for(asyncio.create_task(wait_for_process(process)), timeout=5)
        except asyncio.TimeoutError:
            process.kill()
        
        print("\n🏁 测试完成")
        return True
        
    except Exception as e:
        print(f"❌ 测试过程中出错: {e}")
        if 'process' in locals():
            process.terminate()
        return False

async def send_request(process: subprocess.Popen, request: Dict[str, Any]):
    """发送请求到stdio进程"""
    request_line = json.dumps(request) + "\n"
    process.stdin.write(request_line)
    process.stdin.flush()
    print(f"📤 发送请求: {request['method']}")

async def read_response(process: subprocess.Popen, timeout: int = 5) -> Dict[str, Any]:
    """从stdio进程读取响应"""
    try:
        # 使用asyncio读取stdout，带超时
        line = await asyncio.wait_for(
            asyncio.get_event_loop().run_in_executor(None, process.stdout.readline),
            timeout=timeout
        )
        
        if line:
            response = json.loads(line.strip())
            print(f"📥 收到响应: {response.get('id', 'no-id')}")
            return response
        else:
            print("📥 收到空响应")
            return None
    except asyncio.TimeoutError:
        print(f"⏰ 读取响应超时 ({timeout}秒)")
        return None
    except Exception as e:
        print(f"❌ 读取响应失败: {e}")
        return None

async def read_stderr(process: subprocess.Popen) -> str:
    """读取stderr输出"""
    try:
        # 非阻塞读取stderr
        import select
        if select.select([process.stderr], [], [], 0)[0]:
            return process.stderr.read()
        return ""
    except:
        return ""

async def wait_for_process(process: subprocess.Popen):
    """等待进程结束"""
    await asyncio.get_event_loop().run_in_executor(None, process.wait)

def test_network_connectivity():
    """测试网络连接"""
    import socket
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        result = sock.connect_ex(('10.90.0.101', 8000))
        sock.close()
        return result == 0
    except Exception as e:
        print(f"网络连接测试失败: {e}")
        return False

if __name__ == "__main__":
    print("🔍 检查远程服务器网络连接 (10.90.0.101:8000)...")
    if not test_network_connectivity():
        print("❌ 无法连接到远程服务器!")
        print("请确保:")
        print("1. 远程服务器 10.90.0.101:8000 正在运行")
        print("2. 网络连接正常")
        print("3. 防火墙允许访问端口8000")
        sys.exit(1)
    
    print("✅ 网络连接正常，开始测试MCP协议...")
    
    result = asyncio.run(test_stdio_client())
    print(f"\n🏁 测试结果: {'成功' if result else '失败'}") 