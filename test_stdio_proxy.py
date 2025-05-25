#!/usr/bin/env python3
"""
测试STDIO代理服务器
"""
import asyncio
import json
import subprocess
import sys
import time

class StdioProxyTester:
    def __init__(self, proxy_script: str = "./stdio_proxy.py"):
        self.proxy_script = proxy_script
        self.process = None
    
    async def start_proxy(self):
        """启动代理进程"""
        print("🚀 启动STDIO代理服务器...")
        self.process = await asyncio.create_subprocess_exec(
            sys.executable, self.proxy_script,
            stdin=asyncio.subprocess.PIPE,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE
        )
        
        # 等待一下让服务器启动
        await asyncio.sleep(1)
        print("✅ 代理服务器已启动")
    
    async def send_request(self, method: str, params: dict = None, request_id: int = 1):
        """发送JSON-RPC请求"""
        request = {
            "jsonrpc": "2.0",
            "id": request_id,
            "method": method
        }
        if params:
            request["params"] = params
        
        request_json = json.dumps(request) + "\n"
        print(f"📤 发送请求: {method}")
        
        # 发送请求
        self.process.stdin.write(request_json.encode())
        await self.process.stdin.drain()
        
        # 读取响应
        response_line = await self.process.stdout.readline()
        if response_line:
            response = json.loads(response_line.decode().strip())
            print(f"📥 收到响应: {json.dumps(response, indent=2, ensure_ascii=False)}")
            return response
        else:
            print("❌ 没有收到响应")
            return None
    
    async def test_sequence(self):
        """执行测试序列"""
        try:
            # 1. 初始化
            print("\n🔧 测试1: 初始化")
            response = await self.send_request("initialize", {
                "protocolVersion": "2024-11-05",
                "capabilities": {},
                "clientInfo": {
                    "name": "test-client",
                    "version": "1.0.0"
                }
            })
            
            if response and "result" in response:
                print("✅ 初始化成功")
            else:
                print("❌ 初始化失败")
                return False
            
            # 2. 获取工具列表
            print("\n🔧 测试2: 获取工具列表")
            response = await self.send_request("tools/list", {}, 2)
            
            if response and "result" in response:
                tools = response["result"].get("tools", [])
                print(f"✅ 获取到 {len(tools)} 个工具")
                for i, tool in enumerate(tools[:5]):  # 显示前5个
                    print(f"  {i+1}. {tool['name']}: {tool['description'][:50]}...")
            else:
                print("❌ 获取工具列表失败")
                return False
            
            # 3. 测试工具调用
            print("\n🔧 测试3: 调用工具")
            if tools:
                # 尝试调用第一个工具
                first_tool = tools[0]
                response = await self.send_request("tools/call", {
                    "name": first_tool["name"],
                    "arguments": {}
                }, 3)
                
                if response and "result" in response:
                    print(f"✅ 工具调用成功: {first_tool['name']}")
                else:
                    print(f"⚠️  工具调用失败: {first_tool['name']} (可能是参数问题)")
            
            return True
            
        except Exception as e:
            print(f"❌ 测试过程中出错: {e}")
            return False
    
    async def stop_proxy(self):
        """停止代理进程"""
        if self.process:
            print("\n🛑 停止代理服务器...")
            self.process.terminate()
            await self.process.wait()
            print("✅ 代理服务器已停止")
    
    async def run_test(self):
        """运行完整测试"""
        try:
            await self.start_proxy()
            success = await self.test_sequence()
            return success
        finally:
            await self.stop_proxy()

async def main():
    """主函数"""
    print("🧪 开始测试STDIO代理服务器\n")
    
    tester = StdioProxyTester()
    success = await tester.run_test()
    
    print(f"\n🏁 测试完成: {'成功' if success else '失败'}")
    return success

if __name__ == "__main__":
    try:
        result = asyncio.run(main())
        sys.exit(0 if result else 1)
    except KeyboardInterrupt:
        print("\n👋 测试被中断")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ 测试异常: {e}")
        sys.exit(1) 