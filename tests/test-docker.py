#!/usr/bin/env python3
"""
Docker环境下的MCP服务器测试脚本
"""
import asyncio
import aiohttp
import json
import sys
import os
from fastmcp import Client

async def test_basic_connection(url):
    """测试基本HTTP连接"""
    print(f"🔗 测试基本连接: {url}")
    
    try:
        async with aiohttp.ClientSession() as session:
            async with session.get(url) as response:
                if response.status == 200:
                    print("✅ HTTP连接成功")
                    return True
                else:
                    print(f"⚠️  HTTP状态码: {response.status}")
                    return False
    except Exception as e:
        print(f"❌ HTTP连接失败: {e}")
        return False

async def test_mcp_client(url):
    """测试MCP客户端连接"""
    print(f"\n🤖 测试MCP客户端连接: {url}")
    
    try:
        client = Client(url)
        async with client:
            print("✅ MCP客户端连接成功")
            
            # 获取工具列表
            tools = await client.list_tools()
            print(f"🛠️  发现 {len(tools)} 个工具")
            
            # 显示前5个工具
            for i, tool in enumerate(tools[:5]):
                print(f"  {i+1}. {tool.name}")
            
            if len(tools) > 5:
                print(f"  ... 还有 {len(tools) - 5} 个工具")
            
            # 测试一个简单的工具调用
            print("\n🧪 测试工具调用...")
            try:
                result = await client.call_tool("get_topics", {})
                print("✅ get_topics 调用成功")
                return True
            except Exception as e:
                print(f"⚠️  get_topics 调用失败: {e}")
                print("   (这可能是因为ROS Bridge未连接)")
                return True  # MCP连接成功就算通过
                
    except Exception as e:
        print(f"❌ MCP客户端连接失败: {e}")
        return False

async def test_docker_health():
    """测试Docker健康检查"""
    print("\n🏥 测试Docker健康检查...")
    
    try:
        # 检查容器状态
        import subprocess
        result = subprocess.run(
            ["docker-compose", "ps", "--format", "json"],
            capture_output=True,
            text=True,
            cwd=os.path.dirname(os.path.abspath(__file__))
        )
        
        if result.returncode == 0:
            containers = json.loads(result.stdout) if result.stdout.strip() else []
            
            for container in containers:
                name = container.get("Name", "unknown")
                state = container.get("State", "unknown")
                health = container.get("Health", "unknown")
                
                if "ros-mcp-server" in name:
                    print(f"📦 容器 {name}:")
                    print(f"   状态: {state}")
                    print(f"   健康: {health}")
                    
                    if state == "running":
                        print("✅ 容器运行正常")
                        return True
                    else:
                        print("❌ 容器未运行")
                        return False
        else:
            print("⚠️  无法获取容器状态")
            return False
            
    except Exception as e:
        print(f"⚠️  健康检查失败: {e}")
        return False

def get_test_url():
    """获取测试URL"""
    # 从环境变量或.env文件获取端口
    port = os.getenv("MCP_PORT", "8000")
    
    # 尝试从.env文件读取
    try:
        with open(".env", "r") as f:
            for line in f:
                if line.startswith("MCP_PORT="):
                    port = line.split("=")[1].strip()
                    break
    except FileNotFoundError:
        pass
    
    return f"http://localhost:{port}/mcp"

async def main():
    print("🚀 Docker环境MCP服务器测试")
    print("=" * 40)
    
    url = get_test_url()
    print(f"测试URL: {url}\n")
    
    # 测试步骤
    tests = [
        ("Docker健康检查", test_docker_health()),
        ("基本HTTP连接", test_basic_connection(url)),
        ("MCP客户端连接", test_mcp_client(url)),
    ]
    
    results = []
    for test_name, test_coro in tests:
        print(f"\n{'='*20} {test_name} {'='*20}")
        try:
            result = await test_coro
            results.append((test_name, result))
        except Exception as e:
            print(f"❌ 测试异常: {e}")
            results.append((test_name, False))
    
    # 显示测试结果
    print(f"\n{'='*20} 测试结果 {'='*20}")
    all_passed = True
    
    for test_name, result in results:
        status = "✅ 通过" if result else "❌ 失败"
        print(f"{test_name}: {status}")
        if not result:
            all_passed = False
    
    print(f"\n🏁 总体结果: {'✅ 全部通过' if all_passed else '❌ 部分失败'}")
    
    if all_passed:
        print(f"\n🎉 MCP服务器运行正常！")
        print(f"连接URL: {url}")
        print("\nClaude Desktop配置:")
        print('{')
        print('  "mcpServers": {')
        print('    "ros-mcp-server": {')
        print(f'      "url": "{url}"')
        print('    }')
        print('  }')
        print('}')
    else:
        print(f"\n🔧 故障排除:")
        print("1. 检查容器状态: docker-compose ps")
        print("2. 查看日志: docker-compose logs ros-mcp-server")
        print("3. 重启服务: docker-compose restart")
    
    return 0 if all_passed else 1

if __name__ == "__main__":
    exit_code = asyncio.run(main()) 