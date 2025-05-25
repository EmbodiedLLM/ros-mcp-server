#!/usr/bin/env python3
"""
测试ROS MCP服务器连接的脚本
"""

import asyncio
import aiohttp
import json
import sys

async def test_mcp_server(url):
    """测试MCP服务器连接"""
    print(f"Testing MCP server at: {url}")
    
    try:
        async with aiohttp.ClientSession() as session:
            # 测试基本连接
            async with session.get(url) as response:
                if response.status == 200:
                    print("✅ Server is responding")
                    
                    # 尝试获取服务器信息
                    try:
                        data = await response.json()
                        print(f"✅ Server response: {data}")
                    except:
                        text = await response.text()
                        print(f"✅ Server response (text): {text[:200]}...")
                else:
                    print(f"❌ Server returned status code: {response.status}")
                    return False
                    
    except aiohttp.ClientConnectorError:
        print("❌ Cannot connect to server - check if server is running and URL is correct")
        return False
    except Exception as e:
        print(f"❌ Error testing server: {e}")
        return False
    
    return True

async def test_with_fastmcp_client(url):
    """使用FastMCP客户端测试连接"""
    try:
        from fastmcp import Client
        
        print(f"\nTesting with FastMCP client...")
        
        async with Client(url) as client:
            # 列出可用工具
            tools = await client.list_tools()
            print(f"✅ Available tools: {len(tools)} tools found")
            
            for tool in tools[:5]:  # 只显示前5个工具
                print(f"  - {tool.name}: {tool.description}")
            
            if len(tools) > 5:
                print(f"  ... and {len(tools) - 5} more tools")
                
            return True
            
    except ImportError:
        print("⚠️  FastMCP not installed, skipping client test")
        print("   Install with: pip install fastmcp")
        return True
    except Exception as e:
        print(f"❌ FastMCP client test failed: {e}")
        return False

def main():
    if len(sys.argv) != 2:
        print("Usage: python test_server.py <server_url>")
        print("Example: python test_server.py http://localhost:8000/mcp")
        sys.exit(1)
    
    url = sys.argv[1]
    
    async def run_tests():
        print("🚀 Starting MCP Server Tests\n")
        
        # 基本连接测试
        basic_test = await test_mcp_server(url)
        
        if basic_test:
            # FastMCP客户端测试
            await test_with_fastmcp_client(url)
        
        print("\n🏁 Tests completed")
        
        if basic_test:
            print("\n✅ Server appears to be working correctly!")
            print(f"\nTo connect from Claude Desktop, add this to your config:")
            print(f'{{')
            print(f'  "mcpServers": {{')
            print(f'    "ros-mcp-server": {{')
            print(f'      "url": "{url}"')
            print(f'    }}')
            print(f'  }}')
            print(f'}}')
        else:
            print("\n❌ Server test failed. Please check the server logs.")
    
    asyncio.run(run_tests())

if __name__ == "__main__":
    main() 