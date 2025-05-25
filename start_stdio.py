#!/usr/bin/env python3
"""
快速测试MCP服务器连接
"""
import asyncio
from fastmcp import Client

async def test_mcp():
    try:
        print("🔗 正在连接到MCP服务器...")
        client = Client('http://10.90.0.101:8000/mcp')
        
        async with client:
            print("✅ 连接成功！")
            
            # 测试获取工具列表
            tools = await client.list_tools()
            print(f"🛠️  发现 {len(tools)} 个工具:")
            
            for i, tool in enumerate(tools[:10]):  # 显示前10个工具
                print(f"  {i+1}. {tool.name}: {tool.description[:60]}...")
            
            if len(tools) > 10:
                print(f"  ... 还有 {len(tools) - 10} 个工具")
            
            # 测试一个简单的工具调用
            print("\n🧪 测试工具调用...")
            try:
                result = await client.call_tool("get_topics", {})
                print(f"✅ get_topics 调用成功: {str(result)[:100]}...")
            except Exception as e:
                print(f"⚠️  get_topics 调用失败 (可能是ROS未连接): {e}")
            
            return True
            
    except Exception as e:
        print(f"❌ 连接失败: {e}")
        return False

if __name__ == "__main__":
    print("🚀 开始MCP服务器快速测试\n")
    result = asyncio.run(test_mcp())
    print(f"\n🏁 测试完成: {'成功' if result else '失败'}") 