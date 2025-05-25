#!/usr/bin/env python3
"""
MCP STDIO代理服务器
对外提供stdio接口，对内连接HTTP MCP服务器
"""
import asyncio
import json
import sys
import logging
from typing import Any, Dict, List, Optional
from fastmcp import Client

# 配置日志到stderr，避免干扰stdio通信
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    stream=sys.stderr
)
logger = logging.getLogger(__name__)

class StdioProxy:
    def __init__(self, mcp_server_url: str = 'http://10.90.0.101:8000/mcp'):
        self.mcp_server_url = mcp_server_url
        self.client: Optional[Client] = None
        self.request_id = 0
        
    async def start(self):
        """启动代理服务器"""
        try:
            logger.info(f"正在连接到MCP服务器: {self.mcp_server_url}")
            self.client = Client(self.mcp_server_url)
            await self.client.__aenter__()
            logger.info("✅ 成功连接到MCP服务器")
            
            # 开始处理stdio消息
            await self.handle_stdio()
            
        except Exception as e:
            logger.error(f"❌ 启动失败: {e}")
            await self.send_error(-1, -32603, f"Internal error: {e}")
        finally:
            if self.client:
                await self.client.__aexit__(None, None, None)
    
    async def handle_stdio(self):
        """处理stdio输入输出"""
        logger.info("🔄 开始处理stdio消息...")
        
        while True:
            try:
                # 从stdin读取一行
                line = await asyncio.get_event_loop().run_in_executor(
                    None, sys.stdin.readline
                )
                
                if not line:
                    logger.info("📥 stdin关闭，退出")
                    break
                
                line = line.strip()
                if not line:
                    continue
                
                logger.info(f"📨 收到消息: {line[:100]}...")
                
                # 解析JSON-RPC消息
                try:
                    message = json.loads(line)
                except json.JSONDecodeError as e:
                    logger.error(f"❌ JSON解析错误: {e}")
                    await self.send_error(-1, -32700, "Parse error")
                    continue
                
                # 处理消息
                await self.handle_message(message)
                
            except Exception as e:
                logger.error(f"❌ 处理消息时出错: {e}")
                await self.send_error(-1, -32603, f"Internal error: {e}")
    
    async def handle_message(self, message: Dict[str, Any]):
        """处理单个JSON-RPC消息"""
        request_id = message.get('id', -1)
        method = message.get('method')
        params = message.get('params', {})
        
        logger.info(f"🔧 处理方法: {method}")
        
        try:
            if method == 'initialize':
                await self.handle_initialize(request_id, params)
            elif method == 'tools/list':
                await self.handle_list_tools(request_id)
            elif method == 'tools/call':
                await self.handle_call_tool(request_id, params)
            elif method == 'resources/list':
                await self.handle_list_resources(request_id)
            elif method == 'resources/read':
                await self.handle_read_resource(request_id, params)
            elif method == 'prompts/list':
                await self.handle_list_prompts(request_id)
            elif method == 'prompts/get':
                await self.handle_get_prompt(request_id, params)
            else:
                logger.warning(f"⚠️  未知方法: {method}")
                await self.send_error(request_id, -32601, f"Method not found: {method}")
                
        except Exception as e:
            logger.error(f"❌ 处理方法 {method} 时出错: {e}")
            await self.send_error(request_id, -32603, f"Internal error: {e}")
    
    async def handle_initialize(self, request_id: int, params: Dict[str, Any]):
        """处理初始化请求"""
        response = {
            "jsonrpc": "2.0",
            "id": request_id,
            "result": {
                "protocolVersion": "2024-11-05",
                "capabilities": {
                    "tools": {},
                    "resources": {},
                    "prompts": {}
                },
                "serverInfo": {
                    "name": "ros-mcp-stdio-proxy",
                    "version": "1.0.0"
                }
            }
        }
        await self.send_response(response)
    
    async def handle_list_tools(self, request_id: int):
        """处理工具列表请求"""
        try:
            tools = await self.client.list_tools()
            tools_data = []
            
            for tool in tools:
                tool_data = {
                    "name": tool.name,
                    "description": tool.description
                }
                if hasattr(tool, 'inputSchema') and tool.inputSchema:
                    tool_data["inputSchema"] = tool.inputSchema
                tools_data.append(tool_data)
            
            response = {
                "jsonrpc": "2.0",
                "id": request_id,
                "result": {
                    "tools": tools_data
                }
            }
            await self.send_response(response)
            
        except Exception as e:
            logger.error(f"❌ 获取工具列表失败: {e}")
            await self.send_error(request_id, -32603, f"Failed to list tools: {e}")
    
    async def handle_call_tool(self, request_id: int, params: Dict[str, Any]):
        """处理工具调用请求"""
        try:
            tool_name = params.get('name')
            arguments = params.get('arguments', {})
            
            if not tool_name:
                await self.send_error(request_id, -32602, "Missing tool name")
                return
            
            logger.info(f"🔧 调用工具: {tool_name} with {arguments}")
            result = await self.client.call_tool(tool_name, arguments)
            
            response = {
                "jsonrpc": "2.0",
                "id": request_id,
                "result": {
                    "content": [
                        {
                            "type": "text",
                            "text": str(result)
                        }
                    ]
                }
            }
            await self.send_response(response)
            
        except Exception as e:
            logger.error(f"❌ 工具调用失败: {e}")
            await self.send_error(request_id, -32603, f"Tool call failed: {e}")
    
    async def handle_list_resources(self, request_id: int):
        """处理资源列表请求"""
        # 暂时返回空列表，可以根据需要扩展
        response = {
            "jsonrpc": "2.0",
            "id": request_id,
            "result": {
                "resources": []
            }
        }
        await self.send_response(response)
    
    async def handle_read_resource(self, request_id: int, params: Dict[str, Any]):
        """处理资源读取请求"""
        await self.send_error(request_id, -32601, "Resources not implemented")
    
    async def handle_list_prompts(self, request_id: int):
        """处理提示列表请求"""
        # 暂时返回空列表，可以根据需要扩展
        response = {
            "jsonrpc": "2.0",
            "id": request_id,
            "result": {
                "prompts": []
            }
        }
        await self.send_response(response)
    
    async def handle_get_prompt(self, request_id: int, params: Dict[str, Any]):
        """处理获取提示请求"""
        await self.send_error(request_id, -32601, "Prompts not implemented")
    
    async def send_response(self, response: Dict[str, Any]):
        """发送响应到stdout"""
        response_json = json.dumps(response, ensure_ascii=False)
        print(response_json, flush=True)
        logger.info(f"📤 发送响应: {response_json[:100]}...")
    
    async def send_error(self, request_id: int, code: int, message: str):
        """发送错误响应"""
        error_response = {
            "jsonrpc": "2.0",
            "id": request_id,
            "error": {
                "code": code,
                "message": message
            }
        }
        await self.send_response(error_response)

async def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='MCP STDIO代理服务器')
    parser.add_argument(
        '--server-url', 
        default='http://10.90.0.101:8000/mcp',
        help='MCP服务器URL (默认: http://10.90.0.101:8000/mcp)'
    )
    
    args = parser.parse_args()
    
    logger.info(f"🚀 启动MCP STDIO代理服务器")
    logger.info(f"📡 目标服务器: {args.server_url}")
    
    proxy = StdioProxy(args.server_url)
    await proxy.start()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("👋 收到中断信号，正在退出...")
    except Exception as e:
        logger.error(f"❌ 程序异常退出: {e}")
        sys.exit(1) 