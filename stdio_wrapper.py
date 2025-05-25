#!/usr/bin/env python3
"""
ROS MCP Server - Stdio Wrapper
用于将HTTP MCP服务器转换为stdio接口，支持Claude Desktop等客户端的本地传输

Usage:
    python stdio_wrapper.py

Environment Variables:
    MCP_SERVER_URL: MCP服务器URL (默认: http://localhost:8000/mcp)
    DEBUG: 启用调试模式 (默认: false)
"""

import sys
import json
import asyncio
import aiohttp
import logging
from typing import Dict, Any, Optional
import os
from urllib.parse import urljoin

# 配置日志
DEBUG = os.getenv("DEBUG", "false").lower() == "true"
if DEBUG:
    logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
else:
    logging.basicConfig(level=logging.WARNING)

logger = logging.getLogger(__name__)

class StdioMCPWrapper:
    """将HTTP MCP服务器包装为stdio接口"""
    
    def __init__(self, server_url: str = "http://localhost:8000/mcp"):
        self.server_url = server_url
        self.session: Optional[aiohttp.ClientSession] = None
        
    async def __aenter__(self):
        self.session = aiohttp.ClientSession()
        return self
        
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        if self.session:
            await self.session.close()
    
    async def send_request(self, request: Dict[str, Any]) -> Dict[str, Any]:
        """发送请求到HTTP MCP服务器"""
        if not self.session:
            raise RuntimeError("Session not initialized")
            
        try:
            logger.debug(f"Sending request: {request}")
            
            async with self.session.post(
                self.server_url,
                json=request,
                headers={"Content-Type": "application/json"}
            ) as response:
                if response.status == 200:
                    result = await response.json()
                    logger.debug(f"Received response: {result}")
                    return result
                else:
                    error_text = await response.text()
                    logger.error(f"HTTP error {response.status}: {error_text}")
                    return {
                        "jsonrpc": "2.0",
                        "id": request.get("id"),
                        "error": {
                            "code": -32603,
                            "message": f"HTTP error {response.status}: {error_text}"
                        }
                    }
        except Exception as e:
            logger.error(f"Request failed: {e}")
            return {
                "jsonrpc": "2.0", 
                "id": request.get("id"),
                "error": {
                    "code": -32603,
                    "message": f"Request failed: {str(e)}"
                }
            }
    
    async def handle_initialize(self, request: Dict[str, Any]) -> Dict[str, Any]:
        """处理初始化请求"""
        # 转发到HTTP服务器
        response = await self.send_request(request)
        return response
    
    async def handle_tools_list(self, request: Dict[str, Any]) -> Dict[str, Any]:
        """处理工具列表请求"""
        response = await self.send_request(request)
        return response
    
    async def handle_tools_call(self, request: Dict[str, Any]) -> Dict[str, Any]:
        """处理工具调用请求"""
        response = await self.send_request(request)
        return response
    
    async def process_request(self, request: Dict[str, Any]) -> Dict[str, Any]:
        """处理MCP请求"""
        method = request.get("method")
        
        if method == "initialize":
            return await self.handle_initialize(request)
        elif method == "tools/list":
            return await self.handle_tools_list(request)
        elif method == "tools/call":
            return await self.handle_tools_call(request)
        else:
            # 对于其他方法，直接转发
            return await self.send_request(request)
    
    async def run(self):
        """运行stdio包装器"""
        logger.info(f"Starting stdio wrapper for MCP server: {self.server_url}")
        
        try:
            while True:
                # 从stdin读取请求
                line = await asyncio.get_event_loop().run_in_executor(
                    None, sys.stdin.readline
                )
                
                if not line:
                    break
                    
                line = line.strip()
                if not line:
                    continue
                
                try:
                    request = json.loads(line)
                    logger.debug(f"Received request: {request}")
                    
                    # 处理请求
                    response = await self.process_request(request)
                    
                    # 发送响应到stdout
                    response_line = json.dumps(response, ensure_ascii=False)
                    print(response_line, flush=True)
                    logger.debug(f"Sent response: {response}")
                    
                except json.JSONDecodeError as e:
                    logger.error(f"Invalid JSON: {e}")
                    error_response = {
                        "jsonrpc": "2.0",
                        "id": None,
                        "error": {
                            "code": -32700,
                            "message": f"Parse error: {str(e)}"
                        }
                    }
                    print(json.dumps(error_response), flush=True)
                    
                except Exception as e:
                    logger.error(f"Error processing request: {e}")
                    error_response = {
                        "jsonrpc": "2.0", 
                        "id": request.get("id") if 'request' in locals() else None,
                        "error": {
                            "code": -32603,
                            "message": f"Internal error: {str(e)}"
                        }
                    }
                    print(json.dumps(error_response), flush=True)
                    
        except KeyboardInterrupt:
            logger.info("Shutting down stdio wrapper")
        except Exception as e:
            logger.error(f"Fatal error: {e}")
            sys.exit(1)

async def main():
    """主函数"""
    server_url = os.getenv("MCP_SERVER_URL", "http://localhost:8000/mcp")
    
    # 检查服务器是否可达
    try:
        async with aiohttp.ClientSession() as session:
            async with session.get(server_url.replace('/mcp', '/health')) as response:
                if response.status != 200:
                    logger.warning(f"Server health check failed: {response.status}")
    except Exception as e:
        logger.warning(f"Could not reach server at {server_url}: {e}")
        logger.info("Make sure the ROS MCP server is running with: python server.py")
    
    # 启动stdio包装器
    async with StdioMCPWrapper(server_url) as wrapper:
        await wrapper.run()

if __name__ == "__main__":
    asyncio.run(main()) 