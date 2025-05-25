#!/usr/bin/env python3
"""
ROS MCP Server - Stdio启动脚本
自动启动HTTP服务器和stdio包装器

Usage:
    python start_stdio.py [--server-only] [--wrapper-only]
    
Options:
    --server-only: 只启动HTTP服务器
    --wrapper-only: 只启动stdio包装器（需要服务器已运行）
"""

import os
import sys
import time
import signal
import argparse
import subprocess
import asyncio
from pathlib import Path

class StdioLauncher:
    def __init__(self):
        self.server_process = None
        self.wrapper_process = None
        self.shutdown = False
        
    def signal_handler(self, signum, frame):
        """处理信号"""
        print(f"\nReceived signal {signum}, shutting down...")
        self.shutdown = True
        self.cleanup()
        sys.exit(0)
        
    def cleanup(self):
        """清理进程"""
        if self.wrapper_process:
            print("Stopping stdio wrapper...")
            self.wrapper_process.terminate()
            try:
                self.wrapper_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.wrapper_process.kill()
                
        if self.server_process:
            print("Stopping HTTP server...")
            self.server_process.terminate()
            try:
                self.server_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.server_process.kill()
    
    def start_server(self):
        """启动HTTP服务器"""
        print("Starting ROS MCP HTTP server...")
        
        # 设置环境变量
        env = os.environ.copy()
        env["MCP_TRANSPORT"] = "streamable-http"
        env["MCP_HOST"] = "127.0.0.1"
        env["MCP_PORT"] = "8000"
        
        # 启动服务器
        self.server_process = subprocess.Popen(
            [sys.executable, "server.py"],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1
        )
        
        # 等待服务器启动
        print("Waiting for server to start...")
        for i in range(30):  # 最多等待30秒
            if self.server_process.poll() is not None:
                print("Server failed to start!")
                return False
                
            try:
                import requests
                response = requests.get("http://127.0.0.1:8000/health", timeout=1)
                if response.status_code == 200:
                    print("Server started successfully!")
                    return True
            except:
                pass
                
            time.sleep(1)
            
        print("Server startup timeout!")
        return False
    
    def start_wrapper(self):
        """启动stdio包装器"""
        print("Starting stdio wrapper...")
        
        # 设置环境变量
        env = os.environ.copy()
        env["MCP_SERVER_URL"] = "http://127.0.0.1:8000/mcp"
        
        # 启动包装器
        self.wrapper_process = subprocess.Popen(
            [sys.executable, "stdio_wrapper.py"],
            env=env,
            stdin=sys.stdin,
            stdout=sys.stdout,
            stderr=subprocess.PIPE,
            text=True
        )
        
        return True
    
    def run_server_only(self):
        """只运行HTTP服务器"""
        if not self.start_server():
            return False
            
        print("HTTP server is running. Press Ctrl+C to stop.")
        print("Connect URL: http://127.0.0.1:8000/mcp")
        
        try:
            while not self.shutdown:
                if self.server_process.poll() is not None:
                    print("Server process exited!")
                    break
                time.sleep(1)
        except KeyboardInterrupt:
            pass
            
        return True
    
    def run_wrapper_only(self):
        """只运行stdio包装器"""
        print("Starting stdio wrapper (server should be running)...")
        
        if not self.start_wrapper():
            return False
            
        try:
            self.wrapper_process.wait()
        except KeyboardInterrupt:
            pass
            
        return True
    
    def run_both(self):
        """运行服务器和包装器"""
        # 启动HTTP服务器
        if not self.start_server():
            return False
            
        # 启动stdio包装器
        if not self.start_wrapper():
            return False
            
        print("Both server and wrapper are running.")
        print("You can now connect MCP clients via stdio.")
        
        try:
            # 等待包装器进程
            self.wrapper_process.wait()
        except KeyboardInterrupt:
            pass
            
        return True

def main():
    parser = argparse.ArgumentParser(description="ROS MCP Server Stdio Launcher")
    parser.add_argument("--server-only", action="store_true", 
                       help="Only start HTTP server")
    parser.add_argument("--wrapper-only", action="store_true",
                       help="Only start stdio wrapper")
    
    args = parser.parse_args()
    
    # 检查必要文件
    if not Path("server.py").exists():
        print("Error: server.py not found!")
        sys.exit(1)
        
    if not args.server_only and not Path("stdio_wrapper.py").exists():
        print("Error: stdio_wrapper.py not found!")
        sys.exit(1)
    
    launcher = StdioLauncher()
    
    # 注册信号处理器
    signal.signal(signal.SIGINT, launcher.signal_handler)
    signal.signal(signal.SIGTERM, launcher.signal_handler)
    
    try:
        if args.server_only:
            success = launcher.run_server_only()
        elif args.wrapper_only:
            success = launcher.run_wrapper_only()
        else:
            success = launcher.run_both()
            
        if not success:
            sys.exit(1)
            
    finally:
        launcher.cleanup()

if __name__ == "__main__":
    main() 