# ROS MCP Server - 使用示例

本文档提供了不同使用场景下的详细安装和配置示例。

## 📋 使用场景

### 场景1：仅作为ROS服务器使用

如果你只需要运行ROS MCP服务器，不需要stdio包装器功能：

```bash
# 安装服务器依赖
uv sync --group server

# 或使用便捷脚本
./scripts/install.sh server

# 或使用Makefile
make install-server

# 启动服务器
python server.py
```

**依赖说明：**
- `fastmcp`: MCP协议核心库
- `opencv-python`: 图像处理
- `websocket`: WebSocket通信
- `websocket-client`: WebSocket客户端

### 场景2：仅作为MCP客户端使用

如果你只需要使用stdio包装器连接到远程的ROS MCP服务器：

```bash
# 安装客户端依赖
uv sync --group client

# 或使用便捷脚本
./scripts/install.sh client

# 或使用Makefile
make install-client

# 设置远程服务器URL
export MCP_SERVER_URL="http://remote-server:8000/mcp"

# 启动stdio包装器
python stdio_wrapper.py
```

**依赖说明：**
- `fastmcp`: MCP协议核心库
- `aiohttp`: 异步HTTP客户端
- `requests`: HTTP请求库

### 场景3：开发和测试

如果你需要进行开发、测试或贡献代码：

```bash
# 安装开发依赖
uv sync --group dev

# 或使用便捷脚本
./scripts/install.sh dev

# 或使用Makefile
make install-dev

# 运行测试
pytest tests/

# 代码格式化
black .

# 类型检查
mypy .

# 代码风格检查
flake8 .
```

**依赖说明：**
- `pytest`: 测试框架
- `pytest-asyncio`: 异步测试支持
- `black`: 代码格式化
- `flake8`: 代码风格检查
- `mypy`: 类型检查
- `pre-commit`: Git钩子管理

### 场景4：完整安装

如果你需要所有功能（服务器、客户端、开发工具）：

```bash
# 安装所有依赖
uv sync --group all

# 或使用便捷脚本
./scripts/install.sh all

# 或使用Makefile
make install-all
```

## 🔧 配置示例

### Claude Desktop配置

#### 本地stdio模式（推荐）

```json
{
  "mcpServers": {
    "ros-mcp-server": {
      "command": "python",
      "args": ["/path/to/ros-mcp-server/start_stdio.py"],
      "cwd": "/path/to/ros-mcp-server"
    }
  }
}
```

#### 分离模式（高级用户）

```json
{
  "mcpServers": {
    "ros-mcp-server": {
      "command": "python",
      "args": ["/path/to/ros-mcp-server/stdio_wrapper.py"],
      "env": {
        "MCP_SERVER_URL": "http://localhost:8000/mcp"
      },
      "cwd": "/path/to/ros-mcp-server"
    }
  }
}
```

### 环境变量配置

创建 `.env` 文件：

```bash
# MCP服务器配置
MCP_HOST=0.0.0.0
MCP_PORT=8000
MCP_TRANSPORT=streamable-http

# ROS配置
ROSBRIDGE_IP=10.90.0.101
ROSBRIDGE_PORT=9090
LOCAL_IP=10.90.0.101

# 客户端配置
MCP_SERVER_URL=http://localhost:8000/mcp
DEBUG=false
```

## 🚀 快速启动指南

### 1. 本地开发（推荐新手）

```bash
# 克隆项目
git clone https://github.com/EmbodiedLLM/ros-mcp-server.git
cd ros-mcp-server

# 安装所有依赖
make install-all

# 启动服务器和包装器
python start_stdio.py

# 在另一个终端测试
echo '{"jsonrpc": "2.0", "id": 1, "method": "tools/list"}' | python stdio_wrapper.py
```

### 2. 生产环境部署

```bash
# 服务器端
make install-server
python server.py

# 客户端（另一台机器）
make install-client
export MCP_SERVER_URL="http://server-ip:8000/mcp"
python stdio_wrapper.py
```

### 3. Docker部署

```bash
# 一键部署
make deploy

# 或手动部署
make build
make up
```

## 🔍 故障排除

### 依赖冲突

如果遇到依赖冲突，可以清理环境重新安装：

```bash
# 清理虚拟环境
rm -rf .venv

# 重新安装
uv sync --group all
```

### 权限问题

在Linux/macOS上，确保脚本有执行权限：

```bash
chmod +x scripts/install.sh
chmod +x deploy-docker.sh
```

### 网络问题

如果在中国大陆，可以使用镜像源：

```bash
# 设置pip镜像
export PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple

# 重新安装
uv sync --group all
```

## 📚 更多资源

- [README.md](README.md) - 项目主文档
- [README_zh.md](README_zh.md) - 中文文档
- [POSITION_EXAMPLES.md](POSITION_EXAMPLES.md) - 位置示例
- [tests/](tests/) - 测试用例
- [utils/](utils/) - 工具类文档 