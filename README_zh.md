# ROS MCP Server

![Static Badge](https://img.shields.io/badge/ROS-Available-green)
![Static Badge](https://img.shields.io/badge/ROS2-Available-green)
[![smithery badge](https://smithery.ai/badge/@lpigeon/ros-mcp-server)](https://smithery.ai/server/@lpigeon/ros-mcp-server)
![Static Badge](https://img.shields.io/badge/License-MIT-blue)

> **[English Documentation](README.md)**

一个基于FastMCP的ROS机器人控制服务器，允许Claude等AI助手通过MCP协议控制ROS机器人。同时支持HTTP和stdio传输，具有最大兼容性。

## 📋 目录

- [快速开始](#-快速开始)
- [安装](#-安装)
- [配置](#️-配置)
- [客户端接入](#-客户端接入)
- [部署场景](#️-部署场景)
- [功能特性](#️-功能特性)
- [测试](#-测试)
- [故障排除](#-故障排除)
- [项目结构](#-项目结构)

## 🚀 快速开始

### 急性子用户

```bash
# 1. 克隆并安装
git clone https://github.com/EmbodiedLLM/ros-mcp-server.git
cd ros-mcp-server
make install-all

# 2. 启动所有服务（本地开发）
python start_stdio.py

# 3. 配置Claude Desktop（参见客户端接入部分）
```

## 📦 安装

### 前置要求

- Python 3.10+
- [uv](https://docs.astral.sh/uv/) 包管理器
- ROS/ROS2 和 rosbridge_server（用于服务器部署）

### 安装uv（如果尚未安装）

```bash
# Linux/macOS
curl -LsSf https://astral.sh/uv/install.sh | sh

# Windows
powershell -c "irm https://astral.sh/uv/install.ps1 | iex"
```

### 选择安装方式

项目使用依赖组来支持不同的使用场景：

#### 选项1：完整安装（推荐新手）

```bash
# 安装所有依赖
uv sync --group all
# 或: make install-all
# 或: ./scripts/install.sh all
```

#### 选项2：仅服务器端（ROS机器）

```bash
# 用于运行ROS的机器
uv sync --group server
# 或: make install-server
# 或: ./scripts/install.sh server
```

#### 选项3：仅客户端（远程机器）

```bash
# 用于连接远程ROS服务器的机器
uv sync --group client
# 或: make install-client
# 或: ./scripts/install.sh client
```

#### 选项4：开发环境（贡献者）

```bash
# 用于开发和测试
uv sync --group dev
# 或: make install-dev
# 或: ./scripts/install.sh dev
```

## ⚙️ 配置

### 步骤1：ROS Bridge设置

首先，确保ROS Bridge在你的ROS机器上运行：

```bash
# ROS 1
roslaunch rosbridge_server rosbridge_websocket.launch

# ROS 2
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### 步骤2：环境配置

在项目根目录创建`.env`文件：

```bash
# 复制示例文件
cp env.example .env

# 编辑配置
nano .env
```

**本地开发配置：**
```bash
# MCP服务器配置
MCP_HOST=127.0.0.1
MCP_PORT=8000
MCP_TRANSPORT=streamable-http

# ROS配置
ROSBRIDGE_IP=localhost
ROSBRIDGE_PORT=9090
LOCAL_IP=127.0.0.1

# 客户端配置
MCP_SERVER_URL=http://localhost:8000/mcp
DEBUG=false
```

**远程服务器部署配置：**
```bash
# MCP服务器配置（在ROS机器上）
MCP_HOST=0.0.0.0              # 监听所有接口
MCP_PORT=8000
MCP_TRANSPORT=streamable-http

# ROS配置
ROSBRIDGE_IP=localhost        # 本地ROS Bridge
ROSBRIDGE_PORT=9090
LOCAL_IP=192.168.1.100       # 你的服务器IP

# 客户端配置（在客户端机器上）
MCP_SERVER_URL=http://192.168.1.100:8000/mcp
DEBUG=false
```

### 步骤3：IP配置

根据你的网络设置修改IP地址：

- `LOCAL_IP`: 运行MCP服务器的机器IP地址
- `ROSBRIDGE_IP`: ROS Bridge服务器的IP地址
- `MCP_SERVER_URL`: 连接到MCP服务器的完整URL

## 🔌 客户端接入

### Claude Desktop集成

#### 方法1：使用uv run（推荐）

在Claude Desktop配置文件中添加：

```json
{
  "mcpServers": {
    "ros-mcp-server": {
      "command": "uv",
      "args": ["run", "start_stdio.py"],
      "cwd": "/path/to/ros-mcp-server"
    }
  }
}
```

#### 方法2：远程服务器连接

```json
{
  "mcpServers": {
    "ros-mcp-server": {
      "command": "uv",
      "args": ["run", "stdio_wrapper.py"],
      "env": {
        "MCP_SERVER_URL": "http://your-server-ip:8000/mcp"
      },
      "cwd": "/path/to/ros-mcp-server"
    }
  }
}
```

#### 方法3：直接使用Python

```json
{
  "mcpServers": {
    "ros-mcp-server": {
      "command": "python",
      "args": ["/path/to/ros-mcp-server/start_stdio.py"]
    }
  }
}
```

**配置文件位置：**
- **macOS**: `~/Library/Application Support/Claude/claude_desktop_config.json`
- **Windows**: `%APPDATA%\Claude\claude_desktop_config.json`
- **Linux**: `~/.config/claude/claude_desktop_config.json`

### 其他MCP客户端

#### HTTP传输（直接）

```json
{
  "mcpServers": {
    "ros-mcp-server": {
      "url": "http://localhost:8000/mcp"
    }
  }
}
```

#### 自定义集成

```python
# HTTP传输
import requests

response = requests.post("http://localhost:8000/mcp", json={
    "jsonrpc": "2.0",
    "id": 1,
    "method": "tools/list"
})

# Stdio传输
import subprocess
import json

process = subprocess.Popen(
    ["uv", "run", "stdio_wrapper.py"],
    stdin=subprocess.PIPE,
    stdout=subprocess.PIPE,
    text=True,
    env={"MCP_SERVER_URL": "http://server-ip:8000/mcp"}
)

request = {"jsonrpc": "2.0", "id": 1, "method": "tools/list"}
process.stdin.write(json.dumps(request) + "\n")
process.stdin.flush()

response = json.loads(process.stdout.readline())
```

## 🏗️ 部署场景

### 场景1：本地开发

**使用场景**：在一台机器上进行开发和测试。

```bash
# 1. 安装所有依赖
make install-all

# 2. 启动ROS Bridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# 3. 启动MCP服务器和stdio包装器
python start_stdio.py

# 4. 使用本地设置配置Claude Desktop
```

### 场景2：远程服务器部署

**使用场景**：ROS运行在机器人/服务器上，Claude Desktop在你的笔记本电脑上。

**在ROS机器上（服务器）：**
```bash
# 1. 安装服务器依赖
make install-server

# 2. 配置远程访问
export MCP_HOST=0.0.0.0
export LOCAL_IP=192.168.1.100  # 你的服务器IP

# 3. 启动ROS Bridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# 4. 启动MCP服务器
uv run server.py
```

**在你的笔记本电脑上（客户端）：**
```bash
# 1. 安装客户端依赖
make install-client

# 2. 配置Claude Desktop
# 使用客户端接入部分的方法2
# 设置MCP_SERVER_URL为http://192.168.1.100:8000/mcp
```

### 场景3：Docker部署

**使用场景**：生产环境的容器化部署。

```bash
# 1. 一键部署
make deploy

# 2. 或手动部署
make build
make up

# 3. 查看日志
make logs
```

### 场景4：开发和测试

**使用场景**：为项目贡献代码或进行大量测试。

```bash
# 1. 安装开发依赖
make install-dev

# 2. 运行测试
pytest tests/

# 3. 代码格式化
black .

# 4. 类型检查
mypy .
```

## 🛠️ 功能特性

### 机器人控制
- `pub_twist()` - 发布速度控制命令
- `send_nav2_goal()` - 发送导航目标点
- `send_waypoints()` - 发送多个航点
- `start_waypoint_following()` - 开始航点跟随
- `stop_waypoint_following()` - 停止航点跟随
- `cancel_waypoint_following()` - 取消当前导航

### 状态获取
- `get_robot_position()` - 获取机器人位置（AMCL/里程计）
- `get_robot_velocity()` - 获取机器人速度
- `get_robot_odometry()` - 获取完整里程计数据
- `get_robot_amcl_pose()` - 获取AMCL定位数据
- `get_topics()` - 获取ROS话题列表

### 传感器数据
- `sub_image()` - 订阅相机图像
- `sub_jointstate()` - 订阅关节状态
- `pub_jointstate()` - 发布关节状态

## 🧪 测试

### 快速测试

```bash
# 测试安装
make test

# 测试特定组件
uv run tests/quick_test.py
```

### Docker环境测试

```bash
make test-docker
```

### 手动测试

```bash
# 1. 启动服务器
python start_stdio.py

# 2. 在另一个终端测试stdio包装器
echo '{"jsonrpc": "2.0", "id": 1, "method": "tools/list"}' | python stdio_wrapper.py

# 3. 测试HTTP端点
curl -X POST http://localhost:8000/mcp \
  -H "Content-Type: application/json" \
  -d '{"jsonrpc": "2.0", "id": 1, "method": "tools/list"}'
```

## 🔍 故障排除

### 常见问题

#### 1. "uv not found"
```bash
# 安装uv
curl -LsSf https://astral.sh/uv/install.sh | sh
source ~/.bashrc
```

#### 2. 连接远程服务器时"Connection refused"
```bash
# 检查服务器是否运行
curl http://server-ip:8000/health

# 检查防火墙设置
sudo ufw allow 8000

# 验证服务器上MCP_HOST设置为0.0.0.0
```

#### 3. "ROS Bridge connection failed"
```bash
# 检查ROS Bridge是否运行
rostopic list  # ROS 1
ros2 topic list  # ROS 2

# 如果没有运行，启动ROS Bridge
roslaunch rosbridge_server rosbridge_websocket.launch  # ROS 1
ros2 launch rosbridge_server rosbridge_websocket_launch.xml  # ROS 2
```

#### 4. "Module not found"错误
```bash
# 重新安装依赖
rm -rf .venv
uv sync --group all
```

#### 5. 脚本权限被拒绝
```bash
chmod +x scripts/install.sh
chmod +x deploy-docker.sh
```

### 中国大陆网络问题

```bash
# 使用镜像源
export PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple
uv sync --group all
```

### 调试模式

```bash
# 启用调试日志
export DEBUG=true
python stdio_wrapper.py
```

## 📁 项目结构

```
ros-mcp-server/
├── server.py              # 主MCP服务器
├── stdio_wrapper.py       # Stdio传输包装器
├── start_stdio.py         # 自动启动器
├── scripts/               # 安装脚本
│   ├── install.sh         # Linux/macOS安装器
│   └── install.bat        # Windows安装器
├── msgs/                   # ROS消息类型
├── utils/                  # 工具类
├── tests/                  # 测试文件
├── config/                # 配置文件
├── docker-compose.yml      # Docker配置
├── Dockerfile             # Docker镜像
├── deploy-docker.sh       # Docker部署脚本
├── Makefile              # 构建命令
├── pyproject.toml         # 项目配置
├── env.example           # 环境变量模板
├── README.md             # 英文文档
└── README_zh.md          # 本文件
```

## 🐳 Docker命令

```bash
# 构建镜像
make build

# 启动服务
make up

# 停止服务
make down

# 查看日志
make logs

# 清理资源
make clean
```

## 🤝 贡献

1. Fork仓库
2. 安装开发依赖：`make install-dev`
3. 进行修改
4. 运行测试：`pytest tests/`
5. 格式化代码：`black .`
6. 提交Pull Request

## 📄 许可证

MIT许可证 - 详见[LICENSE](LICENSE)文件。

## 🆘 支持

- **问题反馈**: [GitHub Issues](https://github.com/EmbodiedLLM/ros-mcp-server/issues)
- **讨论**: [GitHub Discussions](https://github.com/EmbodiedLLM/ros-mcp-server/discussions)
- **文档**: 本README和英文版本
