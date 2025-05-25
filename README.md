# ROS MCP Server

![Static Badge](https://img.shields.io/badge/ROS-Available-green)
![Static Badge](https://img.shields.io/badge/ROS2-Available-green)
[![smithery badge](https://smithery.ai/badge/@lpigeon/ros-mcp-server)](https://smithery.ai/server/@lpigeon/ros-mcp-server)
![Static Badge](https://img.shields.io/badge/License-MIT-blue)

一个基于FastMCP的ROS机器人控制服务器，允许Claude等AI助手通过MCP协议控制ROS机器人。

## 🚀 快速开始

### 方法1：Docker部署（推荐）

```bash
# 克隆项目
git clone https://github.com/lpigeon/ros-mcp-server.git
cd ros-mcp-server

# 一键部署
./deploy-docker.sh

# 或使用Makefile
make deploy
```

### 方法2：本地开发

```bash
# 安装uv（如果没有）
curl -LsSf https://astral.sh/uv/install.sh | sh

# 安装依赖
uv sync

# 启动服务器
uv run server.py
```

## ⚙️ 配置

### ROS Bridge配置

1. 修改`server.py`中的IP配置：
```python
LOCAL_IP = "10.90.0.101"        # 本机IP
ROSBRIDGE_IP = "10.90.0.101"    # ROS Bridge服务器IP
ROSBRIDGE_PORT = 9090           # ROS Bridge端口
```

2. 启动ROS Bridge：
```bash
# ROS 1
roslaunch rosbridge_server rosbridge_websocket.launch

# ROS 2
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### Claude Desktop配置

在Claude Desktop配置文件中添加：

```json
{
  "mcpServers": {
    "ros-mcp-server": {
      "url": "http://localhost:8000/mcp"
    }
  }
}
```

配置文件位置：
- **macOS**: `~/Library/Application Support/Claude/claude_desktop_config.json`
- **Windows**: `%APPDATA%\Claude\claude_desktop_config.json`

## 🧪 测试

```bash
# 快速测试
make test

# Docker环境测试
make test-docker

# 查看所有命令
make help
```

## 🛠️ 主要功能

### 机器人控制
- `pub_twist()` - 发布速度控制命令
- `send_nav2_goal()` - 发送导航目标点
- `send_waypoints()` - 发送多个航点

### 状态获取
- `get_robot_position()` - 获取机器人位置
- `get_robot_velocity()` - 获取机器人速度
- `get_topics()` - 获取ROS话题列表

### 传感器数据
- `sub_image()` - 订阅相机图像
- `sub_jointstate()` - 订阅关节状态

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

## 🔧 环境变量

创建`.env`文件配置：

```bash
# MCP服务器配置
MCP_PORT=8000
MCP_TRANSPORT=streamable-http

# ROS配置
ROSBRIDGE_IP=10.90.0.101
ROSBRIDGE_PORT=9090
LOCAL_IP=10.90.0.101

# 镜像源配置（加速构建）
USE_MIRROR=true
APT_MIRROR=mirrors.tuna.tsinghua.edu.cn
PIP_MIRROR=https://pypi.tuna.tsinghua.edu.cn/simple
```

## 📁 项目结构

```
ros-mcp-server/
├── server.py              # 主服务器文件
├── msgs/                   # ROS消息类型
├── utils/                  # 工具类
├── tests/                  # 测试文件
├── docker-compose.yml      # Docker配置
├── Dockerfile             # Docker镜像
├── deploy-docker.sh       # 一键部署脚本
├── Makefile              # 便捷命令
└── README.md             # 说明文档
```

## 🤝 贡献

欢迎提交Issue和Pull Request！

## �� 许可证

MIT License
