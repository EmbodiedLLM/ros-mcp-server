# ROS MCP Server

![Static Badge](https://img.shields.io/badge/ROS-Available-green)
![Static Badge](https://img.shields.io/badge/ROS2-Available-green)
[![smithery badge](https://smithery.ai/badge/@lpigeon/ros-mcp-server)](https://smithery.ai/server/@lpigeon/ros-mcp-server)
![Static Badge](https://img.shields.io/badge/License-MIT-blue)

> **[中文文档 / Chinese Documentation](README_zh.md)**

A FastMCP-based ROS robot control server that allows AI assistants like Claude to control ROS robots through the MCP protocol.

## 🚀 Quick Start

### Method 1: Docker Deployment (Recommended)

```bash
# Clone the project
git clone https://github.com/EmbodiedLLM/ros-mcp-server.git
cd ros-mcp-server

# One-click deployment
./deploy-docker.sh

# Or use Makefile
make deploy
```

### Method 2: Local Development

```bash
# Install uv (if not already installed)
curl -LsSf https://astral.sh/uv/install.sh | sh

# Install dependencies
uv sync

# Start the server
uv run server.py
```

## ⚙️ Configuration

### ROS Bridge Setup

1. Modify IP configuration in `server.py`:

```python
LOCAL_IP = "10.90.0.101"        # Local machine IP
ROSBRIDGE_IP = "10.90.0.101"    # ROS Bridge server IP
ROSBRIDGE_PORT = 9090           # ROS Bridge port
```

2. Start ROS Bridge:

```bash
# ROS 1
roslaunch rosbridge_server rosbridge_websocket.launch

# ROS 2
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### Claude Desktop Configuration

Add to Claude Desktop configuration file:

```json
{
  "mcpServers": {
    "ros-mcp-server": {
      "url": "http://localhost:8000/mcp"
    }
  }
}
```

Configuration file locations:

- **macOS**: `~/Library/Application Support/Claude/claude_desktop_config.json`
- **Windows**: `%APPDATA%\Claude\claude_desktop_config.json`

## 🧪 Testing

```bash
# Quick test
make test

# Docker environment test
make test-docker

# View all commands
make help
```

## 🛠️ Main Features

### Robot Control

- `pub_twist()` - Publish velocity control commands
- `send_nav2_goal()` - Send navigation goals
- `send_waypoints()` - Send multiple waypoints

### Status Retrieval

- `get_robot_position()` - Get robot position
- `get_robot_velocity()` - Get robot velocity
- `get_topics()` - Get ROS topic list

### Sensor Data

- `sub_image()` - Subscribe to camera images
- `sub_jointstate()` - Subscribe to joint states

## 🐳 Docker Commands

```bash
# Build image
make build

# Start services
make up

# Stop services
make down

# View logs
make logs

# Clean resources
make clean
```

## 🔧 Environment Variables

Create `.env` file for configuration:

```bash
# MCP server configuration
MCP_PORT=8000
MCP_TRANSPORT=streamable-http

# ROS configuration
ROSBRIDGE_IP=10.90.0.101
ROSBRIDGE_PORT=9090
LOCAL_IP=10.90.0.101

# Mirror source configuration (build acceleration)
USE_MIRROR=true
APT_MIRROR=mirrors.tuna.tsinghua.edu.cn
PIP_MIRROR=https://pypi.tuna.tsinghua.edu.cn/simple
```

## 📁 Project Structure

```
ros-mcp-server/
├── server.py              # Main server file
├── msgs/                   # ROS message types
├── utils/                  # Utility classes
├── tests/                  # Test files
├── docker-compose.yml      # Docker configuration
├── Dockerfile             # Docker image
├── deploy-docker.sh       # One-click deployment script
├── Makefile              # Convenient commands
├── README.md             # Documentation (English)
└── README_zh.md          # Documentation (Chinese)
```

## 🤝 Contributing

Issues and Pull Requests are welcome!

## 📄 License

MIT License
