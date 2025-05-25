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

# Install dependencies based on your needs:

# For server only (ROS MCP server)
uv sync --group server
# or: make install-server
# or: ./scripts/install.sh server

# For client only (stdio wrapper)
uv sync --group client
# or: make install-client
# or: ./scripts/install.sh client

# For development (includes testing tools)
uv sync --group dev
# or: make install-dev
# or: ./scripts/install.sh dev

# For everything (full installation)
uv sync --group all
# or: make install-all
# or: ./scripts/install.sh all

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

## 🔌 Client Integration

### Method 1: Stdio Local Transport (Recommended for Claude Desktop)

Most MCP clients like Claude Desktop use stdio local transport. Use our stdio wrapper for seamless integration:

#### Quick Start with Stdio

```bash
# Start both HTTP server and stdio wrapper
python start_stdio.py

# Or start them separately
python start_stdio.py --server-only    # Terminal 1: Start HTTP server
python start_stdio.py --wrapper-only   # Terminal 2: Start stdio wrapper
```

#### Claude Desktop Configuration

Add to Claude Desktop configuration file:

**Option 1: Using uv run (Recommended)**

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

**Option 2: Using uv run with python**

```json
{
  "mcpServers": {
    "ros-mcp-server": {
      "command": "uv",
      "args": ["run", "python", "start_stdio.py"],
      "cwd": "/path/to/ros-mcp-server"
    }
  }
}
```

**Option 3: Using direct python (if uv not available)**

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

**Option 4: Stdio wrapper only (server must be running separately)**

```json
{
  "mcpServers": {
    "ros-mcp-server": {
      "command": "uv",
      "args": ["run", "stdio_wrapper.py"],
      "env": {
        "MCP_SERVER_URL": "http://localhost:8000/mcp"
      },
      "cwd": "/path/to/ros-mcp-server"
    }
  }
}
```

Configuration file locations:

- **macOS**: `~/Library/Application Support/Claude/claude_desktop_config.json`
- **Windows**: `%APPDATA%\Claude\claude_desktop_config.json`
- **Linux**: `~/.config/claude/claude_desktop_config.json`

### Method 2: Direct HTTP Connection

For clients that support HTTP transport directly:

```json
{
  "mcpServers": {
    "ros-mcp-server": {
      "url": "http://localhost:8000/mcp"
    }
  }
}
```

### Method 3: Custom Integration

For custom MCP clients, you can integrate directly:

```python
# HTTP transport
import requests

response = requests.post("http://localhost:8000/mcp", json={
    "jsonrpc": "2.0",
    "id": 1,
    "method": "tools/list"
})

# Stdio transport (using our wrapper)
import subprocess
import json

process = subprocess.Popen(
    ["python", "stdio_wrapper.py"],
    stdin=subprocess.PIPE,
    stdout=subprocess.PIPE,
    text=True
)

# Send request
request = {"jsonrpc": "2.0", "id": 1, "method": "tools/list"}
process.stdin.write(json.dumps(request) + "\n")
process.stdin.flush()

# Read response
response = json.loads(process.stdout.readline())
```

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
├── stdio_wrapper.py       # Stdio transport wrapper
├── start_stdio.py         # Stdio launcher script
├── scripts/               # Installation scripts
│   ├── install.sh         # Linux/macOS installation script
│   └── install.bat        # Windows installation script
├── msgs/                   # ROS message types
├── utils/                  # Utility classes
├── tests/                  # Test files
├── docker-compose.yml      # Docker configuration
├── Dockerfile             # Docker image
├── deploy-docker.sh       # One-click deployment script
├── Makefile              # Convenient commands
├── pyproject.toml         # Project configuration and dependency management
├── README.md             # Documentation (English)
├── README_zh.md          # Documentation (Chinese)
└── USAGE_EXAMPLES.md     # Usage examples and scenarios
```

## 📖 Documentation

- **[USAGE_EXAMPLES.md](USAGE_EXAMPLES.md)** - Detailed usage examples for different scenarios
- **[POSITION_EXAMPLES.md](POSITION_EXAMPLES.md)** - Robot position examples

## 🤝 Contributing

Issues and Pull Requests are welcome!

## 📄 License

MIT License
