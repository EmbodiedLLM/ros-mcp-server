# ROS MCP Server

![Static Badge](https://img.shields.io/badge/ROS-Available-green)
![Static Badge](https://img.shields.io/badge/ROS2-Available-green)
[![smithery badge](https://smithery.ai/badge/@lpigeon/ros-mcp-server)](https://smithery.ai/server/@lpigeon/ros-mcp-server)
![Static Badge](https://img.shields.io/badge/License-MIT-blue)

> **[中文文档 / Chinese Documentation](README_zh.md)**

A FastMCP-based ROS robot control server that allows AI assistants like Claude to control ROS robots through the MCP protocol. Supports both HTTP and stdio transport for maximum compatibility.

## 📋 Table of Contents

- [Quick Start](#-quick-start)
- [Installation](#-installation)
- [Configuration](#️-configuration)
- [Client Integration](#-client-integration)
- [Deployment Scenarios](#-deployment-scenarios)
- [Features](#️-features)
- [Testing](#-testing)
- [Troubleshooting](#-troubleshooting)
- [Project Structure](#-project-structure)

## 🚀 Quick Start

### For Impatient Users

```bash
# 1. Clone and install
git clone https://github.com/EmbodiedLLM/ros-mcp-server.git
cd ros-mcp-server
make install-all

# 2. Start everything (local development)
python start_stdio.py

# 3. Configure Claude Desktop (see Client Integration section)
```

## 📦 Installation

### Prerequisites

- Python 3.10+
- [uv](https://docs.astral.sh/uv/) package manager
- ROS/ROS2 with rosbridge_server (for server deployment)

### Install uv (if not already installed)

```bash
# Linux/macOS
curl -LsSf https://astral.sh/uv/install.sh | sh

# Windows
powershell -c "irm https://astral.sh/uv/install.ps1 | iex"
```

### Choose Your Installation

The project uses dependency groups for different use cases:

#### Option 1: Full Installation (Recommended for beginners)

```bash
# Install everything
uv sync --group all
# or: make install-all
# or: ./scripts/install.sh all
```

#### Option 2: Server Only (ROS machine)

```bash
# For machines running ROS
uv sync --group server
# or: make install-server
# or: ./scripts/install.sh server
```

#### Option 3: Client Only (Remote machine)

```bash
# For machines connecting to remote ROS server
uv sync --group client
# or: make install-client
# or: ./scripts/install.sh client
```

#### Option 4: Development (Contributors)

```bash
# For development and testing
uv sync --group dev
# or: make install-dev
# or: ./scripts/install.sh dev
```

## ⚙️ Configuration

### Step 1: ROS Bridge Setup

First, ensure ROS Bridge is running on your ROS machine:

```bash
# ROS 1
roslaunch rosbridge_server rosbridge_websocket.launch

# ROS 2
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### Step 2: Environment Configuration

Create a `.env` file in the project root:

```bash
# Copy the example
cp env.example .env

# Edit the configuration
nano .env
```

**For Local Development:**
```bash
# MCP server configuration
MCP_HOST=127.0.0.1
MCP_PORT=8000
MCP_TRANSPORT=streamable-http

# ROS configuration
ROSBRIDGE_IP=localhost
ROSBRIDGE_PORT=9090
LOCAL_IP=127.0.0.1

# Client configuration
MCP_SERVER_URL=http://localhost:8000/mcp
DEBUG=false
```

**For Remote Server Deployment:**
```bash
# MCP server configuration (on ROS machine)
MCP_HOST=0.0.0.0              # Listen on all interfaces
MCP_PORT=8000
MCP_TRANSPORT=streamable-http

# ROS configuration
ROSBRIDGE_IP=localhost        # Local ROS Bridge
ROSBRIDGE_PORT=9090
LOCAL_IP=192.168.1.100       # Your server's IP

# Client configuration (on client machine)
MCP_SERVER_URL=http://192.168.1.100:8000/mcp
DEBUG=false
```

### Step 3: IP Configuration

Modify the IP addresses according to your network setup:

- `LOCAL_IP`: The IP address of the machine running the MCP server
- `ROSBRIDGE_IP`: The IP address of the ROS Bridge server
- `MCP_SERVER_URL`: The full URL to connect to the MCP server

## 🔌 Client Integration

### Claude Desktop Integration

#### Method 1: Using uv run (Recommended)

Add to your Claude Desktop configuration file:

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

#### Method 2: Remote Server Connection

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

#### Method 3: Using Python directly

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

**Configuration file locations:**
- **macOS**: `~/Library/Application Support/Claude/claude_desktop_config.json`
- **Windows**: `%APPDATA%\Claude\claude_desktop_config.json`
- **Linux**: `~/.config/claude/claude_desktop_config.json`

### Other MCP Clients

#### HTTP Transport (Direct)

```json
{
  "mcpServers": {
    "ros-mcp-server": {
      "url": "http://localhost:8000/mcp"
    }
  }
}
```

#### Custom Integration

```python
# HTTP transport
import requests

response = requests.post("http://localhost:8000/mcp", json={
    "jsonrpc": "2.0",
    "id": 1,
    "method": "tools/list"
})

# Stdio transport
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

## 🏗️ Deployment Scenarios

### Scenario 1: Local Development

**Use case**: Everything on one machine for development and testing.

```bash
# 1. Install all dependencies
make install-all

# 2. Start ROS Bridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# 3. Start MCP server and stdio wrapper
python start_stdio.py

# 4. Configure Claude Desktop with local settings
```

### Scenario 2: Remote Server Deployment

**Use case**: ROS runs on a robot/server, Claude Desktop on your laptop.

**On the ROS machine (server):**
```bash
# 1. Install server dependencies
make install-server

# 2. Configure for remote access
export MCP_HOST=0.0.0.0
export LOCAL_IP=192.168.1.100  # Your server IP

# 3. Start ROS Bridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# 4. Start MCP server
uv run server.py
```

**On your laptop (client):**
```bash
# 1. Install client dependencies
make install-client

# 2. Configure Claude Desktop
# Use Method 2 from Client Integration section
# Set MCP_SERVER_URL to http://192.168.1.100:8000/mcp
```

### Scenario 3: Docker Deployment

**Use case**: Containerized deployment for production.

```bash
# 1. One-click deployment
make deploy

# 2. Or manual deployment
make build
make up

# 3. Check logs
make logs
```

### Scenario 4: Development and Testing

**Use case**: Contributing to the project or extensive testing.

```bash
# 1. Install development dependencies
make install-dev

# 2. Run tests
pytest tests/

# 3. Code formatting
black .

# 4. Type checking
mypy .
```

## 🛠️ Features

### Robot Control
- `pub_twist()` - Publish velocity control commands
- `send_nav2_goal()` - Send navigation goals
- `send_waypoints()` - Send multiple waypoints
- `start_waypoint_following()` - Start waypoint following
- `stop_waypoint_following()` - Stop waypoint following
- `cancel_waypoint_following()` - Cancel current navigation

### Status Retrieval
- `get_robot_position()` - Get robot position (AMCL/Odometry)
- `get_robot_velocity()` - Get robot velocity
- `get_robot_odometry()` - Get complete odometry data
- `get_robot_amcl_pose()` - Get AMCL localization data
- `get_topics()` - Get ROS topic list

### Sensor Data
- `sub_image()` - Subscribe to camera images
- `sub_jointstate()` - Subscribe to joint states
- `pub_jointstate()` - Publish joint states

## 🧪 Testing

### Quick Test

```bash
# Test the installation
make test

# Test specific components
uv run tests/quick_test.py
```

### Docker Environment Test

```bash
make test-docker
```

### Manual Testing

```bash
# 1. Start the server
python start_stdio.py

# 2. In another terminal, test the stdio wrapper
echo '{"jsonrpc": "2.0", "id": 1, "method": "tools/list"}' | python stdio_wrapper.py

# 3. Test HTTP endpoint
curl -X POST http://localhost:8000/mcp \
  -H "Content-Type: application/json" \
  -d '{"jsonrpc": "2.0", "id": 1, "method": "tools/list"}'
```

## 🔍 Troubleshooting

### Common Issues

#### 1. "uv not found"
```bash
# Install uv
curl -LsSf https://astral.sh/uv/install.sh | sh
source ~/.bashrc
```

#### 2. "Connection refused" when connecting to remote server
```bash
# Check if server is running
curl http://server-ip:8000/health

# Check firewall settings
sudo ufw allow 8000

# Verify MCP_HOST is set to 0.0.0.0 on server
```

#### 3. "ROS Bridge connection failed"
```bash
# Check if ROS Bridge is running
rostopic list  # ROS 1
ros2 topic list  # ROS 2

# Start ROS Bridge if not running
roslaunch rosbridge_server rosbridge_websocket.launch  # ROS 1
ros2 launch rosbridge_server rosbridge_websocket_launch.xml  # ROS 2
```

#### 4. "Module not found" errors
```bash
# Reinstall dependencies
rm -rf .venv
uv sync --group all
```

#### 5. Permission denied on scripts
```bash
chmod +x scripts/install.sh
chmod +x deploy-docker.sh
```

### Network Issues in China

```bash
# Use mirror sources
export PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple
uv sync --group all
```

### Debug Mode

```bash
# Enable debug logging
export DEBUG=true
python stdio_wrapper.py
```

## 📁 Project Structure

```
ros-mcp-server/
├── server.py              # Main MCP server
├── stdio_wrapper.py       # Stdio transport wrapper
├── start_stdio.py         # Automatic launcher
├── scripts/               # Installation scripts
│   ├── install.sh         # Linux/macOS installer
│   └── install.bat        # Windows installer
├── msgs/                   # ROS message types
├── utils/                  # Utility classes
├── tests/                  # Test files
├── config/                # Configuration files
├── docker-compose.yml      # Docker configuration
├── Dockerfile             # Docker image
├── deploy-docker.sh       # Docker deployment script
├── Makefile              # Build commands
├── pyproject.toml         # Project configuration
├── env.example           # Environment variables template
├── README.md             # This file
└── README_zh.md          # Chinese documentation
```

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

## 🤝 Contributing

1. Fork the repository
2. Install development dependencies: `make install-dev`
3. Make your changes
4. Run tests: `pytest tests/`
5. Format code: `black .`
6. Submit a pull request

## 📄 License

MIT License - see [LICENSE](LICENSE) file for details.

## 🆘 Support

- **Issues**: [GitHub Issues](https://github.com/EmbodiedLLM/ros-mcp-server/issues)
- **Discussions**: [GitHub Discussions](https://github.com/EmbodiedLLM/ros-mcp-server/discussions)
- **Documentation**: This README and the Chinese version
