## Overview
![Static Badge](https://img.shields.io/badge/ROS-Available-green)
![Static Badge](https://img.shields.io/badge/ROS2-Available-green)
[![smithery badge](https://smithery.ai/badge/@lpigeon/ros-mcp-server)](https://smithery.ai/server/@lpigeon/ros-mcp-server)
![Static Badge](https://img.shields.io/badge/License-MIT-blue)

<center><img src="https://github.com/lpigeon/ros-mcp-server/blob/main/img/framework.png"/></center>

The ROS MCP Server is designed to support robots in performing complex tasks and adapting effectively to various environments by providing a set of functions that transform natural language commands, entered by a user through an LLM, into ROS commands for robot control. Furthermore, by utilizing ``rosbridge``, it is configured to operate with both ``ROS`` and ``ROS2`` systems, and its WebSocket-based communication enables broad applicability across diverse platforms.

Research based on this project can be found in the video linked below.  
- [An Efficient Robot Control Framework Using the Model Context Protocol](https://www.youtube.com/watch?v=7ut4eqTxwHA)

## Supported Types

- geometry_msgs/Twist
- geometry_msgs/PoseStamped (Nav2 goals)
- geometry_msgs/PoseArray (Nav2 waypoints)
- sensor_msgs/Image
- sensor_msgs/JointState
- Service calls (Nav2 waypoint following control)

## Features

- **WebSocket-based universal compatibility**: Communicates with both ROS and ROS2 systems using rosbridge, enabling seamless integration regardless of ROS version.
- **Cross-platform support**: Works on Linux, Windows, and MacOS, making it suitable for diverse development and deployment environments.
- **Easy integration with LLMs and AI systems**: Natural language commands can be directly translated into robot actions via MCP functions.
- **Nav2 navigation support**: Send navigation goals and control waypoint following (equivalent to RViz2 Nav2 Goal functionality).
- **Extensible function set**: Easily add new robot control or sensor functions by extending the MCP tool interface.
- **No ROS node modification required**: Interacts with existing ROS/ROS2 topics and services without changing your robot's core code.
- **Native ROS/ROS2 command compatibility**: Optionally supports using local ROS/ROS2 libraries, so you can run native ROS commands and tools alongside WebSocket-based control.

## Navigation Features

### Single Goal Navigation
Send individual navigation goals to Nav2:
```python
# Send a simple navigation goal
send_nav2_goal(x=2.0, y=3.0, yaw=1.57)

# Send detailed goal with quaternion orientation
send_nav2_goal_detailed(x=2.0, y=3.0, z=0.0, qx=0.0, qy=0.0, qz=0.707, qw=0.707)
```

### Waypoint Following
Control multi-waypoint navigation (equivalent to RViz2 waypoint functionality):
```python
# Define waypoints: [[x, y, yaw], [x, y, yaw], ...]
waypoints = [
    [1.0, 2.0, 0.0],      # First waypoint
    [3.0, 4.0, 1.57],     # Second waypoint  
    [5.0, 6.0, 3.14]      # Third waypoint
]

# Send waypoints and start following
send_and_start_waypoints(waypoints)

# Or control step by step
send_waypoints(waypoints)
start_waypoint_following()
pause_waypoint_following()
resume_waypoint_following()
stop_waypoint_following()
```

For detailed examples, see [WAYPOINT_EXAMPLES.md](WAYPOINT_EXAMPLES.md).

### Position and Localization
Get current robot position and motion state:
```python
# Get robot position (automatically selects best source)
position = get_robot_position()

# Get position from specific source
amcl_position = get_robot_position(source="amcl")    # Map-based localization
odom_position = get_robot_position(source="odom")    # Odometry-based position

# Get current velocity
velocity = get_robot_velocity()

# Get complete odometry information
odometry = get_robot_odometry()

# Get AMCL pose with covariance
amcl_pose = get_robot_amcl_pose()
```

**Position data includes:**
- Coordinates (x, y, z)
- Orientation (yaw angle in radians and degrees)
- Quaternion representation
- Frame ID (coordinate system)
- Velocity information (for odometry)
- Uncertainty/covariance (for AMCL)

## Contributing
Contributions are welcome!  
Whether you're fixing a typo, adding a new function, or suggesting improvements, your help is appreciated.  
Please follow the [contributing guidelines](CONTRIBUTING.md) for more details on how to contribute to this project.

## Installation

### Installing via Smithery

To install ``ros-mcp-server`` for Claude Desktop automatically via [Smithery](https://smithery.ai/server/@lpigeon/ros-mcp-server):

```bash
npx -y @smithery/cli install @lpigeon/ros-mcp-server --client claude
```

### Installing Locally

### `uv` Installation
- To install `uv`, you can use the following command:
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```
or
```bash
pip install uv
```

- Create virtual environment and activate it (Optional)
```bash
uv venv
source .venv/bin/activate
```

### MCP Server Configuration
Set MCP setting to mcp.json.

```bash
"ros-mcp-server": {
  "command": "uv",
  "args": [
    "--directory",
    "/ABSOLUTE/PATH/TO/PARENT/FOLDER/ros-mcp-server",,
    "run",
    "server.py"
  ]
}
```

If you use Claude Desktop, you can find mcp.json using the following command:

- MacOS/Linux
```bash
code ~/Library/Application\ Support/Claude/claude_desktop_config.json
```

- Windows
```bash
code $env:AppData\Claude\claude_desktop_config.json
```

## MCP Functions

You can find the list of functions in the [MCPFUNCTIONS.md](MCPFUNCTIONS.md).

## How To Use
### 1. Set IP and Port to connect rosbridge.
- Open `server.py` and change your `LOCAL_IP`, `ROSBRIDGE_IP` and `ROSBRIDGE_PORT`. (`ROSBRIDGE_PORT`'s default value is `9090`)

### 2. Run rosbridge server.
ROS 1
```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```
ROS 2
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### 3. Run any AI system that has imported ``ros-mcp-server``.

### 4. Type "Make the robot move forward.".
<center><img src="https://github.com/lpigeon/ros-mcp-server/blob/main/img/how_to_use_1.png" width="500"/></center>

### 5. Check `rosbridge_server` and `ros topic`.
- `rosbridge_server`
<center><img src="https://github.com/lpigeon/ros-mcp-server/blob/main/img/how_to_use_2.png" /></center>

- `ros topic`
<center><img src="https://github.com/lpigeon/ros-mcp-server/blob/main/img/how_to_use_3.png" /></center>

## Simulation Test
MCP-based control using the MOCA mobile manipulator within the NVIDIA Isaac Sim simulation environment. 

<center><img src="https://github.com/lpigeon/ros-mcp-server/blob/main/img/result.gif" /></center>
