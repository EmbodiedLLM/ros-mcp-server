# MCP Functions

This is a list of functions that can be used in the ROS MCP Server.

## get_topics
- **Purpose**: Retrieves the list of available topics from the robot's ROS system.
- **Returns**: List of topics (List[Any])

## pub_twist
- **Purpose**: Sends movement commands to the robot by setting linear and angular velocities.
- **Parameters**:
  - `linear`: Linear velocity (List[Any])
  - `angular`: Angular velocity (List[Any])

## pub_twist_seq
- **Purpose**: Sends a sequence of movement commands to the robot, allowing for multi-step motion control.
- **Parameters**:
  - `linear`: List of linear velocities (List[Any])
  - `angular`: List of angular velocities (List[Any])
  - `duration`: List of durations for each step (List[Any])
 
## sub_image
- **Purpose**: Receive images from the robot's point of view or of the surrounding environment.
- **Parameters**:
  - `save_path`: By default, the image is saved to the ``Downloads`` folder.

## pub_jointstate
- **Purpose**: Publishes a custom JointState message to the `/joint_states` topic.
- **Parameters**:
  - `name`: List of joint names (list[str])
  - `position`: List of joint positions (list[float])
  - `velocity`: List of joint velocities (list[float])
  - `effort`: List of joint efforts (list[float])

## sub_jointstate
- **Purpose**: Subscribes to the `/joint_states` topic and returns the latest JointState message as a formatted JSON string.
- **Returns**: JointState message (str)

## send_nav2_goal
- **Purpose**: Sends a navigation goal to nav2 using simple coordinates and yaw angle.
- **Parameters**:
  - `x`: Target x coordinate (float)
  - `y`: Target y coordinate (float)
  - `yaw`: Target orientation in radians (float, default: 0.0)
  - `frame_id`: Coordinate frame ID (str, default: "map")
- **Returns**: Success/failure message (str)

## send_nav2_goal_detailed
- **Purpose**: Sends a detailed navigation goal to nav2 with full pose information including quaternion orientation.
- **Parameters**:
  - `x`: Target x coordinate (float)
  - `y`: Target y coordinate (float)
  - `z`: Target z coordinate (float, default: 0.0)
  - `qx`: Quaternion x component (float, default: 0.0)
  - `qy`: Quaternion y component (float, default: 0.0)
  - `qz`: Quaternion z component (float, default: 0.0)
  - `qw`: Quaternion w component (float, default: 1.0)
  - `frame_id`: Coordinate frame ID (str, default: "map")
- **Returns**: Success/failure message (str)

## send_waypoints_auto
- **Purpose**: Intelligently sends waypoints using the best available method (RECOMMENDED).
- **Parameters**:
  - `waypoint_list`: List of waypoints in format [[x1, y1, yaw1], [x2, y2, yaw2], ...] (List[List[float]])
  - `frame_id`: Coordinate frame ID (str, default: "map")
- **Returns**: Success/failure message (str)

## send_waypoints_action
- **Purpose**: Sends waypoints via Nav2 action (standard Nav2 configurations).
- **Parameters**:
  - `waypoint_list`: List of waypoints in format [[x1, y1, yaw1], [x2, y2, yaw2], ...] (List[List[float]])
  - `frame_id`: Coordinate frame ID (str, default: "map")
- **Returns**: Success/failure message (str)

## send_waypoints_topic
- **Purpose**: Sends waypoints via topic (compatible with some custom configurations).
- **Parameters**:
  - `waypoint_list`: List of waypoints in format [[x1, y1, yaw1], [x2, y2, yaw2], ...] (List[List[float]])
  - `frame_id`: Coordinate frame ID (str, default: "map")
- **Returns**: Success/failure message (str)

## send_waypoints_sequential
- **Purpose**: Sends waypoints as sequential individual goals (fallback method).
- **Parameters**:
  - `waypoint_list`: List of waypoints in format [[x1, y1, yaw1], [x2, y2, yaw2], ...] (List[List[float]])
  - `frame_id`: Coordinate frame ID (str, default: "map")
  - `delay`: Delay between goals in seconds (float, default: 2.0)
- **Returns**: Success/failure message (str)

## cancel_waypoint_following
- **Purpose**: Cancels current waypoint following execution.
- **Returns**: Success/failure message (str)

## send_waypoints (Legacy)
- **Purpose**: Legacy waypoint sending method (may not work with all configurations).
- **Parameters**:
  - `waypoint_list`: List of waypoints in format [[x1, y1, yaw1], [x2, y2, yaw2], ...] (List[List[float]])
  - `frame_id`: Coordinate frame ID (str, default: "map")
- **Returns**: Success/failure message (str)

## start_waypoint_following (Legacy)
- **Purpose**: Legacy method to start waypoint following (may not work with all configurations).
- **Returns**: Success/failure message (str)

## stop_waypoint_following (Legacy)
- **Purpose**: Legacy method to stop waypoint following (may not work with all configurations).
- **Returns**: Success/failure message (str)

## pause_waypoint_following (Legacy)
- **Purpose**: Legacy method to pause waypoint following (may not work with all configurations).
- **Returns**: Success/failure message (str)

## resume_waypoint_following (Legacy)
- **Purpose**: Legacy method to resume waypoint following (may not work with all configurations).
- **Returns**: Success/failure message (str)

## send_and_start_waypoints (Legacy)
- **Purpose**: Legacy combined operation (may not work with all configurations).
- **Parameters**:
  - `waypoint_list`: List of waypoints in format [[x1, y1, yaw1], [x2, y2, yaw2], ...] (List[List[float]])
  - `frame_id`: Coordinate frame ID (str, default: "map")
- **Returns**: Success/failure message (str)

---

## Recommended Usage

For waypoint following, use these functions in order of preference:

1. **`send_waypoints_auto()`** - Automatically selects the best method for your system
2. **`send_waypoints_action()`** - For standard Nav2 configurations
3. **`send_waypoints_topic()`** - For custom configurations using topics
4. **`send_waypoints_sequential()`** - As a fallback when waypoint following is not available

If you encounter issues, see [WAYPOINT_TROUBLESHOOTING.md](WAYPOINT_TROUBLESHOOTING.md) for detailed troubleshooting steps.

---

## Position and Localization Functions

## get_robot_position
- **Purpose**: Gets the current robot position from available localization sources.
- **Parameters**:
  - `source`: Position data source (str, default: "auto")
    - "auto": Automatically selects the best available source (AMCL → Odometry)
    - "amcl": Uses AMCL localization data (/amcl_pose topic)
    - "odom": Uses odometry data (/odom topic)
  - `timeout`: Timeout in seconds (float, default: 5.0)
- **Returns**: Position information dictionary with status and coordinates
- **Example Return**:
  ```json
  {
    "source": "amcl",
    "position": {
      "x": 1.234, "y": 5.678, "z": 0.0,
      "yaw": 1.57, "yaw_degrees": 90.0,
      "quaternion": {"x": 0.0, "y": 0.0, "z": 0.707, "w": 0.707},
      "frame_id": "map"
    },
    "status": "success"
  }
  ```

## get_robot_velocity
- **Purpose**: Gets the current robot velocity from odometry.
- **Parameters**:
  - `timeout`: Timeout in seconds (float, default: 5.0)
- **Returns**: Velocity information dictionary
- **Example Return**:
  ```json
  {
    "velocity": {
      "linear": {"x": 0.5, "y": 0.0, "z": 0.0},
      "angular": {"x": 0.0, "y": 0.0, "z": 0.2}
    },
    "status": "success"
  }
  ```

## get_robot_odometry
- **Purpose**: Gets complete odometry information (position + velocity).
- **Parameters**:
  - `timeout`: Timeout in seconds (float, default: 5.0)
- **Returns**: Complete odometry data dictionary
- **Example Return**:
  ```json
  {
    "odometry": {
      "position": {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.5},
      "velocity": {
        "linear": {"x": 0.1, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.1}
      },
      "timestamp": {"sec": 1234567890, "nanosec": 123456789},
      "frame_id": "odom",
      "child_frame_id": "base_link"
    },
    "status": "success"
  }
  ```

## get_robot_amcl_pose
- **Purpose**: Gets AMCL localization result with covariance information.
- **Parameters**:
  - `timeout`: Timeout in seconds (float, default: 5.0)
- **Returns**: AMCL pose data dictionary with covariance
- **Example Return**:
  ```json
  {
    "amcl_pose": {
      "position": {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.5},
      "covariance": [0.1, 0.0, 0.0, ...],
      "timestamp": {"sec": 1234567890, "nanosec": 123456789},
      "frame_id": "map"
    },
    "status": "success"
  }
  ```

---

## Position Function Usage

For getting robot position, use these functions based on your needs:

1. **`get_robot_position()`** - General position query (automatically selects best source)
2. **`get_robot_position(source="amcl")`** - For map-based localization (more accurate)
3. **`get_robot_position(source="odom")`** - For odometry-based position (always available)
4. **`get_robot_velocity()`** - For current movement information
5. **`get_robot_odometry()`** - For complete motion state
6. **`get_robot_amcl_pose()`** - For detailed localization with uncertainty

**Note**: AMCL-based functions require AMCL localization to be running. Odometry-based functions work with basic robot setup.
