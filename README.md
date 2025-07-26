# Fastbot Racing Contest - ROS2 Autonomous Racing Solution

## Overview

This project was developed for **Online ROS 2 Robot Racing Contest**, where the goal is to make a yellow Fastbot robot complete a full lap of a racing circuit as fast as possible using ROS2. The solution uses waypoint-based navigation with a pure pursuit controller to achieve autonomous racing.
The key insight for this solution was leveraging the **`/fastbot_1/odom` topic as ground truth positioning**.

## Contest Information

- **Objective**: Make the Fastbot robot complete a full lap autonomously as fast as possible
- **Platform**: The Construct virtual machines with ROS2
- **More info**: [OpenRobotics Discourse announcement](https://discourse.openrobotics.org/t/ros-2-online-robot-racing-contest-fun-challenge-await-this-july/45100)

## Project Structure

This project consists of four main ROS2 packages and the saved map directory:

```
construct-racing-contest/
├── fastbot_racing/          # Main launch package
├── racetrack_controller/    # Pure pursuit controller for following waypoints
├── waypoint_manager/        # Waypoint management and path planning
├── waypoint_msgs/           # Custom message definitions
└── map/                     # Pre-recorded waypoints
```

## Installation and Setup

### Prerequisites

- ROS2 (Humble)
- The Construct simulation environment with Fastbot
- Gazebo simulation running

### Installation Steps

**Build the workspace:**
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

## Usage

### Quick Start (Competition Mode)

**Important**: Make sure the Gazebo simulation is running before starting the racing program. Start the system from a sensible location.

```bash
ros2 launch fastbot_racing start_racing.launch.py
```

This command launches the complete racing system and the robot will start following the pre-recorded waypoints autonomously.

### Development Mode

For development and debugging, you can launch with RViz visualization:

```bash
ros2 launch fastbot_racing start_racing.launch.py dev:=true
```

This will also open RViz2 with a pre-configured view showing:
- Robot position and orientation
- Planned waypoint path
- Sensor data visualization

## System Architecture

### Components

1. **Waypoint Manager** (`waypoint_manager`)
   - Loads waypoints from YAML file
   - Provides waypoint management services
   - Publishes navigation path
   - Handles waypoint interpolation using [Catmull-Rom splines](https://en.wikipedia.org/wiki/Centripetal_Catmull%E2%80%93Rom_spline)

2. **Racetrack Controller** (`racetrack_controller`)
   - Implements pure pursuit algorithm
   - Subscribes to robot odometry
   - Follows waypoint path
   - Publishes velocity commands to `/fastbot_1/cmd_vel`

3. **Waypoint Messages** (`waypoint_msgs`)
   - Custom message and service definitions
   - Waypoint data structures
   - Service interfaces for waypoint management

### Available Robot Topics

The Fastbot robot provides the following topics:
- `/fastbot_1/cmd_vel` - Velocity commands (Twist)
- `/fastbot_1/odom` - Odometry data
- `/fastbot_1/scan` - LiDAR data
- `/fastbot_1/camera/image_raw` - Camera feed
- `/fastbot_1/joint_states` - Joint states

## Waypoint Management

### Creating/Editing Waypoints

During development, you can manually create waypoints using the waypoint management services:

1. **Start the system in development mode:**
   ```bash
   ros2 launch fastbot_racing start_racing.launch.py dev:=true
   ```

2. **Pause the controller to prevent movement:**
   ```bash
   ros2 service call /toggle_pause std_srvs/srv/Trigger
   ```

3. **Use teleop to drive the robot to desired positions:**
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/fastbot_1/cmd_vel
   ```

4. **Add waypoints at current position:**
   ```bash
   ros2 service call /waypoint/append waypoint_msgs/srv/AppendWaypoint
   ```

5. **Save waypoints to file:**
   ```bash
   ros2 service call /waypoint/save waypoint_msgs/srv/SaveWaypoints
   ```

### Available Waypoint Services

- `/waypoint/append` - Add waypoint at current robot position
- `/waypoint/delete` - Delete waypoint by index
- `/waypoint/insert` - Insert waypoint at specific index
- `/waypoint/overwrite` - Overwrite waypoint at index
- `/waypoint/save` - Save current waypoints to file

## Configuration Parameters

### Racetrack Controller Parameters

- `lookahead_distance` (default: 1.5) - Pure pursuit lookahead distance
- `max_linear_speed` (default: 2.5) - Maximum linear velocity
- `max_angular_speed` (default: 2.0) - Maximum angular velocity
- `start_paused` (default: false) - Start controller in paused state

### Tuning Performance

To optimize lap times, you can adjust:
1. **Lookahead distance**: Smaller values for tighter following, larger for smoother turns
2. **Speed limits**: Increase for faster laps (be careful with stability)
3. **Waypoint density**: More waypoints for precision, fewer for speed

## Development Tools

### Visualization

- **RViz2**: `rviz2 -d ~/ros2_ws/src/construct-racing-contest/fastbot_racing/fastbot_racing/config/developer_conf.rviz`
- **Robot teleoperation**: `ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/fastbot_1/cmd_vel`

## Contest Strategy

This solution uses a **waypoint-based pure pursuit approach**:

1. **Pre-recorded path**: Waypoints recorded along the fastest assumed racing line
2. **Pure pursuit controller**: Smooth path following with configurable lookahead
3. **Speed optimization**: Tuned parameters for maximum safe speed
4. **Robust navigation**: Uses odometry for reliable positioning

### Performance

The robot completes the track in approximately **~80 seconds** with the current configuration and waypoint path.

### Path Optimization Limitations

**Important Note**: The current implementation uses a **hand-recorded path** which is likely following the "slow line" rather than the optimal racing line. For maximum performance, the waypoints should be optimized using raceline optimization techniques.

**For advanced raceline optimization**, refer to: [F1Tenth - L22 Raceline Optimization](https://youtu.be/Pf4p3bR7AAQ)
