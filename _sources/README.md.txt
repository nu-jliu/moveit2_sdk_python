# MoveIt2 SDK Python

A Python SDK for MoveIt2 that provides a simplified interface for robotic manipulation tasks, specifically designed for the Franka Emika Panda robot.

## Features

- **Simplified MoveIt2 Interface**: Easy-to-use Python API for motion planning and execution
- **Cartesian Path Planning**: Compute and execute Cartesian paths through waypoints
- **Joint Space Control**: Move robot to specific joint configurations
- **Franka Integration**: Specialized support for Franka Emika Panda robot with gripper control
- **Speech Feedback**: Audio feedback for robot actions using Google Speech
- **Async/Await Support**: Modern Python async programming for non-blocking operations

## Installation

### Prerequisites

- ROS 2 (Humble or later)
- MoveIt2
- Python 3.8+
- Franka ROS 2 packages (for Franka robot support)

### Dependencies

Install the required Python packages:

```bash
pip install rclpy geometry_msgs moveit_msgs sensor_msgs std_srvs franka_msgs google_speech
```

### Building from Source

1. Clone the repository into your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository_url> moveit2_sdk_python
```

2. Install dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:
```bash
colcon build --packages-select moveit2_sdk_python
```

4. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

## Usage

### Basic MoveIt2 API

```python
import asyncio
from moveit2_sdk_python.moveit2_sdk_python import Moveit2Python
from geometry_msgs.msg import Pose, Point, Quaternion

# Initialize the API
api = Moveit2Python(
    base_frame="panda_link0",
    ee_frame="panda_hand_tcp", 
    group_name="panda_manipulator"
)

async def main():
    # Move to joint configuration
    joint_names = [f"panda_joint{i+1}" for i in range(7)]
    joint_positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    await api.move_group(joint_names, joint_positions)
    
    # Plan and execute Cartesian path
    waypoints = [
        Pose(
            position=Point(x=0.4, y=0.2, z=0.4),
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        )
    ]
    trajectory = await api.get_cartesian_path(waypoints)
    await api.execute_trajectory(trajectory)

# Run the async function
asyncio.run(main())
```

### Franka Mover (High-level Interface)

```python
import asyncio
from moveit2_sdk_python.franka_mover import FrankaMover

async def main():
    # Initialize Franka mover
    mover = FrankaMover()
    
    # Move to home position
    await mover.home()
    
    # Move to a specific position
    await mover.move(x=0.5, y=0.0, z=0.3)
    
    # Execute grasp
    await mover.grasp()
    
    # Pour motion
    await mover.pour(x=0.4, y=0.2, z=0.4)

asyncio.run(main())
```

### Running the Test Node

The package includes a test node that demonstrates various functionalities:

```bash
# Launch the test node
ros2 run moveit2_sdk_python try_node

# Trigger the test sequence
ros2 service call /test_cartesian std_srvs/srv/Empty
```

## API Reference

### Moveit2Python Class

#### Methods

- `move_group(names, positions)`: Move robot to specified joint configuration
- `get_cartesian_path(waypoints)`: Compute Cartesian path through waypoints  
- `execute_trajectory(trajectory)`: Execute a pre-computed trajectory

### FrankaMover Class

#### Methods

- `home()`: Move robot to home position
- `move(x, y, z)`: Move to Cartesian coordinates with approach motion
- `grasp()`: Execute grasp action with Franka gripper
- `pour(x, y, z)`: Execute pouring motion at specified location

## Configuration

### Robot Parameters

The SDK can be configured for different robot setups by modifying the initialization parameters:

```python
api = Moveit2Python(
    base_frame="your_base_frame",      # Robot base frame
    ee_frame="your_ee_frame",          # End-effector frame  
    group_name="your_move_group"       # MoveIt planning group
)
```

### Motion Parameters

Cartesian path planning parameters can be adjusted in the `get_cartesian_path` method:

- `max_step`: Maximum step size for path interpolation (default: 0.01)
- `max_cartesian_speed`: Maximum Cartesian velocity (default: 0.08)
- `max_velocity_scaling_factor`: Velocity scaling (default: 0.08)
- `max_acceleration_scaling_factor`: Acceleration scaling (default: 0.03)

## Documentation

Full API documentation is available at: [GitHub Pages](https://your-username.github.io/moveit2_sdk_python)

To build documentation locally:

```bash
pip install -r docs-requirements.txt
sphinx-build -b html docs docs/_build/html
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

This project is licensed under the Apache License 2.0 - see the LICENSE file for details.

## Troubleshooting

### Common Issues

1. **Import errors**: Ensure all ROS 2 and MoveIt2 packages are properly installed and sourced
2. **Service timeouts**: Check that MoveIt2 move_group node is running
3. **Planning failures**: Verify robot URDF and SRDF are correctly configured

### Getting Help

- Check the [documentation](https://your-username.github.io/moveit2_sdk_python)
- Open an issue on GitHub
- Consult MoveIt2 documentation for underlying motion planning concepts
