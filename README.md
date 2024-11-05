# ROS2 Rust Workshop

This repository contains the materials for a workshop on developing real robotic applications using ROS2 and Rust. The workshop demonstrates how to create ROS2 nodes in Rust for robotics applications.

## Project Overview

This workshop teaches:
- Introduction to Rust and its benefits for robotics
- How to install Rust with ROS2
- How to run simulations
- How to move a robot using ROS2 and Rust
- Creating ROS2 packages in Rust
- Basic Rust programming tips
- Creating subscribers and publishers in Rust

## Prerequisites

- Ubuntu (tested on Ubuntu 22.04)
- ROS2 Humble
- Rust
- Docker (optional)

## Installation

1. Install ROS2 Humble following the [official instructions](https://docs.ros.org/en/humble/Installation.html)

2. Install Rust:
```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

## Building the Project

1. Source ROS2:
```bash
source /opt/ros/humble/setup.bash
```

2. Build the workspace:
```bash
cd ros2_rust_workshop/ros_ws
colcon build
```

3. Source the workspace:
```bash
source install/setup.bash
```

## Developing Rust Code for ROS2

### Creating a New ROS2 Rust Package

1. Create a new Rust package:
```bash
cd ~/ros2_rust_workshop/ros_ws/src
cargo new rust_apps
```

2. Create a package.xml file in the rust_apps directory:
```xml
<?xml version="1.0"?>
<package format="3">
  <name>rust_apps</name>
  <version>0.0.0</version>
  <description>ROS2 Rust main package</description>
  <maintainer email="user@gmail.com">user</maintainer>
  <license>MIT</license>

  <depend>rclrs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>

  <export>
    <build_type>ament_cargo</build_type>
  </export>
</package>
```

3. Configure Cargo.toml:
```toml
[package]
name = "rust_apps"
version = "0.1.0"
edition = "2021"

[dependencies]
rclrs = "*"
anyhow = "1.0"
sensor_msgs = "*"
geometry_msgs = "*"

[[bin]]
name = "scan_subscriber_node"
path = "src/scan_subscriber.rs"

[[bin]]
name = "cmd_vel_publisher_node"
path = "src/cmd_vel_publisher.rs"

[[bin]]
name = "obstacle_avoidance_node"
path = "src/obstacle_avoidance.rs"
```

### Creating ROS2 Nodes in Rust

#### Basic Node Structure
```rust
use std::env;
use anyhow::{Error, Result};

fn main() -> Result<(), Error> {
    // Initialize ROS context
    let context = rclrs::Context::new(env::args())?;
    
    // Create node
    let node = rclrs::create_node(&context, "node_name")?;
    
    // Node implementation here
    
    Ok(())
}
```

#### Creating a Publisher
```rust
let publisher = node.create_publisher::<Twist>("cmd_vel", rclrs::QOS_PROFILE_DEFAULT)?;
publisher.publish(&message)?;
```

#### Creating a Subscriber
```rust
let _subscription = node.create_subscription::<sensor_msgs::msg::LaserScan, _>(
    "topic_name",
    rclrs::QOS_PROFILE_DEFAULT,
    move |msg: sensor_msgs::msg::LaserScan| {
        // Handle message here
    },
)?;
```

### Building and Testing

1. Add new dependencies:
```bash
cargo add anyhow
cargo add rclrs
```

2. Build specific package:
```bash
colcon build --packages-select rust_apps
```

3. Test your node:
```bash
source install/setup.sh
ros2 run rust_apps <node_name>
```

## Running the Examples

### 1. Launch the Simulation

In Terminal 1:
```bash
cd ~/ros2_rust_workshop/ros_ws
. install/setup.sh
ros2 launch champ_config gazebo.launch.py
```

### 2. Run the Scan Subscriber

In Terminal 2:
```bash
cd ~/ros2_rust_workshop/ros_ws
source install/setup.sh
ros2 run rust_apps scan_subscriber_node
```

### 3. Run the Cmd Vel Publisher

In Terminal 2:
```bash
cd ~/ros2_rust_workshop/ros_ws
source install/setup.sh
ros2 run rust_apps cmd_vel_publisher_node
```

### 4. Run the Obstacle Avoidance Node

In Terminal 2:
```bash
cd ~/ros2_rust_workshop/ros_ws
source install/setup.sh
ros2 run rust_apps obstacle_avoidance_node
```

To stop the robot movement:
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

## Project Structure

```
ros2_rust_workshop/
├── ros_ws/
│   ├── src/
│   │   └── rust_apps/
│   │       ├── src/
│   │       │   ├── scan_subscriber.rs
│   │       │   ├── cmd_vel_publisher.rs
│   │       │   └── obstacle_avoidance.rs
│   │       ├── Cargo.toml
│   │       └── package.xml
│   └── ...
└── ...
```

## Common ROS2 Commands for Development

```bash
# List all topics
ros2 topic list

# Echo topic data
ros2 topic echo /topic_name

# Get topic info
ros2 topic info /topic_name

# Get message structure
ros2 interface show geometry_msgs/msg/Twist
```

## Resources

- [ROS2 Rust Repository](https://github.com/ros2-rust/ros2_rust)
- [Rust Programming Language Book](https://doc.rust-lang.org/book/)
- [ROS2 Documentation](https://docs.ros.org/en/humble/index.html)

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## References

This workshop is adapted from the tutorial and materials available at: https://github.com/roboticswithjulia/ros2_rust_workshop
