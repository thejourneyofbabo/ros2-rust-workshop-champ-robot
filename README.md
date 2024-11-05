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

## Understanding Rust and ROS2

### What is Cargo?
Cargo is Rust's package manager and build tool. It handles dependencies, project building, and testing. With Cargo, you can:
- Compile your code
- Download and update third-party packages (crates)
- Manage project configurations

### What are Crates?
Crates are Rust's package units. There are two types:
- Library Crates: Code that can be used by other crates
- Binary Crates: Code that can be compiled into an executable

Visit [Crates.io](https://crates.io) to explore available crates.

### ROS2 Package Structure
Essential files in a ROS2 Rust package:
- `src/` directory: Contains source files
- `Cargo.toml`: Defines dependencies and compiler configurations
- `Cargo.lock`: Contains exact dependency information (automatically maintained)
- `package.xml`: ROS2 package metadata and dependencies

## ROS2 Programming with Rust

### 1. Basic Rust Concepts

#### Functions
```rust
// Basic function syntax
fn function_name(variable: type) -> return_type {
    // Function body
}

// Main function with Result
fn main() -> Result<(), Error> {
    // Main function body
    Ok(())
}
```

#### Variables and Mutability
```rust
// Mutable variable
let mut message = std_msgs::msg::String::default();
```

### 2. ROS2 Node Creation Steps

#### a. Create Context
```rust
let context = rclrs::Context::new(env::args())?;
```

#### b. Create Node
```rust
// Function signature
pub fn create_node(
    context: &Context,
    node_name: &str
) -> Result<Arc<Node>, RclrsError>

// Usage
let node = rclrs::create_node(&context, "node_name")?;
```

#### c. Create Subscriber
```rust
// Function signature
pub fn create_subscription<T, Args>(
    &self,
    topic: &str,
    qos: QoSProfile,
    callback: impl SubscriptionCallback<T, Args>
) -> Result<Arc<Subscription<T>>, RclrsError>
where
    T: Message

// Example usage
let _subscription = node.create_subscription::<sensor_msgs::msg::LaserScan, _>(
    "scan",
    rclrs::QOS_PROFILE_DEFAULT,
    move |msg: sensor_msgs::msg::LaserScan| {
        println!("Angle min: '{}'", msg.angle_min);
    },
)?;
```

#### d. Create Publisher
```rust
// Function signature
pub fn create_publisher<T>(
    &self,
    topic: &str,
    qos: QoSProfile
) -> Result<Arc<Publisher<T>>, RclrsError>
where
    T: Message

// Example usage
let publisher = node.create_publisher::<Twist>("cmd_vel", rclrs::QOS_PROFILE_DEFAULT)?;
```

### 3. Node Execution Methods

#### Spin (Outside Loop)
```rust
rclrs::spin(node).map_err(|err| err.into())
```

#### Spin Once (Inside Loop)
```rust
rclrs::spin_once(node.clone(), Some(std::time::Duration::from_millis(500)));
```

### 4. Working with ROS2 Messages

#### Message Inspection
```bash
# View topic data
ros2 topic echo /scan

# Check message type
ros2 topic info /scan

# View message structure
ros2 interface show sensor_msgs/msg/LaserScan
```

#### Quality of Service (QoS) Profiles
Available profiles:
```rust
QOS_PROFILE_CLOCK
QOS_PROFILE_DEFAULT
QOS_PROFILE_PARAMETERS
QOS_PROFILE_PARAMETER_EVENTS
QOS_PROFILE_SENSOR_DATA
QOS_PROFILE_SERVICES_DEFAULT
QOS_PROFILE_SYSTEM_DEFAULT
```

## Implementation Examples

### 1. LaserScan Subscriber
```rust
use std::env;
use anyhow::{Error, Result};

fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;
    let node = rclrs::create_node(&context, "scan_subscriber")?;
    let mut num_messages: usize = 0;

    let _subscription = node.create_subscription::<sensor_msgs::msg::LaserScan, _>(
        "scan",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: sensor_msgs::msg::LaserScan| {
            num_messages += 1;
            println!("Back range[m]: '{:.2}'", msg.ranges[0]);
            println!("Ranges size: '{}'", msg.ranges.len());
            println!("Angle min: '{}'", msg.angle_min);
            println!("Angle max: '{}'", msg.angle_max);
            println!("(Got {} messages so far)", num_messages);
        },
    )?;

    rclrs::spin(node).map_err(|err| err.into())
}
```

### 2. Cmd Vel Publisher
```rust
use std::env;
use anyhow::{Error, Result};
use geometry_msgs::msg::Twist as Twist;

fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;
    let node = rclrs::create_node(&context, "cmd_vel_publisher")?;
    let publisher = node.create_publisher::<Twist>("cmd_vel", rclrs::QOS_PROFILE_DEFAULT)?;
    
    let mut cmd_vel_message = Twist::default();
    let mut velocity = 1.0;
    let velocity_threshold = 1.0;
    let velocity_decrease = 0.05;

    while context.ok() {
        cmd_vel_message.linear.x = velocity;
        cmd_vel_message.linear.y = velocity;
        cmd_vel_message.angular.z = 0.0;
        if velocity < velocity_threshold*(-1.0) {velocity = velocity_threshold}
        else {velocity-=velocity_decrease};
        println!("Moving velocity lineal x: {:.2} and angular z: {:.2} m/s.",
                cmd_vel_message.linear.x, cmd_vel_message.angular.z);
        publisher.publish(&cmd_vel_message)?;
        std::thread::sleep(std::time::Duration::from_millis(500));
    }
    Ok(())
}
```

## Building and Running

1. Build the package:
```bash
cd ~/ros2_rust_workshop/ros_ws
colcon build --packages-select rust_apps
source install/setup.bash
```

2. Run the examples:
```bash
# Launch simulation
ros2 launch champ_config gazebo.launch.py

# Run subscriber
ros2 run rust_apps scan_subscriber_node

# Run publisher
ros2 run rust_apps cmd_vel_publisher_node

# Run obstacle avoidance
ros2 run rust_apps obstacle_avoidance_node
```

3. Stop robot:
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
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

## Resources

### ROS2 Rust Resources
- [ROS2 Rust Repository](https://github.com/ros2-rust/ros2_rust)
- [ROS2 Basics in 3 Days (Rust)](https://app.theconstruct.ai/courses/168)
- [ROS2 with Rust | ROS2 Developers Open Class](https://youtu.be/ShCnUasOBzU?feature=shared)
- [QoS Documentation](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html)

### Rust Learning Resources
- [The Rust Programming Language Book](https://doc.rust-lang.org/book/)
- [Best Rust Programming Courses for 2024](https://medium.com/javarevisited/7-best-rust-programming-courses-and-books-for-beginners-in-2021-2ed2311af46c)

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## References

This workshop is adapted from the tutorial and materials available at: https://github.com/roboticswithjulia/ros2_rust_workshop
