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

// Main function with Result type for error handling
fn main() -> Result<(), Error> {
    // Main function body
    Ok(())
}
```

#### Variables and Mutability
```rust
// Mutable variable declaration and initialization
let mut message = std_msgs::msg::String::default();
```

### 2. Creating ROS2 Nodes

#### Basic Node Creation
```rust
// Initialize ROS context with command line arguments
let context = rclrs::Context::new(env::args())?;

// Create a new node with specified name
let node = rclrs::create_node(&context, "node_name")?;
```

### 3. Example Implementations

#### 1. LaserScan Subscriber Node
```rust
use std::env;
use anyhow::{Error, Result};

fn main() -> Result<(), Error> {
    // Initialize ROS context
    let context = rclrs::Context::new(env::args())?;
    // Create node named "scan_subscriber"
    let node = rclrs::create_node(&context, "scan_subscriber")?;
    // Message counter
    let mut num_messages: usize = 0;

    // Create subscription to laser scan topic
    let _subscription = node.create_subscription::<sensor_msgs::msg::LaserScan, _>(
        "scan",                          // Topic name
        rclrs::QOS_PROFILE_DEFAULT,      // Quality of Service profile
        move |msg: sensor_msgs::msg::LaserScan| {
            num_messages += 1;
            // Print laser scan data
            println!("Back range[m]: '{:.2}'", msg.ranges[0]);
            println!("Ranges size: '{}'", msg.ranges.len());
            println!("Angle min: '{}'", msg.angle_min);
            println!("Angle max: '{}'", msg.angle_max);
            println!("(Got {} messages so far)", num_messages);
        },
    )?;

    // Spin the node
    rclrs::spin(node).map_err(|err| err.into())
}
```

#### 2. Cmd Vel Publisher Node
```rust
use std::env;
use anyhow::{Error, Result};
use geometry_msgs::msg::Twist as Twist;

fn main() -> Result<(), Error> {
    // Initialize ROS context
    let context = rclrs::Context::new(env::args())?;
    // Create node named "cmd_vel_publisher"
    let node = rclrs::create_node(&context, "cmd_vel_publisher")?;
    // Create publisher for velocity commands
    let publisher = node.create_publisher::<Twist>("cmd_vel", rclrs::QOS_PROFILE_DEFAULT)?;
    
    // Initialize velocity message
    let mut cmd_vel_message = Twist::default();
    let mut velocity = 1.0;
    let velocity_threshold = 1.0;
    let velocity_decrease = 0.05;

    // Main loop
    while context.ok() {
        // Set velocity commands
        cmd_vel_message.linear.x = velocity;
        cmd_vel_message.linear.y = velocity;
        cmd_vel_message.angular.z = 0.0;
        
        // Update velocity
        if velocity < velocity_threshold*(-1.0) {
            velocity = velocity_threshold
        } else {
            velocity -= velocity_decrease
        };
        
        // Print current velocity
        println!("Moving velocity lineal x: {:.2} and angular z: {:.2} m/s.",
                cmd_vel_message.linear.x, cmd_vel_message.angular.z);
        
        // Publish velocity command
        publisher.publish(&cmd_vel_message)?;
        // Sleep for rate control
        std::thread::sleep(std::time::Duration::from_millis(500));
    }
    Ok(())
}
```

#### 3. Combined Subscriber-Publisher (Obstacle Avoidance) Node
```rust 
// Import error handling traits from anyhow crate
use anyhow::{Error, Result};
// Import ROS2 message types for robot velocity commands and laser scan data
use geometry_msgs::msg::Twist;  // For robot movement commands
use sensor_msgs::msg::LaserScan;  // For laser scanner data
// Import standard library components for threading and environment variables
use std::{
    env,
    sync::{Arc, Mutex},  // Thread-safe shared memory primitives
};

/// Main structure for obstacle avoidance functionality
/// Uses ROS2 subscription for laser data and publishes velocity commands
struct ObstacleAvoidance {
    _subscription: Arc<rclrs::Subscription<LaserScan>>,  // Subscriber for laser scan data
    publication: Arc<rclrs::Publisher<Twist>>,          // Publisher for velocity commands
    twist_msg: Arc<Mutex<Twist>>,                       // Thread-safe storage for current velocity command
}

impl ObstacleAvoidance {
    /// Creates a new ObstacleAvoidance instance
    /// Sets up ROS2 publisher and subscriber with appropriate callbacks
    pub fn new(node: &rclrs::Node) -> Result<Self, rclrs::RclrsError> {
        // Initialize default velocity command message with thread-safe access
        let twist_msg = Arc::new(Mutex::new(Twist::default()));
        
        // Create publisher for velocity commands on "cmd_vel" topic
        let publication = node.create_publisher::<Twist>("cmd_vel", rclrs::QOS_PROFILE_DEFAULT)?;
        
        // Clone twist message for use in callback
        let twist_msg_clone = Arc::clone(&twist_msg);
        
        // Create subscription to laser scan data
        let _subscription = node.create_subscription::<LaserScan, _>(
            "scan",                    // Topic name
            rclrs::QOS_PROFILE_DEFAULT,
            move |msg: LaserScan| {    // Callback closure for processing laser scan data
                // Get mutable access to velocity command
                let mut twist_msg = twist_msg_clone.lock().unwrap();
                
                // Calculate angular resolution of laser scan
                let resolution = msg.ranges.len() / 360;  // Points per degree
                
                // Define scan angle for lateral (side) measurements
                let lateral_scan_angle = 30;  // Degrees
                
                // Extract distances in key directions
                let current_distance_left = msg.ranges[resolution * lateral_scan_angle];
                let current_distance_front = msg.ranges[msg.ranges.len() / 2];
                let current_distance_back = msg.ranges[0];
                let current_distance_right =
                    msg.ranges[msg.ranges.len() - resolution * lateral_scan_angle];
                
                // Print current distances for debugging
                println!(
                    "distance [m]: front '{:.2}', right '{:.2}', back '{:.2}', left '{:.2}'",
                    current_distance_front,
                    current_distance_right,
                    current_distance_back,
                    current_distance_left
                );
                
                // Set default movement (forward and slight left turn)
                twist_msg.linear.x = 0.5;    // Forward velocity
                twist_msg.linear.y = 0.0;    // No lateral movement
                twist_msg.angular.z = -0.15;  // Angular velocity (turning)
                
                // Obstacle avoidance logic
                // Adjust velocity based on detected obstacles
                if current_distance_front < 3.0 {
                    twist_msg.linear.x = -0.5;  // Reverse if obstacle ahead
                }
                if current_distance_right < 3.0 {
                    twist_msg.linear.y = -0.5;  // Move left if obstacle on right
                }
                if current_distance_back < 3.0 {
                    twist_msg.linear.x = 0.5;   // Move forward if obstacle behind
                }
                if current_distance_left < 3.0 {
                    twist_msg.linear.x = 0.5;   // Move forward if obstacle on left
                }
            },
        )?;
        
        // Return constructed ObstacleAvoidance instance
        Ok(Self {
            _subscription,
            publication,
            twist_msg,
        })
    }
    
    /// Publishes current velocity command to ROS2 network
    pub fn publish(&self) {
        let twist_msg = self.twist_msg.lock().unwrap();
        self.publication.publish(&*twist_msg);
    }
}

/// Main function to initialize and run the ROS2 node
fn main() -> Result<(), Error> {
    // Initialize ROS2 context with command line arguments
    let context = rclrs::Context::new(env::args())?;
    
    // Create new ROS2 node named "minimal_subscriber_one"
    let node = rclrs::create_node(&context, "minimal_subscriber_one")?;
    
    // Create obstacle avoidance instance
    let subscriber_node_one = ObstacleAvoidance::new(&node)?;
    
    // Main loop - runs while ROS2 context is valid
    while context.ok() {
        // Publish current velocity command
        subscriber_node_one.publish();
        
        // Process callbacks once with timeout
        rclrs::spin_once(node.clone(), Some(std::time::Duration::from_millis(500)));
        
        // Sleep to control update rate
        std::thread::sleep(std::time::Duration::from_millis(500));
    }
    
    Ok(())
}
```

## Package Configuration

### package.xml
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

### Cargo.toml
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

3. Stop the robot:
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
