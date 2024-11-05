use anyhow::{Error, Result};
use geometry_msgs::msg::Twist;
use std::env;

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
        if velocity < velocity_threshold * (-1.0) {
            velocity = velocity_threshold
        } else {
            velocity -= velocity_decrease
        };
        println!(
            "Moving velocity lineal x: {:.2} and angular z: {:.2} m/s.",
            cmd_vel_message.linear.x, cmd_vel_message.angular.z
        );
        publisher.publish(&cmd_vel_message)?;
        std::thread::sleep(std::time::Duration::from_millis(500));
    }
    Ok(())
}
