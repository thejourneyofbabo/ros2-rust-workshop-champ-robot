use anyhow::{Error, Result};
use std::env;

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
