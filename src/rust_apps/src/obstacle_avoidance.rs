use anyhow::{Error, Result};
use geometry_msgs::msg::Twist;
use sensor_msgs::msg::LaserScan;
use std::{
    env,
    sync::{Arc, Mutex},
};

struct ObstacleAvoidance {
    _subscription: Arc<rclrs::Subscription<LaserScan>>,
    publication: Arc<rclrs::Publisher<Twist>>,
    twist_msg: Arc<Mutex<Twist>>,
}

impl ObstacleAvoidance {
    pub fn new(node: &rclrs::Node) -> Result<Self, rclrs::RclrsError> {
        let twist_msg = Arc::new(Mutex::new(Twist::default()));
        let publication = node.create_publisher::<Twist>("cmd_vel", rclrs::QOS_PROFILE_DEFAULT)?;
        let twist_msg_clone = Arc::clone(&twist_msg);
        let _subscription = node.create_subscription::<LaserScan, _>(
            "scan",
            rclrs::QOS_PROFILE_DEFAULT,
            move |msg: LaserScan| {
                let mut twist_msg = twist_msg_clone.lock().unwrap();

                let resolution = msg.ranges.len() / 360;
                let lateral_scan_angle = 30;
                let current_distance_left = msg.ranges[resolution * lateral_scan_angle];
                let current_distance_front = msg.ranges[msg.ranges.len() / 2];
                let current_distance_back = msg.ranges[0];
                let current_distance_right =
                    msg.ranges[msg.ranges.len() - resolution * lateral_scan_angle];

                println!(
                    "distance [m]: front '{:.2}', right '{:.2}', back '{:.2}', left '{:.2}'",
                    current_distance_front,
                    current_distance_right,
                    current_distance_back,
                    current_distance_left
                );

                twist_msg.linear.x = 0.5;
                twist_msg.linear.y = 0.0;
                twist_msg.angular.z = -0.15;
                if current_distance_front < 3.0 {
                    twist_msg.linear.x = -0.5;
                }
                if current_distance_right < 3.0 {
                    twist_msg.linear.y = -0.5;
                }
                if current_distance_back < 3.0 {
                    twist_msg.linear.x = 0.5;
                }
                if current_distance_left < 3.0 {
                    twist_msg.linear.x = 0.5;
                }
            },
        )?;

        Ok(Self {
            _subscription,
            publication,
            twist_msg,
        })
    }

    pub fn publish(&self) {
        let twist_msg = self.twist_msg.lock().unwrap();
        let _ = self.publication.publish(&*twist_msg);
    }
}

fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;
    let node = rclrs::create_node(&context, "minimal_subscriber_one")?;
    let subscriber_node_one = ObstacleAvoidance::new(&node)?;
    while context.ok() {
        subscriber_node_one.publish();
        let _ = rclrs::spin_once(node.clone(), Some(std::time::Duration::from_millis(500)));
        std::thread::sleep(std::time::Duration::from_millis(500));
    }
    Ok(())
}
