#[macro_use]
extern crate rosrust_codegen;

rosmsg_main!(
    "sensor_msgs/JointState",
    "control_msgs/FollowJointTrajectoryActionGoal"
);