/*
Copyright 2017 Takashi Ogura

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
extern crate env_logger;
extern crate gear;
extern crate glfw;
extern crate k;
extern crate nalgebra as na;
extern crate ncollide;
extern crate urdf_viz;
extern crate urdf_rs;
//#[macro_use]
extern crate rosrust;
#[macro_use]
extern crate rosrust_codegen;

use std::time::{Duration, Instant};
use std::sync::mpsc;
use k::JointContainer;
use glfw::{Action, Key, WindowEvent};
use ncollide::shape::Compound3;

rosmsg_include!();

struct CollisionAvoidApp<I>
where
    I: gear::InverseKinematicsSolver<f64>,
{
    planner: gear::JointPathPlannerWithIK<I>,
    obstacles: Compound3<f64>,
    ik_target_pose: na::Isometry3<f64>,
    colliding_link_names: Vec<String>,
    viewer: urdf_viz::Viewer,
    arm: k::LinkChain<f64>,
}

impl<I> CollisionAvoidApp<I>
where
    I: gear::InverseKinematicsSolver<f64>,
{
    fn new(planner: gear::JointPathPlannerWithIK<I>, end_link_name: &str) -> Self {
        let mut viewer = urdf_viz::Viewer::new("gear: example reach");
        viewer.add_robot(planner.urdf_robot().as_ref().unwrap());
        viewer.add_axis_cylinders("origin", 1.0);

        let urdf_obstacles = urdf_rs::utils::read_urdf_or_xacro("obstacles.urdf")
            .expect("obstacle file not found");
        let obstacles = gear::create_compound_from_urdf_robot(&urdf_obstacles);
        viewer.add_robot(&urdf_obstacles);

        let ik_target_pose = na::Isometry3::from_parts(
            na::Translation3::new(0.40, 0.20, 0.3),
            na::UnitQuaternion::from_euler_angles(0.0, 1.57, 1.57),
        );
        let arm = planner.create_arm(end_link_name).unwrap();
        viewer.add_axis_cylinders("ik_target", 0.3);
        CollisionAvoidApp {
            viewer,
            obstacles,
            ik_target_pose,
            colliding_link_names: Vec::new(),
            planner,
            arm,
        }
    }
    fn update_robot(&mut self) {
        self.viewer.update(&self.planner);
    }
    fn update_ik_target(&mut self) {
        if let Some(obj_vec) = self.viewer.scenes.get_mut("ik_target") {
            for obj in obj_vec {
                obj.0.set_local_transformation(
                    na::convert(self.ik_target_pose),
                )
            }
        }
    }
    fn reset_colliding_link_colors(&mut self) {
        for link in &self.colliding_link_names {
            self.viewer.reset_temporal_color(link);
        }
    }
    fn run(&mut self) {
        let mut is_collide_show = false;
        self.update_robot();
        self.update_ik_target();

        rosrust::init("gear");
        let (tx, rx) = mpsc::channel();
        let mut trajectory_pub = rosrust::publish("/arm_controller/follow_joint_trajectory/goal")
            .unwrap();
        //let mut trajectory_pub = rosrust::publish("/arm_controller/command").unwrap();
        let _joint_subscriber = rosrust::subscribe(
            "joint_states",
            move |s: msg::sensor_msgs::JointState| { tx.send(s).unwrap(); },
        ).unwrap();
        let mut plans: Vec<Vec<f64>> = Vec::new();
        while self.viewer.render() && rosrust::is_ok() {
            if plans.is_empty() {
                if let Ok(msg) = rx.recv_timeout(Duration::from_millis(10)) {
                    let mut angles = self.arm.get_joint_angles();
                    for (i, name) in self.arm.get_joint_names().into_iter().enumerate() {
                        let msg_names = msg.name.clone();
                        if let Some(pos) = msg_names.into_iter().position(|n| n == name) {
                            angles[i] = msg.position[pos];
                        }
                    }
                    self.arm.set_joint_angles(&angles).unwrap();
                    self.update_robot();
                }
            } else {
                self.arm.set_joint_angles(&plans.pop().unwrap()).unwrap();
                self.update_robot();
            }
            for event in self.viewer.events().iter() {
                match event.value {
                    WindowEvent::Key(code, _, Action::Press, mods) => {
                        match code {
                            Key::Up => {
                                if mods.contains(glfw::modifiers::Shift) {
                                    self.ik_target_pose.rotation *=
                                        na::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.2);
                                } else {
                                    self.ik_target_pose.translation.vector[2] += 0.05;
                                }
                                self.update_ik_target();
                            }
                            Key::Down => {
                                if mods.contains(glfw::modifiers::Shift) {
                                    self.ik_target_pose.rotation *=
                                        na::UnitQuaternion::from_euler_angles(0.0, 0.0, -0.2);
                                } else {
                                    self.ik_target_pose.translation.vector[2] -= 0.05;
                                }
                                self.update_ik_target();
                            }
                            Key::Left => {
                                if mods.contains(glfw::modifiers::Shift) {
                                    self.ik_target_pose.rotation *=
                                        na::UnitQuaternion::from_euler_angles(0.0, 0.2, -0.0);
                                } else {
                                    self.ik_target_pose.translation.vector[1] += 0.05;
                                }
                                self.update_ik_target();
                            }
                            Key::Right => {
                                if mods.contains(glfw::modifiers::Shift) {
                                    self.ik_target_pose.rotation *=
                                        na::UnitQuaternion::from_euler_angles(0.0, -0.2, 0.0);
                                } else {
                                    self.ik_target_pose.translation.vector[1] -= 0.05;
                                }
                                self.update_ik_target();
                            }
                            Key::B => {
                                if mods.contains(glfw::modifiers::Shift) {
                                    self.ik_target_pose.rotation *=
                                        na::UnitQuaternion::from_euler_angles(-0.2, 0.0, 0.0);
                                } else {
                                    self.ik_target_pose.translation.vector[0] -= 0.05;
                                }
                                self.update_ik_target();
                            }
                            Key::F => {
                                if mods.contains(glfw::modifiers::Shift) {
                                    self.ik_target_pose.rotation *=
                                        na::UnitQuaternion::from_euler_angles(0.2, 0.0, 0.0);
                                } else {
                                    self.ik_target_pose.translation.vector[0] += 0.05;
                                }
                                self.update_ik_target();
                            }
                            Key::I => {
                                self.reset_colliding_link_colors();
                                let result =
                                    self.planner.solve_ik(&mut self.arm, &self.ik_target_pose);
                                if result.is_ok() {
                                    self.update_robot();
                                } else {
                                    println!("fail!!");
                                }
                            }
                            Key::G => {
                                self.reset_colliding_link_colors();
                                match self.planner.plan_with_ik(
                                    &mut self.arm,
                                    &self.ik_target_pose,
                                    &self.obstacles,
                                ) {
                                    Ok(plan) => {
                                        let trajectory_points = gear::interpolate(&plan, 5.0, 0.1)
                                            .unwrap();
                                        let mut plan_for_viewer = plan.clone();
                                        plan_for_viewer.reverse();
                                        plans = gear::interpolate(&plan_for_viewer, 5.0, 0.1)
                                            .unwrap()
                                            .into_iter()
                                            .map(|point| point.position)
                                            .collect();
                                        let mut msg = msg::control_msgs::FollowJointTrajectoryActionGoal::default();
                                        msg.goal_id.id = format!("gear_ros{:?}", Instant::now());
                                        let mut traj_msg =
                                            msg::trajectory_msgs::JointTrajectory::default();
                                        traj_msg.joint_names = self.arm.get_joint_names();
                                        for (i, trajectory_point) in
                                            trajectory_points.into_iter().enumerate()
                                        {
                                            let mut tp = msg::trajectory_msgs::JointTrajectoryPoint::default();
                                            tp.positions = trajectory_point.position;
                                            tp.velocities = trajectory_point.velocity;
                                            tp.accelerations = trajectory_point.acceleration;

                                            tp.time_from_start = rosrust::Duration::from_nanos(
                                                ((i + 1) * 100000000) as i64,
                                            );
                                            traj_msg.points.push(tp);
                                        }
                                        msg.goal.trajectory = traj_msg;
                                        trajectory_pub.send(msg).unwrap();
                                    }
                                    Err(error) => {
                                        println!("failed to reach!! {}", error);
                                    }
                                };
                            }
                            Key::R => {
                                self.reset_colliding_link_colors();
                                gear::set_random_joint_angles(&mut self.arm).unwrap();
                                self.update_robot();
                            }
                            Key::C => {
                                self.reset_colliding_link_colors();
                                self.colliding_link_names =
                                    self.planner.get_colliding_link_names(&self.obstacles);
                                for name in &self.colliding_link_names {
                                    println!("{}", name);
                                    self.viewer.set_temporal_color(name, 0.8, 0.8, 0.6);
                                }
                                println!("===========");
                            }
                            Key::V => {
                                is_collide_show = !is_collide_show;
                                let ref_robot = self.planner.urdf_robot().as_ref().unwrap();
                                self.viewer.remove_robot(ref_robot);
                                self.viewer.add_robot_with_base_dir_and_collision_flag(
                                    ref_robot,
                                    None,
                                    is_collide_show,
                                );
                                self.viewer.update(&self.planner);
                            }
                            _ => {}
                        }
                    }
                    _ => {}
                }
            }
        }
    }
}

fn main() {
    use std::env;
    env_logger::init().unwrap();
    let input_string = env::args().nth(1).unwrap_or("sample.urdf".to_owned());
    let input_end_link = env::args().nth(2).unwrap_or("l_tool".to_owned());
    let planner = gear::JointPathPlannerBuilder::from_urdf_file(&input_string)
        .unwrap()
        .collision_check_margin(0.01)
        .finalize();
    let solver = gear::JacobianIKSolverBuilder::<f64>::new()
        .num_max_try(1000)
        .allowable_target_distance(0.01)
        .move_epsilon(0.00001)
        .jacobian_move_epsilon(0.001)
        .finalize();
    let solver = gear::RandomInitializeIKSolver::new(solver, 100);
    let planner = gear::JointPathPlannerWithIK::new(planner, solver);
    let mut app = CollisionAvoidApp::new(planner, &input_end_link);
    app.run();
}
