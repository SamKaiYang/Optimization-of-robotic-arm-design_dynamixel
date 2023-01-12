/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <ros/ros.h>
/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */
#include <math.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
// kinematics
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
// 
#include <tf/transform_datatypes.h>
// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;
double Convert(double degree)
{
    double pi = 3.14159265359;
    return (degree * (pi / 180));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangeably.
  static const std::string PLANNING_GROUP = "teco_arm";

  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("dummy");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  move_group_interface.setMaxVelocityScalingFactor(0.5);
  move_group_interface.setMaxAccelerationScalingFactor(0.5);
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // // .. _move_group_interface-planning-to-pose-goal:
  // //
  // // Planning to a Pose goal
  // // ^^^^^^^^^^^^^^^^^^^^^^^
  // // We can plan a motion for this group to a desired pose for the
  // // end-effector.

  // // method 1 : set euler to Quaternion set pose for planning
  // // geometry_msgs::Pose target_pose1;
  // // tf::Quaternion q;
  // // double rotz = 1.57;
  // // double roty = 0.0;
  // // double rotx = 0.0;
  // // q.setEulerZYX(rotz,roty,rotx);

  // // target_pose1.orientation.x = q.x();
  // // target_pose1.orientation.y = q.y();
  // // target_pose1.orientation.z = q.z();
  // // target_pose1.orientation.w = q.w();
  // // target_pose1.position.x = -0.313963;
  // // target_pose1.position.y = -0.100249;
  // // target_pose1.position.z = 0.518271;
  // // move_group_interface.setPoseTarget(target_pose1);

  // // method 2 : Quaternion set pose for planning
  // geometry_msgs::Pose target_pose1;

  // target_pose1.orientation.x = 0.707023;
  // target_pose1.orientation.y = -0.706964;
  // target_pose1.orientation.z = -0.0126717;
  // target_pose1.orientation.w = 0.0126323;
  // target_pose1.position.x = -0.313943;
  // target_pose1.position.y = -0.100248;
  // target_pose1.position.z = 0.518313;
  // move_group_interface.setPoseTarget(target_pose1);

  // // 獲取目標位置
  // ROS_INFO_STREAM("get_pose: " << move_group_interface.getPoseTarget("6"));
  // // 獲取當前歐拉角
  // std::vector<double> RPY = move_group_interface.getCurrentRPY("6");
  // for(std::size_t i = 0; i < 3; ++i)
  // {
  //   ROS_INFO("RPY: %f", RPY[i]);
  // }

  // // Now, we call the planner to compute the plan and visualize it.
  // // Note that we are just planning, not asking move_group_interface
  // // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // // Visualizing plans
  // // ^^^^^^^^^^^^^^^^^
  // // We can also visualize the plan as a line with markers in RViz.
  // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  // visual_tools.publishAxisLabeled(target_pose1, "pose1");
  // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // // Finally, to execute the trajectory stored in my_plan, you could use the following method call:
  // // Note that this can lead to problems if the robot moved in the meanwhile.
  // move_group_interface.execute(my_plan);

  // Moving to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  //
  // If you do not want to inspect the planned trajectory,
  // the following is a more robust combination of the two-step plan+execute pattern shown above
  // and should be preferred. Note that the pose goal we had set earlier is still active,
  // so the robot will try to move to that goal.

  // move_group_interface.move();

  // Planning to a joint-space goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Let's set a joint space goal and move towards it.  This will replace the
  // pose target we set above.
  //
  // To start, we'll create an pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
  //
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  // joint_group_positions[0] = -tau / 6;  // -1/6 turn in radians

  // init 位置的各軸角度
  joint_group_positions[0] = Convert(0);
  joint_group_positions[1] = Convert(0);
  joint_group_positions[2] = Convert(0);
  joint_group_positions[3] = Convert(0);
  joint_group_positions[4] = Convert(0);
  joint_group_positions[5] = Convert(0);

  moveit::core::RobotStatePtr robot_state = move_group_interface.getCurrentState();
  move_group_interface.setJointValueTarget(joint_group_positions);

  // Get Joint Values
  // ^^^^^^^^^^^^^^^^
  // We can retreive the current set of joint values stored in the state for the right arm.
  std::vector<double> joint_values;
  robot_state->copyJointGroupPositions(joint_model_group, joint_values);
  for(int i = 0; i < 6; ++i)
  {
    ROS_INFO("Joint %d: %f", i, joint_values[i]);
  }
  
  // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
  // The default values are 10% (0.1).
  // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
  // or set explicit factors in your code if you need your robot to move faster.
  move_group_interface.setMaxVelocityScalingFactor(0.5);
  move_group_interface.setMaxAccelerationScalingFactor(0.5);

  success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  move_group_interface.execute(my_plan);
// -----result --------------------------------

  // 在execute 後,獲取當前位置, 與姿態
  ROS_INFO_STREAM("get_pose: " << move_group_interface.getCurrentPose().pose);
  std::vector<double> RPY_2 = move_group_interface.getCurrentRPY("6");
  for(std::size_t i = 0; i < 3; ++i)
  {
    ROS_INFO("RPY: %f", RPY_2[i]);
  }

// --------------
  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//action 1
  // Planning to a joint-space goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Let's set a joint space goal and move towards it.  This will replace the
  // pose target we set above.
  //
  // To start, we'll create an pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  current_state = move_group_interface.getCurrentState();
  //
  // Next get the current set of joint values for the group.
  // std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  // joint_group_positions[0] = -tau / 6;  // -1/6 turn in radians

  // init 位置的各軸角度
  joint_group_positions[0] = Convert(0);
  joint_group_positions[1] = Convert(90);
  joint_group_positions[2] = Convert(0);
  joint_group_positions[3] = Convert(0);
  joint_group_positions[4] = Convert(0);
  joint_group_positions[5] = Convert(0);

  robot_state = move_group_interface.getCurrentState();
  move_group_interface.setJointValueTarget(joint_group_positions);

  // Get Joint Values
  // ^^^^^^^^^^^^^^^^
  // We can retreive the current set of joint values stored in the state for the right arm.
  // std::vector<double> joint_values;
  robot_state->copyJointGroupPositions(joint_model_group, joint_values);
  for(int i = 0; i < 6; ++i)
  {
    ROS_INFO("Joint %d: %f", i, joint_values[i]);
  }
  
  // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
  // The default values are 10% (0.1).
  // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
  // or set explicit factors in your code if you need your robot to move faster.
  move_group_interface.setMaxVelocityScalingFactor(0.5);
  move_group_interface.setMaxAccelerationScalingFactor(0.5);

  success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  move_group_interface.execute(my_plan);
// -----result --------------------------------

  // 在execute 後,獲取當前位置, 與姿態
  ROS_INFO_STREAM("get_pose: " << move_group_interface.getCurrentPose().pose);
  RPY_2 = move_group_interface.getCurrentRPY("6");
  for(std::size_t i = 0; i < 3; ++i)
  {
    ROS_INFO("RPY: %f", RPY_2[i]);
  }

// --------------
  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");


// action 2
  // Planning to a joint-space goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Let's set a joint space goal and move towards it.  This will replace the
  // pose target we set above.
  //
  // To start, we'll create an pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  current_state = move_group_interface.getCurrentState();
  //
  // Next get the current set of joint values for the group.
  // std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  // joint_group_positions[0] = -tau / 6;  // -1/6 turn in radians

  // init 位置的各軸角度
  joint_group_positions[0] = Convert(0);
  joint_group_positions[1] = Convert(-90);
  joint_group_positions[2] = Convert(0);
  joint_group_positions[3] = Convert(0);
  joint_group_positions[4] = Convert(0);
  joint_group_positions[5] = Convert(0);

  robot_state = move_group_interface.getCurrentState();
  move_group_interface.setJointValueTarget(joint_group_positions);

  // Get Joint Values
  // ^^^^^^^^^^^^^^^^
  // We can retreive the current set of joint values stored in the state for the right arm.
  // std::vector<double> joint_values;
  robot_state->copyJointGroupPositions(joint_model_group, joint_values);
  for(int i = 0; i < 6; ++i)
  {
    ROS_INFO("Joint %d: %f", i, joint_values[i]);
  }
  
  // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
  // The default values are 10% (0.1).
  // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
  // or set explicit factors in your code if you need your robot to move faster.
  move_group_interface.setMaxVelocityScalingFactor(0.5);
  move_group_interface.setMaxAccelerationScalingFactor(0.5);

  success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  move_group_interface.execute(my_plan);
// -----result --------------------------------

  // 在execute 後,獲取當前位置, 與姿態
  ROS_INFO_STREAM("get_pose: " << move_group_interface.getCurrentPose().pose);
  RPY_2 = move_group_interface.getCurrentRPY("6");
  for(std::size_t i = 0; i < 3; ++i)
  {
    ROS_INFO("RPY: %f", RPY_2[i]);
  }

// --------------
  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  ros::shutdown();
  return 0;
}
