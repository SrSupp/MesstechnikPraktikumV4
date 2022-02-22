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

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

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
  static const std::string PLANNING_GROUP = "helene_arm";

  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));


  // Add table

  moveit_msgs::CollisionObject table;
  table.header.frame_id = "world";

  table.id = "table";

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 1.5;
  primitive.dimensions[primitive.BOX_Y] = 1.5;
  primitive.dimensions[primitive.BOX_Z] = 0.05;

  geometry_msgs::Pose table_pose;
  table_pose.orientation.w = 1.0;
  table_pose.position.z = -0.0251;

  table.primitives.push_back(primitive);
  table.primitive_poses.push_back(table_pose);
  table.operation = table.ADD;

  // Add cylinder

  moveit_msgs::CollisionObject cylinder;
  cylinder.header.frame_id = "world";
  cylinder.id = "cylinder";

  shape_msgs::SolidPrimitive cylinder_primitive;

  cylinder_primitive.type = primitive.CYLINDER;
  cylinder_primitive.dimensions.resize(2);
  cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.07;
  cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.04;

  geometry_msgs::Pose cylinder_pose;
  cylinder_pose.orientation.w = 1.0;
  cylinder_pose.position.x = 0.3;
  cylinder_pose.position.z = 0.035;

  cylinder.primitives.push_back(cylinder_primitive);
  cylinder.primitive_poses.push_back(cylinder_pose);
  cylinder.operation = cylinder.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  // collision_objects.push_back(cylinder);
  collision_objects.push_back(table);

  // Now, let's add the collision object into the world
  // (using a vector that could contain additional objects)
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // The result may look like this:
  //
  // .. image:: ./move_group_interface_tutorial_avoid_path.gif
  //    :alt: animation showing the arm moving avoiding the new obstacle
  //
  // Attaching objects to the robot
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // You can attach objects to the robot, so that it moves with the robot geometry.
  // This simulates picking up the object for the purpose of manipulating it.
  // The motion planning should avoid collisions between the two objects as well.
  // moveit_msgs::CollisionObject needle;
  // needle.id = "needle";

  // shape_msgs::SolidPrimitive cylinder_primitive;
  // cylinder_primitive.type = primitive.CYLINDER;
  // cylinder_primitive.dimensions.resize(2);
  // cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.1;
  // cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.01;

  // // We define the frame/pose for this cylinder so that it appears in the gripper
  // needle.header.frame_id = move_group_interface.getEndEffectorLink();
  // geometry_msgs::Pose grab_pose;
  // grab_pose.orientation.w = 1.0;
  // grab_pose.position.z = 0.05;

  // // First, we add the object to the world (without using a vector)
  // needle.primitives.push_back(cylinder_primitive);
  // needle.primitive_poses.push_back(grab_pose);
  // needle.operation = needle.ADD;
  // planning_scene_interface.applyCollisionObject(needle);

  // // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
  // // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
  // ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
  // move_group_interface.attachObject(needle.id, "iiwa_link_ee");

  /*
  //planning_scene::PlanningScene::getAllowedCollisionMatrix()
  auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  psm->startSceneMonitor("/move_group/monitored_planning_scene");
  bool success = psm->requestPlanningSceneState("/iiwa/get_planning_scene");
  if (success)
  {
    ROS_INFO_NAMED("tutorial", "Success");
  }
  
  collision_detection::AllowedCollisionMatrix test = psm->getPlanningScene()->getAllowedCollisionMatrix();
  ros::V_string names;
  test.getAllEntryNames(names);
  if (test.hasEntry("needle") && test.hasEntry("cube"))
  {
    test.setEntry("needle", "cube", true);
    ROS_INFO_NAMED("tutorial", "set entry to true");
  }
  else
  {
    for (int i=0; i<names.size(); i++)
    {
      ROS_INFO_STREAM_NAMED("tutorial", "Name: " << names[i]);
    }
    
  }
  */
  ros::shutdown();
  return 0;
}
