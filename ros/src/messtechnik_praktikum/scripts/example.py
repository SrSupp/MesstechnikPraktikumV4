#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. (More on these below)
##
## We also import `rospy`_ and some messages that we will use:
##

from shutil import move
import sys
import copy
from time import sleep
from turtle import position
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import *
## END_SUB_TUTORIAL

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class HelperExample(object):
  """HelperExample"""
  def __init__(self):
    super(HelperExample, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Panda
    ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Panda:
    group_name = "helene_arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print("============ Reference frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print("============ End effector: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("============ Robot Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state")
    print(robot.get_current_state())
    print("")
    ## END_SUB_TUTORIAL

    # Misc variables
    self.cylinder_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

    self.cylinder_pose = geometry_msgs.msg.PoseStamped()
    self.cylinder_pose.header.frame_id = "world"
    self.cylinder_pose.pose.position.x = 0.3
    self.cylinder_pose.pose.position.z = 0.035
    self.cylinder_pose.pose.orientation.w = 1.0
    self.cylinder_name = "cylinder"

    self.cylinder_height = 0.08
    self.cylinder_radius = 0.04

  def go_to_joint_state(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3
    joint_goal[6] = 0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_pose_goal(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def plan_cartesian_path(self, constraints=None):

    # Create waypoints

    waypoints = []

    # Save current pose
    opose = self.group.get_current_pose().pose
    # Create copy
    wpose = copy.deepcopy(opose)
    
    # Set first waypoint
    wpose.position.z -= 0.1  # First move down (z)
    waypoints.append(copy.deepcopy(wpose))

    # Set second waypoint
    wpose.position.x += 0.2  # Second move forward in (x)
    waypoints.append(copy.deepcopy(wpose))

    # Set third waypoint
    quat = quaternion_from_euler(0, -pi/2, pi)
    wpose.orientation.x = quat[0]
    wpose.orientation.y = quat[1]
    wpose.orientation.z = quat[2]
    wpose.orientation.w = quat[3]
    waypoints.append(copy.deepcopy(opose))

    # Return to start
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    self.group.plan()
    (plan, fraction) = self.group.compute_cartesian_path(
                        waypoints,                        # waypoints to follow
                        0.01,                             # eef_step
                        0.0,                              # jump_threshold
                        path_constraints = constraints)   # constraints

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

    ## END_SUB_TUTORIAL

  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL

  def wait_for_state_update(self, cylinder_is_known=False, cylinder_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    cylinder_name = self.cylinder_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the cylinder will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the cylinder is in attached objects
      attached_objects = scene.get_attached_objects([cylinder_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the cylinder is in the scene.
      # Note that attaching the cylinder will remove it from known_objects
      is_known = cylinder_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (cylinder_is_attached == is_attached) and (cylinder_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL

  def add_cylinder(self, timeout=4):
    scene = self.scene
    scene.add_cylinder(self.cylinder_name, self.cylinder_pose, self.cylinder_height, self.cylinder_radius)
    return self.wait_for_state_update(cylinder_is_known=True, timeout=timeout)

  def apply_constraints(self):
    orientation_constraint = moveit_msgs.msg.OrientationConstraint()
    orientation_constraint.link_name = "axis_6"
    orientation_constraint.header.frame_id = "axis_0"

    #get_relative_orientation()

    axis0_orientation = self.group.get_current_pose("axis_0").pose.orientation
    axis0_orientation_inverted = [axis0_orientation.x, axis0_orientation.y, axis0_orientation.z, -axis0_orientation.w]
    
    axis6_orientation_msg = self.group.get_current_pose("axis_6").pose.orientation
    axis6_orientation = [axis6_orientation_msg.x, axis6_orientation_msg.y, axis6_orientation_msg.z, axis6_orientation_msg.w]
    
    relative_orientation = quaternion_multiply(axis0_orientation_inverted, axis6_orientation)

    relative_orientation_msg = geometry_msgs.msg.Quaternion(relative_orientation[0], relative_orientation[1], relative_orientation[2], relative_orientation[3])

    print(relative_orientation_msg)

    orientation_constraint.orientation = relative_orientation_msg

    orientation_constraint.absolute_x_axis_tolerance = 0.1
    orientation_constraint.absolute_y_axis_tolerance = 0.1
    orientation_constraint.absolute_z_axis_tolerance = 0.1
    orientation_constraint.weight = 1.0

    constraints = moveit_msgs.msg.Constraints()
    constraints.orientation_constraints.append(orientation_constraint)

    position_constraint = moveit_msgs.msg.PositionConstraint()
    position_constraint.link_name = "axis_6"
    position_constraint.header.frame_id = "axis_0"
    position_constraint.target_point_offset.z = 0

    bounding_region = shape_msgs.msg.SolidPrimitive()
    bounding_region.type = bounding_region.CYLINDER
    bounding_region.dimensions = [0, 0]
    bounding_region.dimensions[bounding_region.CYLINDER_HEIGHT] = 1
    bounding_region.dimensions[bounding_region.CYLINDER_RADIUS] = 0.01

    position_constraint.constraint_region.primitives.append(bounding_region)

    constraints.position_constraints.append(position_constraint)

    self.group.set_path_constraints(constraints)
    return constraints


  def remove_cylinder(self, timeout=4):
    # Remove cylinder from the scene
    self.scene.remove_world_object(self.cylinder_name)

    # We wait for the planning scene to update.
    return self.wait_for_state_update(cylinder_is_attached=False, cylinder_is_known=False, timeout=timeout)


def main():
  try:
    helper = HelperExample()

    helper.add_cylinder()

    constraints = helper.apply_constraints()

    cartesian_plan, fraction = helper.plan_cartesian_path(constraints=constraints)

    helper.execute_plan(cartesian_plan)

    helper.remove_cylinder()

    print("============ Helper demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()