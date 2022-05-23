#!/usr/bin/env python3

from shutil import move
from socket import timeout
import sys
import copy
from time import sleep
from turtle import position
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
import rospkg
import os
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import *
# Import with ROS reference for runtime and relative path reference for IDE
from helene_helper import *


class MoveGroupHelper(helene_helper):
  """MoveGroupHelper, inherits from helpy"""
  def __init__(self):
    """Constructor for MoveGroupHelper"""
    super().__init__()

    # Get planning scene interface
    self.planning_scene = moveit_commander.PlanningSceneInterface()

    # Get planning frame and end effector link
    self.planning_frame = self.move_group.get_planning_frame()
    self.eef_link = self.move_group.get_end_effector_link()

    # Define cylinder containing tumor
    self.cylinder_pose = geometry_msgs.msg.PoseStamped()
    self.cylinder_pose.header.frame_id = "world"
    self.cylinder_pose.pose.position.x = 0.3
    self.cylinder_pose.pose.position.z = 0.035
    self.cylinder_pose.pose.orientation.w = 1.0
    self.cylinder_height = 0.08
    self.cylinder_radius = 0.04
    self.cylinder_name = 'full_cylinder'
    self.hollow_cylinder_name = 'hollow_cylinder'

    # Get ROS package utility for access to package locations
    self.rospack = rospkg.RosPack()

    # Sleep for 5 seconds to allow planning scene interface to properly wake up
    sleep(5)

    # Save current robot pose as start pose
    self.start_pose = copy.deepcopy(self.move_group.get_current_pose().pose)

    # Add cylinder containing tumor to scene
    self.__add_hollow_cylinder()

  def sleep(self, time_in_sec):
    rospy.sleep(time_in_sec)

  def get_cylinder_pose(self):
    """Get pose message for cylinder from planning scene"""
    if self.cylinder_name in self.planning_scene.get_known_object_names():
      self.cylinder_pose.pose = self.planning_scene.get_object_poses([self.cylinder_name])[self.cylinder_name]


  def __add_cylinder(self, timeout=10):
    """Add cylinder to planning scene"""
    # Add cylinder and wait for planning scene to update
    self.planning_scene.add_cylinder(self.cylinder_name, self.cylinder_pose, self.cylinder_height, self.cylinder_radius)
    return self.__wait_for_state_update(self.cylinder_name, cylinder_is_known=True, timeout=timeout)


  def __get_relative_orientation(self, target_link_name, reference_link_name):
    """Calculate target link orientation relative to refernce link orientation"""
    # Compute inverse quaternion of reference link orientation
    reference_orientation = self.move_group.get_current_pose(target_link_name).pose.orientation
    reference_orientation_inverted = [reference_orientation.x, reference_orientation.y, reference_orientation.z, -reference_orientation.w]

    # Compute quaternion of target link orientation
    target_orientation_msg = self.move_group.get_current_pose(reference_link_name).pose.orientation
    target_orientation = [target_orientation_msg.x, target_orientation_msg.y, target_orientation_msg.z, target_orientation_msg.w]

    # Multiply the two quaternions to get the target links orientation relative to the reference links orientation
    relative_orientation = quaternion_multiply(reference_orientation_inverted, target_orientation)

    # Create orientation message
    relative_orientation_msg = geometry_msgs.msg.Quaternion(relative_orientation[0], relative_orientation[1], relative_orientation[2], relative_orientation[3])

    return relative_orientation_msg


  def __remove_cylinder(self, timeout=4):
    """Remove cylinder from planning scene"""
    self.__wait_for_state_update(self.cylinder_name, cylinder_is_known=True, timeout=timeout)

    # Remove cylinder from the scene
    self.planning_scene.remove_attached_object(name=self.cylinder_name)
    self.planning_scene.remove_world_object(name=self.cylinder_name)

    # We wait for the planning scene to update.
    return self.__wait_for_state_update(self.cylinder_name, cylinder_is_known=False, timeout=timeout)


  def add_tumor(self):
    """Enable probing, which allows the robot to pierce the cylinder from the top and imposes motion constraints"""
    # Replace solid cylinder with hollow cylinder
    self.__add_hollow_cylinder()
    self.__remove_cylinder()
    # Set slower movement speed
    #self.move_group.set_max_velocity_scaling_factor(0.01)
    #self.move_group.set_max_acceleration_scaling_factor(0.1)


  def remove_tumor(self):
    """Disable probing"""
    # Replace hollow cylinder with solid cylinder
    self.__add_cylinder()
    self.__remove_hollow_cylinder()
    # Reset movement speed to normal speed
    #self.move_group.set_max_velocity_scaling_factor(0.8)
    #self.move_group.set_max_acceleration_scaling_factor(1)

  def probing_start(self):
    self.set_reserved(1)

  def probing_end(self):
    self.set_reserved(0)


  def go_to_probing_pos(self):
    """Go to probing position"""
    # Create copy of current pose
    wpose = copy.deepcopy(self.move_group.get_current_pose().pose)
    
    # Set first waypoint
    wpose.position.z -= 0.27  # First move down (z)
    wpose.position.x += 0.27  # Second move forward in (x)

    # Set orientation
    quat = quaternion_from_euler(0, -pi/2, pi)
    wpose.orientation.x = quat[0]
    wpose.orientation.y = quat[1]
    wpose.orientation.z = quat[2]
    wpose.orientation.w = quat[3]

    # Go to position
    self.move_group.plan()
    self.move_group.go(copy.deepcopy(wpose))


  def go_home(self):
    """Go to home position (pose saved at startup)"""
    self.move_group.plan()
    self.move_group.go(copy.deepcopy(self.start_pose))


  def plan_probing(self, z = -0.1):
    """Plan cartesian motion for probing, where z defines the vertical motion (default = 0.1m)"""
    # Create waypoints
    waypoints = []

    # Create copy of current pose
    wpose = copy.deepcopy(self.move_group.get_current_pose().pose)
    
    # Set first waypoint
    wpose.position.z += z   # Move down (z)

    # Append target pose to waypoints
    waypoints.append(copy.deepcopy(wpose))

    self.move_group.plan()
    (plan, fraction) = self.move_group.compute_cartesian_path(
                        waypoints,                                        # waypoints to follow
                        0.01,                                             # eef_step
                        0.0                                              # jump_threshold
                        )    # constraints

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction


  def __wait_for_state_update(self, cylinder_name, cylinder_is_known=False, timeout=4):
    """Waits for planning scene to update or times out after given timeout (default=4s)"""
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      
      # Test if the cylinder is in the scene.
      # Note that attaching the cylinder will remove it from known_objects
      is_known = cylinder_name in self.scene.get_known_object_names()

      # Test if we are in the expected state
      if cylinder_is_known == is_known:
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False


  def execute_plan(self, plan):
    """Execute given plan"""
    self.move_group.execute(plan, wait=True)


  def __add_hollow_cylinder(self):
    """Add hollow cylinder to planning scene"""
    cylinder_pose = copy.deepcopy(self.cylinder_pose)
    cylinder_pose.pose.position.z -= 0.035
    filename = os.path.join(self.rospack.get_path('messtechnik_praktikum') ,"stl/hollow_cylinder.stl")
    self.planning_scene.add_mesh(self.hollow_cylinder_name, pose=cylinder_pose, filename=filename)
    return self.__wait_for_state_update(self.hollow_cylinder_name, cylinder_is_known=True)


  def __remove_hollow_cylinder(self):
    """"Remove the hollow cylinder from the planning scene"""
    # Remove cylinder from the scene
    self.planning_scene.remove_world_object(self.hollow_cylinder_name)
    self.__wait_for_state_update(self.hollow_cylinder_name, cylinder_is_known=False)

    # We wait for the planning scene to update.
    return self.__wait_for_state_update(self.cylinder_name, cylinder_is_known=False, timeout=4)

def main():
  try:
    helene = MoveGroupHelper()

    helene.set_led_blue(0)
    helene.set_led_green(0)
    helene.set_speed_scaler(1)
    helene.move_ptp_home_pos()

    print(helene.get_actual_pos())
    helene.set_speed_scaler(0.3)
    helene.set_led_green(255)
    Frame_Start = [0.289,0.0,0.30, *(pi, -pi/2, 0)]
    helene.move_ptp_abs(Frame_Start)
    print(helene.get_actual_pos())

    helene.sleep(1)

    helene.probing_start()
    helene.set_led_green(0)
    helene.set_led_blue(255)
    helene.set_speed_scaler(0.02)
    Frame_Test = [0.289,0.0,0.22, *(pi, -pi/2, 0)]
    helene.move_lin_abs(Frame_Test)
    helene.sleep(3)
    print(helene.get_actual_pos())
    helene.probing_end()

    helene.set_led_green(255)
    helene.set_led_blue(0)
    helene.set_speed_scaler(0.1)
    helene.move_lin_rel([0,0,0.1,*(0,0,0)])

    helene.set_led_blue(0)
    helene.set_led_green(0)
    helene.set_speed_scaler(1)
    helene.move_ptp_home_pos()
    print("Fertig")

    print("\n-- Helper demo complete! --")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()