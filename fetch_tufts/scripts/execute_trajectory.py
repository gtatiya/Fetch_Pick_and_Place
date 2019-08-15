#!/usr/bin/env python

import rospy
import moveit_commander
import tf
import rospkg
import os
import actionlib
import time

import numpy as np

from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal,
                              GripperCommandAction,
                              GripperCommandGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

rospy.init_node('execute_trajectory', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

# You can get a list with all the groups of the robot like this:
print "Robot Groups:"
print robot.get_group_names()

"""
arm_group = moveit_commander.MoveGroupCommander("arm")
arm_with_torso_group = moveit_commander.MoveGroupCommander("arm_with_torso")
grp_group = moveit_commander.MoveGroupCommander("gripper")

# arm_group.set_planner_id("LBKPIECE")

# You can get the reference frame for a certain group by executing this line:
print "Arm Reference frame: %s" % arm_group.get_planning_frame()
print "Gripper Reference frame: %s" % grp_group.get_planning_frame()

# You can get the end-effector link for a certaing group executing this line:
print "Arm End effector: %s" % arm_group.get_end_effector_link()
print "Gripper End effector: %s" % grp_group.get_end_effector_link()


# You can get the current values of the joints like this:
print "Arm Current Joint Values:"
print arm_group.get_current_joint_values()
print "Arm with torso Current Joint Values:"
print arm_with_torso_group.get_current_joint_values()
print "Gripper Current Joint Values:"
print grp_group.get_current_joint_values()

# You can also get the current Pose of the end-effector of the robot like this:
print "Arm Current Pose:"
print arm_group.get_current_pose()
print "Arm with torso Current Pose:"
print arm_with_torso_group.get_current_pose()

# Finally, you can check the general status of the robot like this:
print "Robot State:"
print robot.get_current_state()
"""

def spawn_gazebo_model(model_path, model_name, model_pose, reference_frame="world"):
  """
  Spawn model in gazebo
  """
  model_xml = ''
  with open(model_path, "r") as model_file:
    model_xml = model_file.read().replace('\n', '')
  rospy.wait_for_service('/gazebo/spawn_urdf_model')
  try:
    spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    resp_urdf = spawn_urdf(model_name, model_xml, "/", model_pose, reference_frame)
  except rospy.ServiceException, e:
    rospy.logerr("Spawn URDF service call failed: {0}".format(e))

def delete_gazebo_model(models):
  """
  Delete model in gazebo
  """
  try:
    delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    for a_model in models:
      resp_delete = delete_model(a_model)
  except rospy.ServiceException, e:
    rospy.loginfo("Delete Model service call failed: {0}".format(e))


def get_default_pose():
    arm_joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    # arm_intermediate_positions  = [1.32, 0, -1.4, 1.72, 0.0, 1.66, 0.0]
    arm_intermediate_positions  = [0.0, -0.62, 0, 0, 0.0, 0.62, 0.0]
    # arm_joint_positions  = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
    # arm_joint_positions  = [1.1, -0.64, -1.83, 0.96, 1.13, -.96, 0.0]
    arm_joint_positions  = [1.32, 0.7, 0.0, -2.0, 0.0, -0.57, 0.0]
    head_joint_names = ["head_pan_joint", "head_tilt_joint"]
    head_joint_positions = [0.0, 0.0]

    #rospy.init_node("prepare_simulated_robot_pick_place")

    # Check robot serial number, we never want to run this on a real robot!
    if rospy.get_param("robot/serial") != "ABCDEFGHIJKLMNOPQRSTUVWX":
        rospy.logerr("This script should not be run on a real robot")
        sys.exit(-1)

    # rospy.loginfo("Waiting for move_base...")
    # move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # move_base.wait_for_server()
    # rospy.loginfo("...connected.")

    rospy.loginfo("Waiting for head_controller...")
    head_client = actionlib.SimpleActionClient("head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    head_client.wait_for_server()
    rospy.loginfo("...connected.")

    rospy.loginfo("Waiting for arm_controller...")
    arm_client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    arm_client.wait_for_server()
    rospy.loginfo("...connected.")

    rospy.loginfo("Waiting for gripper_controller...")
    gripper_client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
    gripper_client.wait_for_server()
    rospy.loginfo("...connected.")

    # move_goal = MoveBaseGoal()
    # move_goal.target_pose.pose.position.x = x
    # move_goal.target_pose.pose.position.y = y
    # move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
    # move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
    # move_goal.target_pose.header.frame_id = frame
    # move_goal.target_pose.header.stamp = rospy.Time.now()
    # move_base.send_goal(move_goal)
    # move_base.wait_for_result()

    trajectory = JointTrajectory()
    trajectory.joint_names = head_joint_names
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = head_joint_positions
    trajectory.points[0].velocities = [0.0] * len(head_joint_positions)
    trajectory.points[0].accelerations = [0.0] * len(head_joint_positions)
    trajectory.points[0].time_from_start = rospy.Duration(5.0)

    head_goal = FollowJointTrajectoryGoal()
    head_goal.trajectory = trajectory
    head_goal.goal_time_tolerance = rospy.Duration(0.0)

    trajectory = JointTrajectory()
    trajectory.joint_names = arm_joint_names

    # trajectory.points.append(JointTrajectoryPoint())
    # trajectory.points[0].positions = [0.0] * len(arm_joint_positions)
    # trajectory.points[0].velocities =  [0.0] * len(arm_joint_positions)
    # trajectory.points[0].accelerations = [0.0] * len(arm_joint_positions)
    # duration = 5.0
    # trajectory.points[0].time_from_start = rospy.Duration(duration)
    #
    # trajectory.points.append(JointTrajectoryPoint())
    # trajectory.points[1].positions = arm_intermediate_positions
    # trajectory.points[1].velocities =  [0.0] * len(arm_joint_positions)
    # trajectory.points[1].accelerations = [0.0] * len(arm_joint_positions)
    # duration += 5.0
    # trajectory.points[1].time_from_start = rospy.Duration(duration)
    #
    # trajectory.points.append(JointTrajectoryPoint())
    # trajectory.points[2].positions = arm_joint_positions
    # trajectory.points[2].velocities =  [0.0] * len(arm_joint_positions)
    # trajectory.points[2].accelerations = [0.0] * len(arm_joint_positions)
    # duration += 5.0
    # trajectory.points[2].time_from_start = rospy.Duration(duration)

    i = 0
    duration = 5.0
    trajectory.points.append(JointTrajectoryPoint())
    arm_joint_positions  = [0.0, -1.22, 0.0, 1.20, 0.0, 1.6, 0.0] # above the block, READY TO PICK
    trajectory.points[i].positions = arm_joint_positions
    trajectory.points[i].velocities =  [0.0] * len(arm_joint_positions)
    trajectory.points[i].accelerations = [0.0] * len(arm_joint_positions)
    trajectory.points[i].time_from_start = rospy.Duration(duration)

    i += 1
    duration += 5.0
    trajectory.points.append(JointTrajectoryPoint())
    arm_joint_positions  = [0.0, -1.20, 0.0, 1.45, 0.0, 1.3, 0.0] # PERFECT - PICK POSE
    trajectory.points[i].positions = arm_joint_positions
    trajectory.points[i].velocities =  [0.0] * len(arm_joint_positions)
    trajectory.points[i].accelerations = [0.0] * len(arm_joint_positions)
    trajectory.points[i].time_from_start = rospy.Duration(duration)

    # i += 1
    # duration += 5.0
    # trajectory.points.append(JointTrajectoryPoint())
    # arm_joint_positions  = [0.0, -1.22, 0.0, 1.20, 0.0, 1.6, 0.0] # above the block, READY TO PICK
    # trajectory.points[i].positions = arm_joint_positions
    # trajectory.points[i].velocities =  [0.0] * len(arm_joint_positions)
    # trajectory.points[i].accelerations = [0.0] * len(arm_joint_positions)
    # trajectory.points[i].time_from_start = rospy.Duration(duration)

    arm_goal = FollowJointTrajectoryGoal()
    arm_goal.trajectory = trajectory
    arm_goal.goal_time_tolerance = rospy.Duration(0.0)

    gripper_goal = GripperCommandGoal()
    gripper_goal.command.max_effort = 10.0
    gripper_goal.command.position = 0.1

    rospy.loginfo("Setting positions...")
    head_client.send_goal(head_goal)
    arm_client.send_goal(arm_goal)
    gripper_client.send_goal(gripper_goal)
    gripper_client.wait_for_result(rospy.Duration(5.0))
    arm_client.wait_for_result(rospy.Duration(6.0))
    head_client.wait_for_result(rospy.Duration(6.0))
    rospy.loginfo("...done")

    gripper_goal = GripperCommandGoal()
    gripper_goal.command.max_effort = 10.0
    gripper_goal.command.position = 0.0
    #gripper_client.send_goal(gripper_goal)
    time.sleep(5.0)
    gripper_client.send_goal_and_wait(gripper_goal, rospy.Duration(5.0))
    #gripper_client.wait_for_result(rospy.Duration(100.0))
    rospy.loginfo("...done")

    trajectory = JointTrajectory()
    trajectory.joint_names = arm_joint_names
    i = 0
    duration = 5.0
    trajectory.points.append(JointTrajectoryPoint())
    arm_joint_positions  = [0.0, -1.22, 0.0, 1.20, 0.0, 1.6, 0.0] # above the block, READY TO PICK
    trajectory.points[i].positions = arm_joint_positions
    trajectory.points[i].velocities =  [0.0] * len(arm_joint_positions)
    trajectory.points[i].accelerations = [0.0] * len(arm_joint_positions)
    trajectory.points[i].time_from_start = rospy.Duration(duration)

    arm_goal = FollowJointTrajectoryGoal()
    arm_goal.trajectory = trajectory
    arm_goal.goal_time_tolerance = rospy.Duration(0.0)
    arm_client.send_goal(arm_goal)
    arm_client.wait_for_result(rospy.Duration(6.0))





rospack = rospkg.RosPack()
pack_path = rospack.get_path('fetch_tufts')

block_path = os.path.join(pack_path, 'models', 'block', 'model7.urdf')
block_name = 'block'
block_pose = Pose(position=Point(x=0.57, y=-0.07, z=0.7))

delete_gazebo_model([block_name])
spawn_gazebo_model(block_path, block_name, block_pose)

pack_path = rospack.get_path('fetch_tufts')
block_path = os.path.join(pack_path, 'robots', 'fetch_gazebo.urdf')
block_name = 'fetch'
block_pose = Pose(position=Point(x=0.0, y=0.0, z=0.0))
delete_gazebo_model([block_name])
spawn_gazebo_model(block_path, block_name, block_pose)


#get_default_pose()

exit()

pose = arm_with_torso_group.get_current_pose().pose
# Block point
pose.position.x = 0.0
pose.position.y = 0.0
pose.position.z = 0.0

downOrientation = tf.transformations.quaternion_from_euler(0, 3.1415/2, 0)
print("downOrientation: ", downOrientation)
pose.orientation.x = downOrientation[0]
pose.orientation.y = downOrientation[1]
pose.orientation.z = downOrientation[2]
pose.orientation.w = downOrientation[3]

arm_with_torso_group.set_pose_target(pose)
arm_with_torso_group.go(wait=True)
print("Point 1")
print "Arm with torso Current Joint Values:"
print arm_group.get_current_joint_values()



rospack = rospkg.RosPack()
pack_path = rospack.get_path('fetch_tufts')

block_path = os.path.join(pack_path, 'models', 'block', 'model0.urdf')
block_name = 'block'
block_pose = Pose(position=Point(x=0.6, y=-0.3, z=0.75))

scene.remove_world_object("obstacle")
# Adding obstacle
p = PoseStamped()
p.header.frame_id = robot.get_planning_frame()
p.header.stamp = rospy.Time.now()

p.pose.position.x = 1.0
p.pose.position.y = 0.0
p.pose.position.z = 0.74-0.08

q = tf.transformations.quaternion_from_euler(0.0, 0.0, np.deg2rad(90.0))
p.pose.orientation = Quaternion(*q)

scene.add_box("obstacle", p, (0.913, 0.913, 0.04))


# joint_pose = [-1.60, -1.10, -1.5, -1.5, -0.2, -1.50, 0.0]
# joint_pose = [0.1, 0.1, -1.5, 1.0, -0.2, -1.8, 2.0]
joint_pose = [0, 1.32, 0.7, 0.0, -2.0, 0.0, -0.57, 0.0]
arm_with_torso_group.set_joint_value_target(joint_pose)
arm_with_torso_group.go(wait=True)
print("Point 1")
print "Arm with Torso Current Joint Values:"
print arm_with_torso_group.get_current_joint_values()

delete_gazebo_model([block_name])
spawn_gazebo_model(block_path, block_name, block_pose)

pose = arm_group.get_current_pose().pose
# Block point
pose.position.x = 0.65
pose.position.y = -0.175
pose.position.z = 1.0

downOrientation = tf.transformations.quaternion_from_euler(0, 3.1415/2, 0)
print("downOrientation: ", downOrientation)
pose.orientation.x = downOrientation[0]
pose.orientation.y = downOrientation[1]
pose.orientation.z = downOrientation[2]
pose.orientation.w = downOrientation[3]

arm_group.set_pose_target(pose)
arm_group.go(wait=True)
print("Point 2")
print "Arm Current Joint Values:"
print arm_group.get_current_joint_values()

pose = arm_group.get_current_pose().pose
# Block point
pose.position.x = 0.65
pose.position.y = -0.175
pose.position.z = 0.9

downOrientation = tf.transformations.quaternion_from_euler(0, 3.1415/2, 0)
print("downOrientation: ", downOrientation)
pose.orientation.x = downOrientation[0]
pose.orientation.y = downOrientation[1]
pose.orientation.z = downOrientation[2]
pose.orientation.w = downOrientation[3]

arm_group.set_pose_target(pose)
arm_group.go(wait=True)
print("Point 3")
print "Arm Current Joint Values:"
print arm_group.get_current_joint_values()


"""
The Lenght of gripper is ~0.15
Min. reachable z value on the table is 0.9 from downOrientation
Min. reachable x on table: 0.37

Max. reachable y on table: 0.46

Right front corner of table
  x: 1.3552107811
  y: 0.493778288364
  z: 0.719269096851

Right back corner of table
  x: 0.352727115154
  y: 0.504722833633
  z: 0.714043438435

Left front corner of table
  x: 1.35521054268
  y: -0.491892397404
  z: 0.717385590076

Left back corner of table
  x: 0.363544255495
  y: -0.502857685089
  z: 0.721333742142
"""


arm_group.set_named_target('up')
arm_group.go(wait=True)
print("Point 1")

pose = arm_group.get_current_pose().pose
# Max x points
# pose.position.x = 0.9 #0.9
# pose.position.y = -0.2 # -0.1 to 0.1
# pose.position.z = 0.89

# Min x points
# pose.position.x = 0.37 #0.37
# pose.position.y = -0.46 # -0.46 to 0.46
# pose.position.z = 0.89

# Block point
pose.position.x = 0.4
pose.position.y = 0.0
pose.position.z = 0.90

downOrientation = tf.transformations.quaternion_from_euler(0, 3.1415/2, 0)
print("downOrientation: ", downOrientation)
pose.orientation.x = downOrientation[0]
pose.orientation.y = downOrientation[1]
pose.orientation.z = downOrientation[2]
pose.orientation.w = downOrientation[3]

arm_group.set_pose_target(pose)
arm_group.go(wait=True)
print("Point 2")


# arm_group.set_joint_value_target([-0.21957805043352518, -1.097296859939564, 1.8945345194815335,
#                             -2.366067038969164, -1.571228181260084, -1.0061550793898952])
# arm_group.go(wait=True)
# print("Point 2")

# Close
grp_group.set_joint_value_target([0.8039005131791948, -0.8039005131791948, 0.8039005131791948, 0.8039005131791948, -0.8039005131791948, 0.8039005131791948])
grp_group.go(wait=True)
print("Point 3")

pose = arm_group.get_current_pose().pose
pose.position.x += 0.1
pose.position.y += 0
pose.position.z += 0
arm_group.set_pose_target(pose)
arm_group.go(wait=True)
print("Point 4")

# Open
grp_group.set_joint_value_target([9.800441184282249e-05, -9.800441184282249e-05, 9.800441184282249e-05, 9.800441184282249e-05, -9.800441184282249e-05, 9.800441184282249e-05])
grp_group.go(wait=True)
print("Point 5")

arm_group.set_named_target('up')
arm_group.go(wait=True)
print("Point 6")
