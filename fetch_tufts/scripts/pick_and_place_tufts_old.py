#!/usr/bin/env python

import os
import sys
import time
import subprocess
import signal
import psutil

import rospy
import rospkg
import actionlib

import numpy as np

from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal,
                              GripperCommandAction,
                              GripperCommandGoal)
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


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

def move_arm_joints(client, arm_joint_positions, duration=5.0):
    trajectory = JointTrajectory()
    trajectory.joint_names = ARM_JOINT_NAMES
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = arm_joint_positions
    trajectory.points[0].velocities =  [0.0] * len(arm_joint_positions)
    trajectory.points[0].accelerations = [0.0] * len(arm_joint_positions)
    trajectory.points[0].time_from_start = rospy.Duration(duration)

    arm_goal = FollowJointTrajectoryGoal()
    arm_goal.trajectory = trajectory
    arm_goal.goal_time_tolerance = rospy.Duration(0.0)

    print("Moving amr to joints: ", arm_joint_positions)
    client.send_goal(arm_goal)
    client.wait_for_result(rospy.Duration(duration+1.0))
    rospy.loginfo("...done")

def move_arm_joints_v2(client, arm_joint_positions_list):
    trajectory = JointTrajectory()
    trajectory.joint_names = ARM_JOINT_NAMES

    duration = 0.0
    wait_time = 3.0
    for i, a_joint_pose in enumerate(arm_joint_positions_list):
        duration += wait_time
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[i].positions = a_joint_pose
        trajectory.points[i].velocities =  [0.0] * len(a_joint_pose)
        trajectory.points[i].accelerations = [0.0] * len(a_joint_pose)
        trajectory.points[i].time_from_start = rospy.Duration(duration)

    arm_goal = FollowJointTrajectoryGoal()
    arm_goal.trajectory = trajectory
    arm_goal.goal_time_tolerance = rospy.Duration(0.0)

    print("Moving amr to joints: ", arm_joint_positions_list)
    client.send_goal(arm_goal)
    client.wait_for_result(rospy.Duration(len(arm_joint_positions_list)*wait_time))
    rospy.loginfo("...done")

def move_head_joints(client, head_joint_positions):
    trajectory = JointTrajectory()
    trajectory.joint_names = HEAD_JOINT_NAMES
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = head_joint_positions
    trajectory.points[0].velocities = [0.0] * len(head_joint_positions)
    trajectory.points[0].accelerations = [0.0] * len(head_joint_positions)
    trajectory.points[0].time_from_start = rospy.Duration(1.0)

    head_goal = FollowJointTrajectoryGoal()
    head_goal.trajectory = trajectory
    head_goal.goal_time_tolerance = rospy.Duration(0.0)

    print("Moving head to joints: ", head_joint_positions)
    client.send_goal(head_goal)
    client.wait_for_result(rospy.Duration(1.0))
    rospy.loginfo("...done")

class Gripper(object):
    """Gripper controls the robot's gripper.
    """
    CLOSED_POS = 0.0  # The position for a fully-closed gripper (meters).
    OPENED_POS = 0.10  # The position for a fully-open gripper (meters).
    MIN_EFFORT = 35  # Min grasp force, in Newtons
    MAX_EFFORT = 100  # Max grasp force, in Newtons

    def __init__(self):
        rospy.loginfo("Waiting for gripper_controller...")
        self._client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
        self._client.wait_for_server(rospy.Duration(10))
        rospy.loginfo("...connected.")

    def open(self):
        """Opens the gripper.
        """
        goal = GripperCommandGoal()
        goal.command.position = self.OPENED_POS
        self._client.send_goal_and_wait(goal, rospy.Duration(10))

    def close(self, max_effort=MAX_EFFORT):
        """Closes the gripper.
        Args:
            max_effort: The maximum effort, in Newtons, to use. Note that this
                should not be less than 35N, or else the gripper may not close.
        """
        goal = GripperCommandGoal()
        goal.command.position = self.CLOSED_POS
        goal.command.max_effort = max_effort
        self._client.send_goal_and_wait(goal, rospy.Duration(10))

def start_rosbag_recording(path, filename):
    # find the directory to save to
    rospy.loginfo(rospy.get_name() + ' start')
    rosbagfile_dir = path+os.sep+"rosbagfiles/"

    if not os.path.exists(rosbagfile_dir):
        os.mkdir(rosbagfile_dir)

    rosbag_process = subprocess.Popen('rosbag record -o {} /joint_states'.format(filename), stdin=subprocess.PIPE, shell=True, cwd=rosbagfile_dir)

    return rosbag_process

def stop_rosbag_recording(p):
    rospy.loginfo(rospy.get_name() + ' stop recording.')
    rospy.loginfo(p.pid)

    process = psutil.Process(p.pid)
    for sub_process in process.children(recursive=True):
        sub_process.send_signal(signal.SIGINT)
    p.wait() # we wait for children to terminate
    #p.terminate()

    rospy.loginfo("I'm done recording")
    # rostopic echo -b fetch_grasp_model_0_2019-06-26-19-09-34.bag -p /joint_states > data.csv
    # command = "rostopic echo -b "+source_file+" -p /robot/joint_states > "+target_file+"csv"
    # os.system(command)
    #target_file = path+os.sep+os.sep.join(source_file_list[-4:])[:-3]
    #command = "rostopic echo -b "+source_file+" -p /robot/joint_states > "+target_file+"csv"
    #os.system(command)

def stop_rosbag_recording_v2(p):
    rospy.loginfo(rospy.get_name() + ' stop recording.')
    rospy.loginfo(p.pid)

    s = "/record"
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    for str in list_output.split("\n"):
        if (str.startswith(s)):
            os.system("rosnode kill " + str)

    rospy.loginfo("I'm done recording")

def get_clients():
    ARM_JOINT_NAMES = ["shoulder_pan_joint", "shoulder_lift_joint",
    "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint",
    "wrist_flex_joint", "wrist_roll_joint"]
    rospy.loginfo("Waiting for arm_controller...")
    arm_client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    arm_client.wait_for_server()
    rospy.loginfo("...connected.")

    HEAD_JOINT_NAMES = ["head_pan_joint", "head_tilt_joint"]
    rospy.loginfo("Waiting for head_controller...")
    head_client = actionlib.SimpleActionClient("head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    head_client.wait_for_server()
    rospy.loginfo("...connected.")
    head_joint_positions = [0.0, 0.0]

    gripper_client = Gripper()

    return arm_client, head_client, gripper_client

if __name__ == "__main__":
    rospy.init_node("simulated_robot_pick_place")

    myargv = rospy.myargv(argv=sys.argv)
    num_of_run = int(myargv[1])

    # ARM_JOINT_NAMES = ["shoulder_pan_joint", "shoulder_lift_joint",
    # "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint",
    # "wrist_flex_joint", "wrist_roll_joint"]
    # rospy.loginfo("Waiting for arm_controller...")
    # arm_client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    # arm_client.wait_for_server()
    # rospy.loginfo("...connected.")
    #
    # HEAD_JOINT_NAMES = ["head_pan_joint", "head_tilt_joint"]
    # rospy.loginfo("Waiting for head_controller...")
    # head_client = actionlib.SimpleActionClient("head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    # head_client.wait_for_server()
    # rospy.loginfo("...connected.")
    # head_joint_positions = [0.0, 0.0]
    #
    # gripper_client = Gripper()

    rospack = rospkg.RosPack()
    PATH = rospack.get_path('fetch_tufts')

    # 7 to 18+1
    # model7: table at z -0.08 (Y50)
    # model8: table at z -0.08 (H202)
    # model9: table at z -0.08 (Lab3)
    # model13: table at z -0.095
    for x in range(7, 7+1):
        for _ in range(0, num_of_run):
            print("Picking Block: ", x)
            filename = str(x-7)

            block_path = os.path.join(PATH, 'models', 'block', 'model'+str(x)+'.urdf')
            block_name = 'block'+str(x)
            #block_pose = Pose(position=Point(x=0.57, y=-0.07, z=0.7))
            block_pose = Pose(position=Point(x=0.57, y=-0.05, z=0.7))
            delete_gazebo_model([block_name])
            time.sleep(1.0)
            spawn_gazebo_model(block_path, block_name, block_pose)

            robot_path = os.path.join(PATH, 'robots', 'fetch_gazebo.urdf')
            robot_name = 'fetch'
            robot_pose = Pose(position=Point(x=0.0, y=0.0, z=0.01))
            delete_gazebo_model([robot_name])
            time.sleep(2.0)
            spawn_gazebo_model(robot_path, robot_name, robot_pose)
            arm_client, head_client, gripper_client = get_clients()
            #time.sleep(2.0)
            move_head_joints(head_client, head_joint_positions)

            arm_joint_positions = [0.0, -1.22, 0.0, 1.20, 0.0, 1.6, 0.0] # Above Pick
            move_arm_joints(arm_client, arm_joint_positions, 3.0)
            gripper_client.open()

            fn_grasp = "fetch_grasp_model_" + filename
            rps = start_rosbag_recording(PATH, fn_grasp)
            arm_joint_positions  = [0.0, -1.20, 0.0, 1.45, 0.0, 1.3, 0.0] # Pick Pose
            move_arm_joints(arm_client, arm_joint_positions, 3.0)
            gripper_client.close()
            # stop_rosbag_recording(rps)
            stop_rosbag_recording_v2(rps)

            fn_pick = "fetch_pick_model_" + filename
            rps = start_rosbag_recording(PATH, fn_pick)
            #arm_joint_positions = [0.0, -1.22, 0.4, 1.20, 0.0, 1.4, 0.0] # Above place
            #arm_joint_positions = [0.0, -1.0, 0.4, 1.0, 0.0, 1.6, 0.0] # Above place
            arm_joint_positions = [0.0, np.random.uniform(-1.22, -1.0), 0.4, np.random.uniform(1.0, 1.20), 0.0, np.random.uniform(1.4, 1.6), 0.0]
            move_arm_joints(arm_client, arm_joint_positions, 3.0)
            # stop_rosbag_recording(rps)
            stop_rosbag_recording_v2(rps)

            #"""
            fn_hold = "fetch_hold_model_" + filename
            rps = start_rosbag_recording(PATH, fn_hold)
            time.sleep(2.0)
            # stop_rosbag_recording(rps)
            stop_rosbag_recording_v2(rps)

            fn_shake = "fetch_shake_model_" + filename
            rps = start_rosbag_recording(PATH, fn_shake)
            # arm_joint_positions_list = [
            # [0.90, -1.0, 0.4, 1.0, 0.0, 1.6, 0.0],
            # [0.90, -1.1, 0.4, 1.1, 0.0, 2.0, 0.0],
            # [0.90, -1.2, 0.5, 1.2, 0.0, 1.0, 0.0],
            # [0.90, -1.1, 0.4, 1.1, 0.0, 2.0, 0.0]
            # ]
            arm_joint_positions_list = [
            [0.90, -1.0, 0.4, 1.0, 0.0, 1.6, 0.0],
            [0.90, np.random.uniform(-1.0, -1.1), 0.4, np.random.uniform(1.0, 1.1), 0.0, np.random.uniform(1.9, 2.0), 0.0],
            [0.90, np.random.uniform(-1.1, -1.2), np.random.uniform(0.4, 0.5), np.random.uniform(1.0, 1.2), 0.0, np.random.uniform(0.9, 1.0), 0.0],
            [0.90, np.random.uniform(-1.0, -1.1), 0.4, np.random.uniform(1.0, 1.1), 0.0, np.random.uniform(1.9, 2.0), 0.0],
            [0.90, np.random.uniform(-1.1, -1.2), np.random.uniform(0.4, 0.5), np.random.uniform(1.0, 1.2), 0.0, np.random.uniform(0.9, 1.0), 0.0],
            ]
            move_arm_joints_v2(arm_client, arm_joint_positions_list)
            # stop_rosbag_recording(rps)
            stop_rosbag_recording_v2(rps)
            #"""

            fn_place = "fetch_place_model_" + filename
            rps = start_rosbag_recording(PATH, fn_place)
            #arm_joint_positions = [0.0, -1.20, 0.4, 1.45, 0.0, 1.2, 0.0] # Place pose
            arm_joint_positions = [0.0, -1.10, 0.4, 1.3, 0.0, 1.2, 0.0] # Place pose
            #arm_joint_positions = [0.0, np.random.uniform(-1.20, -1.10), 0.4, np.random.uniform(1.3, 1.45), 0.0, 1.2, 0.0]
            move_arm_joints(arm_client, arm_joint_positions, 3.0)
            gripper_client.open()
            # stop_rosbag_recording(rps)
            stop_rosbag_recording_v2(rps)

            #delete_gazebo_model([block_name, robot_name])
