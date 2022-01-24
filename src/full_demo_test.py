#!/usr/bin/env python

import rospy
import roslib
import tf
import sys
import copy
import numpy as np
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from franka_gripper.msg import GraspAction, GraspGoal, GraspEpsilon
import actionlib

# Initialize moveit commander
moveit_commander.roscpp_initialize(sys.argv)
# Initialize node
rospy.init_node('move_demo_test', anonymous=True)
# Get the robot used
robot = moveit_commander.RobotCommander()
# Planning Scene
scene = moveit_commander.PlanningSceneInterface()

# ARM Move GROUP #############################
move_arm = moveit_commander.MoveGroupCommander("panda_arm")
joints = move_arm.get_joints()
arm_publisher = rospy.Publisher('/move_arm/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory, queue_size=10)

# GRIPPER Move GROUP ###########################
move_hand = moveit_commander.MoveGroupCommander("hand")
hand_publisher = rospy.Publisher('/move_hand/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory, queue_size=10)
#Grasping Group
grasping_group ='hand'


# Print Info ###################################
print "===== Reference frame: %s", move_arm.get_planning_frame()
print "===== End effector link: %s", move_arm.get_end_effector_link()
print "===== Available Planning Groups:", robot.get_group_names()

rate = rospy.Rate(1)
listener = tf.TransformListener()
stage = 0
restart = 1

# Grasp Function ################################
def grasp(width, e_inner, e_outer, speed, force):
  """
  Moves the gripper fingers to a specific width.
  Width: opening width (m), float
  e_inner: epsilon inner, float
  e_outer: epsilon outer, float
  Speed: closing speed (m/s), float
  Force: force to apply (N) 
  """
  client = actionlib.SimpleActionClient('franka_gripper/grasp', GraspAction)
  print "====== Waiting for action server to start"
  client.wait_for_server()
  client.send_goal(GraspGoal(width, GraspEpsilon(e_inner, e_outer), speed, force))
  return client.wait_for_result(rospy.Duration.from_sec(3.0))


# Display Trajectory Function ######################
def display_traj(self, plan):
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()
  display_trajectory.trajectory_start = robot.get_current_state()
  display_trajectory.trajectory.append(plan)
  # Publish
  arm_publisher.publish(display_trajectory);


# Execute Plan for Arm Function #####################
def execute_plan(self, plan):
  move_arm.execute(plan, wait=True)


# Execute Pre Grasp Function ########################
def pre_pose(self, scale=1):

  waypoints = []

  wpose = move_arm.get_current_pose().pose
  wpose.position.z -= scale * 0.2 # First, move down (z)
  waypoints.append(copy.deepcopy(wpose))

  (plan, fraction) = move_arm.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

  # Just planning, not moving Panda yet:
  return plan, fraction

def pre_grasp(self, scale=1):

  waypoints = []

  wpose = move_arm.get_current_pose().pose
  wpose.position.z -= scale * 0.035 # First, move down (z)
  waypoints.append(copy.deepcopy(wpose))

  (plan01, fraction01) = move_arm.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

  # Just planning, not moving Panda yet:
  return plan01, fraction01

def place(self, scale=1):

  waypoints = []

  wpose = move_arm.get_current_pose().pose
  wpose.position.z += scale * 0.3 # First, move up (z)
  wpose.position.y += scale * 0.1 # and sideways (y)
  waypoints.append(copy.deepcopy(wpose))

  wpose.position.x -= scale * 0.2 # Second move backwards (x)
  waypoints.append(copy.deepcopy(wpose))

  wpose.position.z -= scale * 0.22 # Third move down

  (plan02, fraction02) = move_arm.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

  # Just planning, not moving Panda yet:
  return plan02, fraction02

# MAIN ##############################################
if __name__ == '__main__':
  move_arm.allow_replanning(True)
  move_arm.num_planning_attempts = 50

  # Add table to planning scene
  p = PoseStamped()
  rospy.sleep(1)
  p.header.frame_id = robot.get_planning_frame()
  p.pose.position.x = 0.7
  p.pose.position.y = 0.0
  p.pose.position.z = -0.01
  scene.add_box("table", p, (1.2, 1.5, 0.005))  
  rospy.sleep(1)

  while not rospy.is_shutdown():
    while restart == 1:
      if stage == 0:
        print "===== Stage 0: Getting into initial pose"
        # Arm go into ready pose
        move_arm.set_named_target("ready")
        plan1 = move_arm.go(wait=True)
        # Open the gripper
        move_hand.set_named_target("open")
        plan2 = move_hand.go()
        stage = 1
        rospy.sleep(2) # Wait
	valid_init = raw_input('TYPE ok if Panda is in initial pose, else press enter ')
        if valid_init == 'ok':
          stage = 1
  	  rospy.sleep(1)
        else:
  	  print "===== Restart initial pose"
          stage = 0

      if stage == 1:
	print "===== Stage 1: Getting into pre-grasp pose"
	plan, fraction = pre_pose(move_arm, 1)
	display_traj(arm_publisher, plan)
	valid_trajectory = raw_input('TYPE ok to execute trajectory, else press enter ')
        if valid_trajectory == 'ok':
          execute_plan(move_arm, plan)
          stage = 2
  	  rospy.sleep(1)
        else:
  	  print "===== Restart"
          stage = 1

      if stage == 2:
        print "===== Stage 2: Getting arm into pre-grasp pose"
        try:
	  # Wait for TF
          listener.waitForTransform('/panda_link0', '/arm_goal', rospy.Time(), rospy.Duration(20.0))
	  # Look for Transform      
          (translation, rotation) = listener.lookupTransform('/panda_link0', '/arm_goal', rospy.Time(0))
          print "===== Object found"
	  print "Translation ", translation
	  print "Rotation ", rotation
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          continue
        else:
          print "===== Go to Arm Goal Pose"
          pose_goal = geometry_msgs.msg.Pose()
          pose_goal.orientation.x = rotation[0]
          pose_goal.orientation.y = rotation[1]
          pose_goal.orientation.z = rotation[2]
          pose_goal.orientation.w = rotation[3]
          pose_goal.position.x = translation[0] 
          pose_goal.position.y = translation[1] 
          pose_goal.position.z = translation[2]
          move_arm.set_pose_target(pose_goal)
          plan_3 = move_arm.plan()

          print "===== Display trajectory"
	  display_traj(arm_publisher, plan_3)

          valid_trajectory = raw_input('TYPE ok to execute trajectory, else press enter ')
          if valid_trajectory == 'ok':
            execute_plan(move_arm, plan_3)
	    rospy.sleep(2)

	    plan01, fraction01 = pre_grasp(move_arm, 1)
	    display_traj(arm_publisher, plan01)
	    execute_plan(move_arm, plan01)
	    rospy.sleep(1)

            stage = 3
  	    rospy.sleep(1)
          else:
  	    print "===== Restart"
            stage = 2

      if stage == 3: 
	rospy.sleep(1)
        print "===== Stage 3: Grasping"
	grasp(0.05, 0.05, 0.05, 0.08, 3) # width, e inner, e outer, speed, force
	valid_grasp = raw_input('TYPE ok if grasp done, else press enter ')
        if valid_grasp == 'ok':
          stage = 4
  	  rospy.sleep(1)
        else:
          stage = 3

      if stage == 4:
	rospy.sleep(1)
	print "===== Stage 4: Going to Place Pose"
        plan02, fraction02 = place(move_arm, 1)
	display_traj(arm_publisher, plan02)
	valid_place = raw_input('TYPE ok to execute trajectory, else press enter ')
        if valid_place == 'ok':
          execute_plan(move_arm, plan02)
	  stage = 5
  	  rospy.sleep(1)
        else:
  	  print "===== Restart"
          stage = 4

      if stage == 5:
	rospy.sleep(1)
	print "===== Stage 5: Release Object"
	move_hand.set_named_target("open")
        plan_4 = move_hand.go()
	restart = 1
	rospy.sleep(1)
        exit()

    
