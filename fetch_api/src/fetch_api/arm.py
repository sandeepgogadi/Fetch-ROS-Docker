#! /usr/bin/env python

import trajectory_msgs.msg
import rospy
import control_msgs.msg
import actionlib

from .arm_joints import ArmJoints

TIME_FROM_START = 5
ARM_GROUP_NAME = 'arm'
JOINT_ACTION_SERVER = 'arm_controller/follow_joint_trajectory'


class Arm(object):
    """Arm controls the robot's arm.
    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = fetch_api.Arm()
        arm.move_to_joints(joints)
    """

    def __init__(self):
        self._joint_client = actionlib.SimpleActionClient(
            JOINT_ACTION_SERVER, control_msgs.msg.FollowJointTrajectoryAction)
        self._joint_client.wait_for_server(rospy.Duration(10))

    def move_to_joints(self, arm_joints):
        """Moves the robot's arm to the given joints.
        Args:
            arm_joints: An ArmJoints object that specifies the joint values for
                the arm.
        """
        # Create a trajectory point
        trajPoint = trajectory_msgs.msg.JointTrajectoryPoint()
        # Set position of trajectory point
        trajPoint.positions.extend(arm_joints.values())
        # Set time of trajectory point
        trajPoint.time_from_start = rospy.Duration(TIME_FROM_START)
        # Create goal
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        # Add joint name to list
        goal.trajectory.joint_names.extend(arm_joints.names())
        # Add the trajectory point created above to trajectory
        goal.trajectory.points.append(trajPoint)
        # Send goal
        self._joint_client.send_goal(goal)
        self._joint_client.wait_for_result(rospy.Duration(10))
