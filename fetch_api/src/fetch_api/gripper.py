#! /usr/bin/env python

import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
import rospy

ACTION_NAME = 'gripper_controller/gripper_action'
CLOSED_POS = 0.0  # The position for a fully-closed gripper (meters).
OPENED_POS = 0.10  # The position for a fully-open gripper (meters).


class Gripper(object):
    """Gripper controls the robot's gripper.
    """
    MIN_EFFORT = 35  # Min grasp force, in Newtons
    MAX_EFFORT = 100  # Max grasp force, in Newtons

    def __init__(self):
        self._client = actionlib.SimpleActionClient(ACTION_NAME, GripperCommandAction)
        self._client.wait_for_server()

    def open(self):
        """Opens the gripper.
        """
        goal = GripperCommandGoal()
        goal.command.position = OPENED_POS
        self._client.send_goal(goal)
        self._client.wait_for_result(rospy.Duration.from_sec(5.0))

    def close(self, max_effort=MAX_EFFORT):
        """Closes the gripper.

        Args:
            max_effort: The maximum effort, in Newtons, to use. Note that this
                should not be less than 35N, or else the gripper may not close.
        """
        goal = GripperCommandGoal()
        goal.command.position = CLOSED_POS
        goal.command.max_effort = max_effort
        self._client.send_goal(goal)
        self._client.wait_for_result(rospy.Duration.from_sec(5.0))
