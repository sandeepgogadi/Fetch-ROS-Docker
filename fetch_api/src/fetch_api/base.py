#! /usr/bin/env python

from geometry_msgs.msg import Twist, Vector3
import rospy


class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = fetch_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
        self._pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds.

        "Instantaneously" means that this method must be called continuously in
        a loop for the robot to move.

        Args:
            linear_speed: The forward/backward speed, in meters/second. A
                positive value means the robot should move forward.
            angular_speed: The rotation speed, in radians/second. A positive
                value means the robot should rotate clockwise.
        """
        twist_msg = Twist(Vector3(linear_speed, 0, 0), Vector3(0, 0, angular_speed))
        self._pub.publish(twist_msg)

    def stop(self):
        """Stops the mobile base from moving.
        """
        twist_msg = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        self._pub.publish(twist_msg)
