#! /usr/bin/env python

from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
import rospy
import copy
import math
import tf.transformations as tft

class Base(object):
    """Base controls the mobile base portion of the Fetch robot.
    Sample usage:
        base = fetch_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    ANGULAR_DIST_THRESH = 0.005

    def __init__(self):
        self._pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self._odom_sub = rospy.Subscriber('odom', Odometry, callback=self._odom_callback)
        self._latest_odom = None

    def _odom_callback(self, msg):
        self._latest_odom = msg

    def _distance(self, p1, p2):
        return ((p1.x - p2.x)**2 + (p1.y - p2.y)**2)**0.5

    def _angular_dist(self, q1, q2):
        return abs((q1.z - q2.z) % (2 * math.pi))

    def goto_in_odom(self, target_point):
        """Moves the robot to the specified location in odom frame.
            Args:
                target_x, target_y: (x,y) of target location
        """
        while (self._latest_odom is None):
            rospy.sleep(2.0)

        cur_point = self._latest_odom.pose.pose.position
        cur_angle = self._convert_quaternion_to_radian(
                            self._latest_odom.pose.pose.orientation)

        dx = target_point.x - cur_point.x
        dy = target_point.y - cur_point.y
        theta = math.atan2(dy, dx) % (2*math.pi)

        # turn to face the target
        self.turn(theta - cur_angle)
        # move to target
        self.go_forward(self._distance(cur_point, target_point))

    def go_forward(self, distance, speed=0.1):
        """Moves the robot a certain distance.
        It's recommended that the robot move slowly. If the robot moves too
        quickly, it may overshoot the target. Note also that this method does
        not know if the robot's path is perturbed (e.g., by teleop). It stops
        once the distance traveled is equal to the given distance or more.
        Args:
            distance: The distance, in meters, to move. A positive value
                means forward, negative means backward.
            speed: The speed to travel, in meters/second.
        """

        while (self._latest_odom is None):
            rospy.sleep(2.0)

        start = copy.deepcopy(self._latest_odom)
        rate = rospy.Rate(20)


        while self._distance(start.pose.pose.position, self._latest_odom.pose.pose.position) < abs(distance):
            direction = -1 if distance < 0 else 1
            self.move(direction * speed, 0)
            rate.sleep()

    def _is_at_turn_goal(self, start_angle, angular_distance):
        cur_angle = self._convert_quaternion_to_radian(
                            self._latest_odom.pose.pose.orientation)

        dist_so_far = 0
        if angular_distance > 0:
            # travelling counter clockwise, the angle should always increase unless hit reset point (0)
            if (cur_angle - start_angle) > self.ANGULAR_DIST_THRESH:
                # cur > start
                dist_so_far = cur_angle - start_angle
            elif (start_angle - cur_angle) > self.ANGULAR_DIST_THRESH:
                # cur < start, we must have hit reset point
                dist_so_far = (cur_angle + 2*math.pi) - start_angle
        else:
            # travelling clockwise, the angle should always decrease
            if (start_angle - cur_angle) > self.ANGULAR_DIST_THRESH:
                # cur < start
                dist_so_far = start_angle - cur_angle
            elif (cur_angle - start_angle) > self.ANGULAR_DIST_THRESH:
                # cur > start, we must have hit reset point
                dist_so_far = start_angle - (cur_angle - 2*math.pi)

        # print "dist " + str(dist_so_far)

        if dist_so_far >= (abs(angular_distance) % (2*math.pi)):
            return True
        return False

    # Returns radian in range 0 to 2pi
    def _convert_quaternion_to_radian(self, q):
        m = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        x = m[0,0]
        y = m[1,0]
        return math.atan2(y, x) % (2*math.pi)

    def turn(self, angular_distance, speed=0.5):
        """Rotates the robot a certain angle.
        Args:
            angular_distance: The angle, in radians, to rotate. A positive
                value rotates counter-clockwise.
            speed: The angular speed to rotate, in radians/second.
        """
        while (self._latest_odom is None):
            rospy.sleep(2.0)
        start = copy.deepcopy(self._latest_odom)
        start_angle = self._convert_quaternion_to_radian(start.pose.pose.orientation)
        rate = rospy.Rate(50)
        while not self._is_at_turn_goal(start_angle, angular_distance):
            direction = -1 if angular_distance < 0 else 1
            self.move(0, direction * speed)
            rate.sleep()

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

