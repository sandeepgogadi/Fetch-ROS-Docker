#! /usr/bin/env python

import fetch_api
import rospy
from geometry_msgs.msg import Pose, Quaternion, PoseStamped, Point
from moveit_python import PlanningSceneInterface
from nav_msgs.msg import Odometry


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def set_torso_height(height):
    torso = fetch_api.Torso()
    torso.set_height(height)


def goto_pose(position, orientation):
    message = PoseStamped()
    message.header.frame_id = 'map'
    message.pose.position.x = position[0]
    message.pose.position.y = position[1]
    message.pose.position.z = position[2]
    message.pose.orientation.x = orientation[0]
    message.pose.orientation.y = orientation[1]
    message.pose.orientation.z = orientation[2]
    message.pose.orientation.w = orientation[3]
    target_pose_pub.publish(message)


def main():
    rospy.init_node('pick_place')
    wait_for_time()
    argv = rospy.myargv()

    global target_pose_pub
    target_pose_pub = rospy.Publisher('/move_base_simple/goal',
                                      geometry_msgs.msg.PoseStamped, queue_size=10)

    # target1 /odom [x, y, z] = [3.1, 3.22, 0],
    # /amcl_pose [0.005, 0.011, 0], [0, 0, 0, 1]
    goto_pose([0.005, 0.011, 0], [0, 0, 0, 1])
    set_torso_height(.4)

    # pose1 = PoseStamped()
    # pose1.header.frame_id = 'base_link'
    # pose1.pose.position.x = 0.5
    # pose1.pose.position.y = -0.3
    # pose1.pose.position.z = 0.75
    # pose1.pose.orientation.w = 1
    #
    # arm = fetch_api.Arm()
    #
    # def shutdown():
    #     arm.cancel_all_goals()
    # rospy.on_shutdown(shutdown)
    #
    # kwargs = {
    #     'allowed_planning_time': 15,
    #     'execution_timeout': 10,
    #     'num_planning_attempts': 5,
    #     'replan': False
    # }
    # error = arm.move_to_pose(pose1, **kwargs)
    # if error is not None:
    #     rospy.logerr('Pose 1 failed: {}'.format(error))
    # else:
    #     rospy.loginfo('Pose 1 succeeded')
    # rospy.sleep(1)
    # error = arm.move_to_pose(pose2, **kwargs)
    # if error is not None:
    #     rospy.logerr('Pose 2 failed: {}'.format(error))
    # else:
    #     rospy.loginfo('Pose 2 succeeded')

    # target2 /odom [-2.9, 4.6, 0]
    # /amcl [-4.44, 4.2, 0], [0, 0, .73, .68]

    goto_pose([-4.44, 4.2, 0], [0, 0, .73, .68])


if __name__ == '__main__':
    main()
