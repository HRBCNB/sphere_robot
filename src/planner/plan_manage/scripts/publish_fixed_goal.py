#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped


def main():
    rospy.init_node('publish_fixed_goal')

    goal_x = rospy.get_param('~goal_x', 15.0)
    goal_y = rospy.get_param('~goal_y', 0.0)
    goal_z = rospy.get_param('~goal_z', 1.0)
    delay = rospy.get_param('~delay', 3.0)
    repeat = rospy.get_param('~repeat', 3)
    interval = rospy.get_param('~interval', 1.0)

    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)

    rospy.sleep(delay)

    goal = PoseStamped()
    goal.header.frame_id = 'world'
    goal.pose.position.x = goal_x
    goal.pose.position.y = goal_y
    goal.pose.position.z = goal_z
    goal.pose.orientation.w = 1.0

    for _ in range(max(1, int(repeat))):
        if rospy.is_shutdown():
            return
        goal.header.stamp = rospy.Time.now()
        pub.publish(goal)
        rospy.logwarn('[publish_fixed_goal] publish fixed goal: x=%.2f, y=%.2f, z=%.2f', goal_x, goal_y, goal_z)
        rospy.sleep(interval)


if __name__ == '__main__':
    main()
