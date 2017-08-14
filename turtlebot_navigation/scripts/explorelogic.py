#! /usr/bin/env python

import roslib
roslib.load_manifest('turtlebot_navigation')
import rospy
import actionlib

from frontier_exploration.msg._ExploreTaskAction import ExploreTaskAction
from frontier_exploration.msg._ExploreTaskGoal import ExploreTaskGoal
from frontier_exploration.msg._ExploreTaskFeedback import ExploreTaskFeedback

from geometry_msgs.msg import PolygonStamped, PointStamped, PoseStamped

if __name__ == '__main__':
    rospy.init_node('explore_client')
    client = actionlib.SimpleActionClient('explore_server', ExploreTaskAction)
    client.wait_for_server()

    goal = ExploreTaskGoal(
	    explore_center = PointStamped(),
            explore_boundary = PolygonStamped()
        )

    client.send_goal(goal)
    
