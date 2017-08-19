#!/usr/bin/env python
"""
    Explorer node
    Subscribes to an OccupancyGrid on the '/map' topic
    Subscribes to a String on the '/beacon_locator_node/status' topic
    Publishes a PoseStamped to the '/move_base/simple_goal' topic
    First records the initial position of the rover (when this node starts)
    Turns the occupancy grid into a costmap-like matrix so that it can better evaluate frontiers
    Finds closest unobstructed frontier using breadth first search and sends the robot there
    Once all the beacons are found, it sends the robot back to its initial position
    By: Nuno Das Neves
"""

import rospy
import tf

from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose


class Explorer():
    
    def __init__(self):

        self.startPose  = None      # the initial pose of the robot in the /map frame
        self.returnHome = False     # denotes that we've found all the beacons
        self.done       = False     # denotes that we're done and don't need to do anything

        self.tfListener = tf.TransformListener()

        self.goalPub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

        self.mapSub = rospy.Subscriber('/map', OccupancyGrid, self.gotMap)
        self.statusSub = rospy.Subscriber('/beacon_locator_node/status', String, self.gotStatus)

        # only needed for testing
        self.mapPub = rospy.Publisher('/explore_map', OccupancyGrid, queue_size=5) 
    
    def gotStatus(self, status):

        if status.data == "complete":
            rospy.loginfo("All beacons found")
            self.returnHome = True
    
    def gotMap(self, grid):

        if self.done or self.returnHome:
            rospy.loginfo("Done with exploring")
            return
        
        MAX_GOAL_DIST = 1
        MIN_GOAL_DIST = 0.2

        RES_FACTOR = 5

        OG_THRESHOLD = 20
        
        # we need to 'expand' the impassable areas of the map so that we don't try to go through walls if there's a tiny gap
        rospy.loginfo("Map: \n%s" % str(grid.info))
        gWidth = grid.info.width
        gHeight = grid.info.height

        newHeight = gHeight/RES_FACTOR
        newWidth = gWidth/RES_FACTOR
        newVals = [[0]*newHeight for i in range(newWidth)]
        for i in range(gHeight*gWidth):
            if grid.data[i] > OG_THRESHOLD:
                newVals[(i/gHeight)/RES_FACTOR][(i%gWidth)/RES_FACTOR] = 100

        newData = [0]*gWidth*gHeight
        for i in range(gWidth*gHeight):
            newData[i] = newVals[(i/gHeight)/RES_FACTOR][(i%gWidth)/RES_FACTOR]
        grid.data = tuple(newData)

        rospy.loginfo("Published new map")

        self.mapPub.publish(grid)

        # search for the closest frontier using bfs. If it's too close, find another one

        # set a pose at some reasonable distance; 
        # basically go through the points on the path until we reach the max distance we want to set a point at


    def explore(self):

        while not rospy.is_shutdown():

            if self.startPose is None:
                if self.tfListener.frameExists("/base_link") and self.tfListener.frameExists("/map"):
                    try:
                        t = self.tfListener.getLatestCommonTime("/base_link", "/map")
                        #pos, rot = self.tfListener.lookupTransform("/base_link", "/map", t)
                        p = PoseStamped()
                        p.header.frame_id = "/base_link"
                        p.header.stamp = t
                        """
                        p.pose.position.x = pos[0]
                        p.pose.position.y = pos[1]
                        p.pose.position.z = pos[2]
                        p.pose.orientation.x = rot[0]
                        p.pose.orientation.y = rot[1]
                        p.pose.orientation.z = rot[2]
                        p.pose.orientation.w = rot[3]
                        """
                        self.startPose = self.tfListener.transformPose("/map", p)
                        rospy.loginfo("Initial position is : %s" % self.startPose)
                    except Exception as e:
                        rospy.logerr(str(e))
                else:
                    rospy.logwarn("Waiting for /base_link and /map transforms to exist!")

            if self.returnHome:
                rospy.loginfo("Returning to start position")
                self.goalPub.publish(self.startPose)
                self.done = True
                return

            rospy.sleep(2)

        



if __name__ == "__main__":
    rospy.init_node('explorer_node')
    e = Explorer()
    e.explore()
    rospy.spin()

