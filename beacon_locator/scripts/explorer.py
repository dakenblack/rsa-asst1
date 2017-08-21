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

from threading import Lock

#MAX_GOAL_DIST = 1
#MIN_GOAL_DIST = 0.2

OG_THRESHOLD = 50

class Explorer():
    
    def __init__(self):

        self.startPose  = None      # the initial pose of the robot in the /map frame
        self.goalPose   = None      # the pose we're currently navigating to
        self.returnHome = False     # denotes that we've found all the beacons
        self.done       = False     # denotes that we're done and don't need to do anything

        self.gridLock   = Lock()
        self.grid       = None

        self.tfListener = tf.TransformListener()

        self.goalPub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

        self.mapSub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.gotMap)
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

        self.gridLock.acquire()
        rospy.loginfo(" ****** Got map! ****** ")
        self.grid = grid
        self.gridLock.release() 

    def getRobotPose(self):
        """ Get robot post in map frame """
        if self.tfListener.frameExists("/base_link") and self.tfListener.frameExists("/map"):
            try:
                t = self.tfListener.getLatestCommonTime("/base_link", "/map")
                p = PoseStamped()
                p.header.frame_id = "/base_link"
                p.header.stamp = t
                mapPose = self.tfListener.transformPose("/map", p)
                rospy.loginfo("Robot position is : %s" % mapPose)
                return mapPose
            except Exception as e:
                rospy.logerr(str(e))
        else:
            rospy.logwarn("Waiting for /base_link and /map transforms to exist!")
            return None

    def explore(self):

        while not rospy.is_shutdown():

            if self.startPose is None:
                self.startPose = self.getRobotPose()
            
            elif self.returnHome:
                rospy.loginfo("Returning to start position")
                self.goalPub.publish(self.startPose)
                self.done = True
                return

            elif self.goalPose is not None:
                
                self.gridLock.acquire()
                rospy.loginfo(" ****** Doing an explore! ****** ")
                grid = self.grid

                gWidth = grid.info.width
                gHeight = grid.info.height

                robotPose = self.getRobotPose()

                robotGridPos = (
                    abs(int((robotPose.pose.position.x - grid.info.origin.position.x) / grid.info.resolution)),
                    abs(int((robotPose.pose.position.y - grid.info.origin.position.y) / grid.info.resolution))
                    )
                gdata = list(grid.data)
                for x in range(6):
                    for y in range(6):
                        gdata[robotGridPos[0]-3+x + (robotGridPos[1]-3+y)*gHeight] = 0

                # search for the closest frontier using bfs. If it's too close, find another one
                explored = set()
                # robot position in occupancy grid
                curr = robotGridPos
                frontier = {curr}
                foundGoal = False
                while frontier:
                    curr = frontier.pop()
                    explored.add(curr)
                    # if cell is unknown, we've found our place to explore!
                    if gdata[curr[0] + curr[1]*gWidth] == -1:
                        rospy.loginfo("****** Found frontier at  = %s = node %s which is (%s, %s) in /map *****" % (curr, curr[0]+curr[1]*gWidth, curr[0]*grid.info.resolution+grid.info.origin.position.x, curr[0]*grid.info.resolution+grid.info.origin.position.y))
                        foundGoal = True
                        break
                    potentialChildren = [
                        (curr[0]+1, curr[1]-1),
                        (curr[0]+1, curr[1]),
                        (curr[0]+1, curr[1]+1),
                        (curr[0],   curr[1]-1),
                        (curr[0],   curr[1]+1),
                        (curr[0]-1, curr[1]-1),
                        (curr[0]-1, curr[1]),
                        (curr[0]-1, curr[1]+1),
                        ]
                    for c in potentialChildren:
                        # remove children that are off-map, explored, or untraversable
                        if c not in explored and c[0] >= 0 and c[0] < gWidth and c[1] >= 0 and c[1] < gHeight and gdata[curr[0] + curr[1]*gWidth] < OG_THRESHOLD:
                            frontier.add(c)
                if not foundGoal:
                    rospy.loginfo(" ****** Couldn't find a frontier! ****** ")
                    self.goalPose = None
                    self.gridLock.release()
                    continue
                            
                # set a pose
                self.goalPose = robotPose
                self.goalPose.pose.position.x = (curr[0] * grid.info.resolution) + grid.info.origin.position.x
                self.goalPose.pose.position.y = (curr[1] * grid.info.resolution) + grid.info.origin.position.y

                self.gridLock.release()
                
                # rotate pose to opposite direction
                quat = (
                    self.goalPose.pose.orientation.x,
                    self.goalPose.pose.orientation.y,
                    self.goalPose.pose.orientation.z,
                    self.goalPose.pose.orientation.w
                    )
                euler = tf.transformations.euler_from_quaternion(quat)
                euler = (euler[0], euler[1], euler[2] + 180)
                quat = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])
                self.goalPose.pose.orientation.x = quat[0]
                self.goalPose.pose.orientation.y = quat[1]
                self.goalPose.pose.orientation.z = quat[2]
                self.goalPose.pose.orientation.w = quat[3]

                self.goalPose.header.stamp = rospy.Time.now()
                rospy.loginfo("Publishing exploration node: %s", self.goalPose)
                self.goalPub.publish(self.goalPose)
                rospy.sleep(13)


            rospy.sleep(2)

        



if __name__ == "__main__":
    rospy.init_node('explorer_node')
    e = Explorer()
    e.explore()
    rospy.spin()

