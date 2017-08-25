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
from map_msgs.msg import OccupancyGridUpdate    # wtf
from geometry_msgs.msg import PoseStamped, Pose

from threading import Lock
import math

# maximum distance to set a goal
MAX_GOAL_DIST = 1
# only update the goal if we're within this distance of the last goal we set
MIN_DIST_TO_PREV_GOAL = 0.3

# threshold in the costmap we consider occupied
OG_THRESHOLD = 85
# the value in the costmap representing unknown area (the area we want to explore!)
UNKNOWN_COST = -1

class Explorer():
    
    def __init__(self):

        self.startPose  = None      # the initial pose of the robot in the /map frame
        self.goalPose   = None      # the pose we're currently navigating to
        self.prevGoal   = None      # denotes the goal we're currently navigating to as a tuple
        self.returnHome = False     # denotes that we've found all the beacons
        self.done       = False     # denotes that we're done and don't need to do anything

        self.gridLock   = Lock()    # for locking the grid data
        self.grid       = None      # the occupancy grid data as a mutable list

        self.gridMsg    = None      # unmodified occupancy grid message

        self.tfListener = tf.TransformListener()

        self.goalPub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

        self.mapSub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.gotMap)
        self.mapSub = rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, self.gotMapUpdate, queue_size=2)
        self.statusSub = rospy.Subscriber('/beacon_locator_node/status', String, self.gotStatus)

        # only needed for testing
        #self.mapPub = rospy.Publisher('/explore_map', OccupancyGrid, queue_size=1) 
    
    def gotStatus(self, status):

        if status.data == "complete":
            rospy.loginfo("All beacons found")
            self.returnHome = True
    
    def gotMap(self, grid):
        """ This is only called once or twice at the start; from then on the costmap is published as updates """
        if self.done or self.returnHome:
            return

        self.gridLock.acquire()
        #rospy.loginfo(" ****** Got map - size %s pos (%s, %s) ****** " % (grid.info.width*grid.info.height, grid.info.origin.position.x, grid.info.origin.position.y))
        self.gridMsg = grid
        self.grid = list(grid.data)
        self.gridLock.release() 
    
    def gotMapUpdate(self, grid):
        """ A map update may or may not contain the whole map """
        if self.done or self.returnHome:
            return

        self.gridLock.acquire()
        rospy.loginfo(" ****** Got map update - size %s pos (%s, %s) ****** " % (grid.width*grid.height, grid.x, grid.y))
        i = 0
        # if the update is the same size as the original map, we can just replace the whole data array
        if grid.width*grid.height == self.gridMsg.info.height*self.gridMsg.info.width:
            self.grid = list(grid.data)

        # otherwise we only want to update the section that is in this update
        else:
            for y in xrange(grid.y, grid.y+grid.height):
                for x in xrange(grid.x, grid.x+grid.width):
                    self.grid[x + y*self.gridMsg.info.height] = grid.data[i]
                    i += 1
        self.gridLock.release() 
        rospy.loginfo(" ****** Done updating map ****** ")

        # testing
        #self.gridMsg.data = tuple(self.grid)
        #self.gridMsg.header.stamp = rospy.Time.now()
        #self.mapPub.publish(self.gridMsg)

    def getRobotPose(self):
        """ Get robot pose in map frame """
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

    def findGoal(self):
        """ Find a point to navigate to based on the map we have """
        

        self.gridLock.acquire()
        rospy.loginfo(" ****** Finding a goal! ****** ")
        grid = self.grid
        gHeight = self.gridMsg.info.height
        gWidth = self.gridMsg.info.width
   
        robotPose = self.getRobotPose()
        if robotPose is None:
            self.gridLock.release()
            return False

        robotGridPos = (
            abs(int((robotPose.pose.position.x - self.gridMsg.info.origin.position.x) / self.gridMsg.info.resolution)),
            abs(int((robotPose.pose.position.y - self.gridMsg.info.origin.position.y) / self.gridMsg.info.resolution))
            )

        # do a check against the last goal we published; if we're not within a certain distance of it, don't make a new goal
        #if self.prevGoal is not None:
        #    dist = math.sqrt((self.prevGoal[0]-robotGridPos[0])**2 + (self.prevGoal[1]-robotGridPos[1])**2) * self.gridMsg.info.resolution
        #    rospy.loginfo(" ****** dist to prevgoal = %s ******" % dist)
        #    if dist > MIN_DIST_TO_PREV_GOAL:
        #        return True

        start = robotGridPos
        
        # fill a few squares around the robot's current position with 0 so we hopefully move a little bit
        for x in range(6):
            for y in range(6):
                grid[robotGridPos[0]-3+x + (robotGridPos[1]-3+y)*gHeight] = 0

        # search for the closest frontier using bfs
        explored = {}
        # robot position in occupancy grid
        frontier = {start : None}
        foundGoal = False
        t = 0
        curr = None
        prev = None
        while frontier:
            t += 1
            # pop a key, value pair from the dictionary
            curr, prev = frontier.popitem()
            # put the key (representing a point) and the value (representing its parent in the bfs) into the explored dict
            explored[curr] = prev
            # if cell is unknown, we've found where we wanna go!
            if grid[curr[0] + curr[1]*gWidth] == UNKNOWN_COST:
                rospy.loginfo("****** %s nodes traversed ******", t)
                #rospy.loginfo("****** Found frontier in grid at = %s, value =  %s which is (%s, %s) in /map *****" % (curr, gdata[curr[0]+curr[1]*gWidth], curr[0]*grid.info.resolution+grid.info.origin.position.x, curr[0]*grid.info.resolution+grid.info.origin.position.y))
                foundGoal = True
                break

            #rospy.loginfo("****** point value = %s *****" % gdata[curr[0]+curr[1]*gWidth])
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
                if c not in explored and c[0] >= 0 and c[0] < gWidth and c[1] >= 0 and c[1] < gHeight and grid[curr[0] + curr[1]*gWidth] < OG_THRESHOLD:
                    # set the 'prev' of this child to be the current node we're expanding
                    frontier[c] = curr
        if not foundGoal:
            rospy.loginfo(" ****** Couldn't find a frontier! ****** ")
            self.goalPose = None
            self.gridLock.release()
            return False
        
        # at this point we need an already explored point to set as the goal
        # simple solution:
        goal = explored[curr]  # use previous point; it should be explored
        # better solution:
        # basically walk backwards along the path until we're with a certain distance of the robot
        dist = MAX_GOAL_DIST
        while dist >= MAX_GOAL_DIST:
            rospy.loginfo("dist: %s" % dist)
            goal = explored[goal]
            # get euclidean distance and multiply by resolution to get metres
            dist = math.sqrt((goal[0]-start[0])**2 + (goal[1]-start[1])**2) * self.gridMsg.info.resolution

        # set a pose
        self.prevGoal = goal
        self.goalPose = robotPose
        self.goalPose.pose.position.x = (goal[0] * self.gridMsg.info.resolution) + self.gridMsg.info.origin.position.x
        self.goalPose.pose.position.y = (goal[1] * self.gridMsg.info.resolution) + self.gridMsg.info.origin.position.y

        # we don't update the stamp because of weird tranform timing reasons. idk why but commenting this out seems to work
        #self.goalPose.header.stamp = grid.header.stamp
        self.gridLock.release()
        
        # rotate pose to opposite direction (idk if this is correct but it seems to be ok)
        # we do this to create some variance and make sure the rover spins around when it arrives at its location
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

        rospy.loginfo(" ****** Publishing exploration node: %s ******", self.goalPose)
        self.goalPub.publish(self.goalPose)
        return True

    def explore(self):

        while not rospy.is_shutdown():

            if self.startPose is None:
                self.startPose = self.getRobotPose()
            
            elif self.returnHome:
                rospy.loginfo("Returning to start position")
                self.goalPub.publish(self.startPose)
                self.done = True
                return

            elif self.grid is not None and self.gridMsg is not None and self.findGoal():
                rospy.sleep(15)
                continue

            rospy.sleep(2)


if __name__ == "__main__":
    rospy.init_node('explorer_node')
    e = Explorer()
    e.explore()
    rospy.spin()

