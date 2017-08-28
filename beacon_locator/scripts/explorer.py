#!/usr/bin/env python
"""
    Explorer node
    Waits for a start command on crosbot/commands
    Subscribes to a global costmap from move_base topic
    Subscribes to a String on the '/beacon_locator_node/status' topic
    Publishes a PoseStamped to the '/move_base/simple_goal' topic
    First records the initial position of the rover (when this node starts)
    Uses the global costmap published by move_base
    Finds closest unobstructed frontier using breadth first search and sends the robot there
    Once all the beacons are found, it sends the robot back to its initial position
    By: Nuno Das Neves
"""

import rospy
import tf

from crosbot_msgs.msg import ControlCommand, ControlStatus
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate    # wtf
from geometry_msgs.msg import PoseStamped, Pose, Quaternion

from threading import Lock
import math, random, copy

# maximum distance to set a goal
MAX_GOAL_DIST = 1
# only update the goal if we're within this distance of the last goal we set
#MIN_DIST_TO_PREV_GOAL = 0.3

# threshold in the costmap we consider occupied
OG_THRESHOLD = 78
# the value in the costmap representing unknown area (the area we want to explore!)
UNKNOWN_COST = -1

MAP_FRAME = "/comp3431/map"

class Explorer():
    
    def __init__(self):

        self.startPose  = None      # the initial pose of the robot in the map frame
        self.goalPose   = None      # the pose we're currently navigating to

        self.returnHome = False     # denotes that we've found all the beacons
        self.done       = False     # denotes that we're done and don't need to do anything

        self.exploring  = False     # used to pause exploration

        self.rotsLeft   = 2         # when this is > 0, the robot will turn on the spot 180 degrees this many times before continuing

        self.gridLock   = Lock()    # for locking the grid data
        self.grid       = None      # the occupancy grid data as a mutable list
        self.gridMsg    = None      # unmodified occupancy grid message

        self.tfListener = tf.TransformListener()

        self.goalPub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

        self.mapSub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.gotMap)
        self.mapSub = rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, self.gotMapUpdate, queue_size=2)

        self.statusSub = rospy.Subscriber('/beacon_locator_node/status', String, self.gotStatus)

        self.commandSub = rospy.Subscriber('/crosbot/commands', ControlCommand, self.gotCommand)
        self.statusPub = rospy.Publisher("/crosbot/status", ControlStatus, queue_size=10)

    def gotStatus(self, status):

        if status.data == "complete":
            rospy.loginfo("All beacons found")
            self.returnHome = True
    
    def gotMap(self, grid):
        """ This is only called once or twice at the start; from then on the costmap is published as updates """
        if self.done or self.returnHome:
            return

        self.gridLock.acquire()
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

    def getRobotPose(self, xOffset=0):
        """ Get robot pose in map frame """
        if self.tfListener.frameExists("/base_link") and self.tfListener.frameExists(MAP_FRAME):
            try:
                t = self.tfListener.getLatestCommonTime("/base_link", MAP_FRAME)
                p = PoseStamped()
                p.header.frame_id = "/base_link"
                #p.header.stamp = t
                p.pose.position.x = xOffset
                mapPose = self.tfListener.transformPose(MAP_FRAME, p)
                #rospy.loginfo("Robot position is : %s" % mapPose)
                return mapPose
            except Exception as e:
                rospy.logerr(str(e))
        else:
            rospy.logwarn("Waiting for /base_link and %s transforms to exist!" % MAP_FRAME)
            return None 

    def gridToPose(self, aGridTuple):
        aPose = copy.deepcopy(self.robotPose)
        aPose.pose.position.x = (aGridTuple[0] * self.gridMsg.info.resolution) + self.gridMsg.info.origin.position.x
        aPose.pose.position.y = (aGridTuple[1] * self.gridMsg.info.resolution) + self.gridMsg.info.origin.position.y
        return aPose

    def poseToGrid(self, aPose):
        return ( 
            abs(int((aPose.pose.position.x - self.gridMsg.info.origin.position.x) / self.gridMsg.info.resolution)),
            abs(int((aPose.pose.position.y - self.gridMsg.info.origin.position.y) / self.gridMsg.info.resolution))
            )

    def findGoal(self):
        """ Find a point to navigate to based on the map we have """
        
        rospy.loginfo(" ****** Finding a goal! ****** ")
        self.gridLock.acquire()
        grid = self.grid
        gHeight = self.gridMsg.info.height
        gWidth = self.gridMsg.info.width
   
        self.robotPose = self.getRobotPose()

        if self.robotPose is None:
            self.gridLock.release()
            return False

        # change the pose in the map frame into an xy position in the 2d occupancy grid
        start = self.poseToGrid(self.robotPose)

        # THIS IS BAD
        # do a check against the last goal we published; if we're not within a certain distance of it, don't make a new goal
        #if self.prevGoal is not None:
        #    dist = math.sqrt((self.prevGoal[0]-start[0])**2 + (self.prevGoal[1]-start[1])**2) * self.gridMsg.info.resolution
        #    rospy.loginfo(" ****** dist to prevgoal = %s ******" % dist)
        #    if dist > MIN_DIST_TO_PREV_GOAL:
        #        self.gridLock.release()
        #        return True

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
            value = grid[curr[0] + curr[1]*gWidth]

            if value == UNKNOWN_COST:
                # we actually want the previous node; we don't want the goal to be in unknown space
                curr = prev
                #rospy.loginfo("Found node at %s with value %s")
                rospy.loginfo("****** Found unknown node; %s nodes traversed ******", t)
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
                if c not in explored \
                        and c[0] >= 0 and c[0] < gWidth and c[1] >= 0 and c[1] < gHeight \
                        and grid[c[0] + c[1]*gWidth] < OG_THRESHOLD:
                        #and grid[c[0] + c[1]*gWidth] != UNKNOWN_COST:
                    # set the 'prev' of this child to be the current node we're expanding
                    frontier[c] = curr

        # if we can't find unknown space to explore, just go to the furthest away passable point
        # NOTE: This could lead to just wandering the centre of the maze - probably not an issue though
        if not foundGoal:
            rospy.loginfo(" ****** Couldn't find a frontier! Exploring furthest free point ****** ")

        goal = curr

        # basically walk backwards along the path until we're with a certain distance of the robot
        dist = MAX_GOAL_DIST
        while dist >= MAX_GOAL_DIST:
            rospy.loginfo("dist: %s" % dist)
            goal = explored[goal]
            # get euclidean distance and multiply by resolution to get metres
            #rospy.loginfo("goal %s, curr %s, explored[curr] %s, start %s, self.prevGoal %s" %(goal, curr, explored[curr], start, self.prevGoal))
            dist = math.sqrt((goal[0]-start[0])**2 + (goal[1]-start[1])**2) * self.gridMsg.info.resolution

        self.gridLock.release()
        # set a pose

        self.goalPose = self.gridToPose(goal)

        # we don't update the stamp because of weird tranform timing reasons. idk why but commenting this out seems to work
        #self.goalPose.header.stamp = grid.header.stamp

        # we want the yaw to be set to approximately match the direcion of travel
        yawOffset = 0
        if start != goal:  # we must have a previous point to look at, and just sanity check its not the same
            
            prevPose = self.gridToPose(start)
            
            self.goalPose.pose.orientation = Quaternion(0,0,0,1) #reset the rotation; we want to give it an absolute yaw

            # we want the angle from start -> goal
            vec = (
                    self.goalPose.pose.position.x-prevPose.pose.position.x, 
                    self.goalPose.pose.position.y-prevPose.pose.position.y
                ) # direction vector
            # ros knows what its doing somehow and putting these in normally should work (y, x)
            yawOffset = math.atan2(vec[1], vec[0])

            rospy.loginfo(" ****** vec: %s, yaw: %s ******" % (vec, yawOffset))

        # set the prevGoal
        #self.prevGoal = goal

        self.goalPose = self.rotPose(self.goalPose, yawOffset)

        rospy.loginfo(" ****** Publishing exploration node: %s ******", self.goalPose)
        self.goalPub.publish(self.goalPose)
        return True

    def rotPose(self, aPose, rotOffset):
        quat = (
            aPose.pose.orientation.x,
            aPose.pose.orientation.y,
            aPose.pose.orientation.z,
            aPose.pose.orientation.w
            )
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)
        yaw += rotOffset
        # idk if need to do this
        if yaw > 3.14:
            yaw -= 2*3.14
        elif yaw < -3.14:
            yaw += 2*3.14

        quat = tf.transformations.quaternion_from_euler(yaw, pitch, roll, axes='rzyx')
        aPose.pose.orientation = Quaternion(*quat)

        return aPose

    def explore(self):

        while not rospy.is_shutdown():

            """
            aPose = self.getRobotPose()
            if aPose is not None:
                quat = (
                    aPose.pose.orientation.x,
                    aPose.pose.orientation.y,
                    aPose.pose.orientation.z,
                    aPose.pose.orientation.w
                    )
                roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)
                rospy.loginfo(" ****** yaw is %s ******" % yaw)
            """
            """
            if not self.exploring:
                # try to stop move_base from continuing by publishing current pose as goal
                aPose = self.getRobotPose()
                if aPose is not None:
                    self.goalPub.publish(self.goalPose)
                rospy.sleep(5)
                continue
            else:
                rospy.loginfo(" ****** exploring ******")
            """

            if self.startPose is None:
                self.startPose = self.getRobotPose()
                # move forward slightly to fix dwa planner bug (footprint not set error)
                while self.goalPose is None:
                    rospy.sleep(1)
                    self.goalPose = self.getRobotPose(xOffset=0.3)
                    if self.goalPose is not None:
                        self.goalPub.publish(self.goalPose)
                        rospy.sleep(7)
            
            elif self.returnHome:
                rospy.loginfo("Returning to start position")
                self.goalPub.publish(self.startPose)
                self.done = True
                return
            
            # this makes the robot spin a bit at the start to populate the costmap behind its starting location
            elif self.rotsLeft > 0:
                rospy.loginfo(" ****** %s rots left, rotating on the spot! ******" % self.rotsLeft)
                self.goalPose = self.getRobotPose()
                if self.goalPose is not None:
                    self.goalPose = self.rotPose(self.goalPose, 3.14)
                    self.goalPub.publish(self.goalPose)
                    self.rotsLeft -= 1
                    # give it time to do its thing
                    rospy.sleep(12)
                    continue

            elif self.grid is not None and self.gridMsg is not None and self.findGoal():
                rospy.sleep(20)
                continue

            rospy.sleep(2)
    
    def gotCommand(cmd):
        if cmd.command == "command_start":
            self.exploring = True
            self.statusPub.publish(ControlStatus(status="STARTING EXPLORATION"))
        elif cmd.command == "command_stop":
            self.exploring = False
            self.statusPub.publish(ControlStatus(status="STOPPING EXPLORATION"))


if __name__ == "__main__":
    rospy.init_node('explorer_node')
    e = Explorer()
    e.explore()
    rospy.spin()

