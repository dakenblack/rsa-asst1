#!/usr/bin/env python
"""
    Explorer node
    Waits for a start command on crosbot/commands
    Subscribes to a global costmap from 'move_base/global_costmap/costmap'
    Subscribes to a String on the '/beacon_locator_node/status'
    Publishes a PoseStamped to the '/move_base/simple_goal
    Records the initial position of the rover (when this node starts)
    Moves forward slightly to allow DWA planner to start
    Rotates 180 degrees, waits, then 180 degrees so the area behind it can be mapped
    Finds closest unobstructed frontier using breadth first search and sends the robot there
    Once all the beacons are found, it sends the robot back to its initial position
    If there are no unknown areas to explore, navigate to the furthest explored area
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

# threshold in the costmap we consider occupied (our BFS will not expand nodes with this cost or higher)
OG_THRESHOLD = 78
# the value in the costmap representing unknown area (the area we want to explore!)
UNKNOWN_COST = -1

# transorm frame of the map
MAP_FRAME = "/comp3431/map"

class Explorer():
    
    def __init__(self):

        self.startPose  = None      # the initial pose of the robot in the map frame
        self.goalPose   = None      # the pose we're currently navigating to

        self.returnHome = False     # denotes that we've found all the beacons
        self.done       = False     # denotes that we're done and don't need to do anything

        self.exploring  = False     # used to pause exploration

        self.rotsLeft   = 2         # when this is > 0, the robot will turn on the spot this many times in this many increments

        self.gridLock   = Lock()    # for locking the grid data; we don't want to calculate a goal on a map that's changing!
        self.grid       = None      # the occupancy grid data as a mutable list
        self.gridMsg    = None      # unmodified occupancy grid message

        self.tfListener = tf.TransformListener()

        self.goalPub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

        self.mapSub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.gotMap, queue_size=1)
        self.mapSub = rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, self.gotMapUpdate, queue_size=1)

        self.statusSub = rospy.Subscriber('/beacon_locator_node/status', String, self.gotStatus)

        self.commandSub = rospy.Subscriber('/crosbot/commands', ControlCommand, self.gotCommand)
        self.statusPub = rospy.Publisher("/crosbot/status", ControlStatus, queue_size=10)

    def gotStatus(self, status):
        """ callback for the beacon status message; tells the robot to return home """

        if status.data == "complete":
            rospy.loginfo("All beacons found")
            self.returnHome = True
    
    def gotMap(self, grid):
        """ Callback for map. This is only called once at the start; from then on the costmap is published as updates """
        if self.done or self.returnHome:
            return

        self.gridLock.acquire()
        self.gridMsg = grid
        self.grid = list(grid.data)
        self.gridLock.release() 
    
    def gotMapUpdate(self, grid):
        """ Callback for map updates. An  update may or may not contain the whole map """
        if self.done or self.returnHome:
            return

        # acquire the lock so we don't interfere with exploration!
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
        """ Converts a tuple representing an x and y in the grid to a pose in the map frame """
        aPose = copy.deepcopy(self.robotPose)
        aPose.pose.position.x = (aGridTuple[0] * self.gridMsg.info.resolution) + self.gridMsg.info.origin.position.x
        aPose.pose.position.y = (aGridTuple[1] * self.gridMsg.info.resolution) + self.gridMsg.info.origin.position.y
        return aPose

    def poseToGrid(self, aPose):
        """ Converts a pose in the map frame to a tuple containing the x and y in the OccupancyGrid """
        return ( 
            abs(int((aPose.pose.position.x - self.gridMsg.info.origin.position.x) / self.gridMsg.info.resolution)),
            abs(int((aPose.pose.position.y - self.gridMsg.info.origin.position.y) / self.gridMsg.info.resolution))
            )

    def findGoal(self):
        """ Find a point to navigate to based on the map we have, and publish it """
        
        rospy.loginfo(" ****** Finding a goal! ****** ")
        # lock the grid so it can't be updated while we're searching it!
        self.gridLock.acquire()
        grid = self.grid
        # these are just for readability
        gHeight = self.gridMsg.info.height
        gWidth = self.gridMsg.info.width
   
        # we need the current pose of the robot before we can do anything
        self.robotPose = self.getRobotPose()

        if self.robotPose is None:
            self.gridLock.release()
            return False

        # change the pose in the map frame into an xy position in the 2d occupancy grid
        # we represent grid coordinates as a tuple of x, y
        start = self.poseToGrid(self.robotPose)

        # search for the closest frontier using bfs
        # we'll store nodes we've explored, mapped to their parent
        explored = {}
        # and which nodes we've yet to expand
        frontier = {start : None}

        foundGoal = False # use this to determine if the bfs was successful
        t = 0 # debugging counter to show how many nodes were traversed
        curr = None
        prev = None

        # while there are unexplored nodes
        while frontier:
            t += 1
            # pop a key, value pair from the dictionary
            curr, prev = frontier.popitem()
            # put the key (representing a point) and the value (representing its parent in the bfs) into the explored dict
            explored[curr] = prev
            # get the occupancy grid value of the node
            value = grid[curr[0] + curr[1]*gWidth]

            # we exit as soon as we find a node with 'unknown' cost - these are where we want the robot to go!
            if value == UNKNOWN_COST:
                # we actually want the previous node; we don't want the goal to be in unknown space
                curr = prev
                #rospy.loginfo("Found node at %s with value %s")
                rospy.loginfo("****** Found unknown node; %s nodes traversed ******", t)
                foundGoal = True
                break

            # create a list of all potential children of this node (one for each direction)
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
                    # set the 'prev' of this child to be the current node we're expanding
                    frontier[c] = curr

        self.gridLock.release()

        # if we can't find unknown space to explore, just go to the furthest away passable point
        # NOTE: This can lead to wandering the centre of the maze
        if not foundGoal:
            rospy.loginfo(" ****** Couldn't find a frontier! Exploring furthest free point ****** ")

        goal = curr

        # walk backwards along the path until we're with a certain distance of the robot
        dist = MAX_GOAL_DIST
        while dist >= MAX_GOAL_DIST:
            # get parent of goal
            goal = explored[goal]
            # handle an edge case (only 1 node expanded)
            if goal is None:
                return True
            # get euclidean distance and multiply by grid resolution to get metres
            dist = math.sqrt((goal[0]-start[0])**2 + (goal[1]-start[1])**2) * self.gridMsg.info.resolution

        # set a pose
        self.goalPose = self.gridToPose(goal)

        # we don't update the stamp because of weird tranform timing reasons; move_base complains less if we comment this out
        #self.goalPose.header.stamp = grid.header.stamp

        # we want the yaw to be set to approximately match the direcion of travel; this makes move_base perform better
        yawOffset = 0

        # we need the start and end to be different for this to be meaningful
        if start != goal:
            
            prevPose = self.gridToPose(start)
            
            self.goalPose.pose.orientation = Quaternion(0,0,0,1) #reset the rotation; we want to give it an absolute yaw

            # get the vector from prev -> goal
            vec = (
                    self.goalPose.pose.position.x-prevPose.pose.position.x, 
                    self.goalPose.pose.position.y-prevPose.pose.position.y
                )
            # ros knows what its doing somehow and putting these in normally works (y, x)
            yawOffset = math.atan2(vec[1], vec[0]) # atan2 requires no special conversions for ros; returns radians between -3.14 and 3.14

            rospy.loginfo(" ****** vec: %s, yaw: %s ******" % (vec, yawOffset))

        # convert the radians back into a Quaternion
        self.goalPose = self.rotPose(self.goalPose, yawOffset)

        rospy.loginfo(" ****** Publishing exploration node: %s ******", self.goalPose)
        self.goalPub.publish(self.goalPose)
        return True

    def rotPose(self, aPose, rotOffset):
        """ Rotate a pose (in Z) by an offset (passed as radians) """

        # first convert the quaternion to an euler angle so it is easier to manipulate
        quat = (
            aPose.pose.orientation.x,
            aPose.pose.orientation.y,
            aPose.pose.orientation.z,
            aPose.pose.orientation.w
            )
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)

        # apply the offset
        yaw += rotOffset

        # just a check to make sure we're putting in a sane value
        if yaw > 3.14:
            yaw -= 2*3.14
        elif yaw < -3.14:
            yaw += 2*3.14

        # convert back to a quaternion that ros can recognise
        quat = tf.transformations.quaternion_from_euler(yaw, pitch, roll, axes='rzyx')
        aPose.pose.orientation = Quaternion(*quat)

        return aPose

    def explore(self):
        """ 'main loop' of the node; manages high level behaviour of the robot """

        while not rospy.is_shutdown():

            # check to see if we're actually supposed to be exploring
            if not self.exploring:
                # stop move_base from continuing by publishing current pose as goal
                aPose = self.getRobotPose()
                if aPose is not None:
                    self.goalPub.publish(self.goalPose)
                rospy.sleep(5)
                continue
            else:
                rospy.loginfo(" ****** exploring ******")

            # when we first start up, we need to save the starting pose
            if self.startPose is None:
                self.startPose = self.getRobotPose()
                # move forward slightly to fix dwa planner bug (footprint not set error)
                while self.goalPose is None:
                    rospy.sleep(1)
                    self.goalPose = self.getRobotPose(xOffset=0.35)
                    if self.goalPose is not None:
                        for i in range(6):
                            self.goalPub.publish(self.goalPose)
                            rospy.sleep(2)
            
            # if it's time to go back to the start, simply publish the start position
            elif self.returnHome:
                rospy.loginfo("Returning to start position")
                self.goalPub.publish(self.startPose)
                self.done = True
                rospy.sleep(5)
            
            # this makes the robot spin 180 degrees twice to populate the costmap behind its starting location
            elif self.rotsLeft > 0:
                rospy.loginfo(" ****** %s rots left, rotating on the spot! ******" % self.rotsLeft)
                self.goalPose = self.getRobotPose()
                if self.goalPose is not None:
                    # rotation is radians
                    self.goalPose = self.rotPose(self.goalPose, 3.14)

                    self.goalPub.publish(self.goalPose)
                    self.rotsLeft -= 1
                    # give it time to do its thing
                    rospy.sleep(10)

            # we need to make sure there is a map, and we only do a long sleep if findGoal() was successful 
            elif self.grid is not None and self.gridMsg is not None and self.findGoal():
                rospy.sleep(15)

            rospy.sleep(2)
    
    # read a command from the crosbot controls
    def gotCommand(cmd):
        if cmd.command == "command_start":
            self.exploring = True
            self.statusPub.publish(ControlStatus(status="STARTING EXPLORATION"))
        elif cmd.command == "command_stop":
            self.exploring = False
            self.statusPub.publish(ControlStatus(status="STOPPING EXPLORATION"))


if __name__ == "__main__":
    # start the node and call explore()
    rospy.init_node('explorer_node')
    e = Explorer()
    e.explore()
    rospy.spin()

