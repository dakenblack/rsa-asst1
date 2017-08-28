#!/usr/bin/env python

import rospy
from crosbot_msgs.msg import ControlCommand, ControlStatus

# To CMakeLists.txt add:
# find_package(catkin REQUIRED COMPONENTS
#   ...
#   crosbot_msgs
#   ...
# )

commandRec = None
commandPub = None

def commandReceiverCallback(cmd):
	if cmd.command == "command_start":
		print "START"
		sendStatus("START RECEIVED")
	elif cmd.command == "command_stop":
		print "STOP"
		sendStatus("STOP RECEIVED")
	elif cmd.command == "command_reset":
		print "RESET"
		sendStatus("RESET RECEIVED")

def sendStatus(msg):
	status = ControlStatus()
	status.status = msg
	commandPub.publish(status)

if __name__ == '__main__':
	rospy.init_node('test_commands') # create node
	commandRec = rospy.Subscriber('/crosbot/commands', ControlCommand, commandReceiverCallback)
	commandPub = rospy.Publisher("/crosbot/status", ControlStatus, queue_size=10)
	rospy.spin()
