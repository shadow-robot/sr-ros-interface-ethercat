#!/usr/bin/env python
import roslib
import rospy
from sr_robot_msgs.msg import sendupdate, joint
from std_msgs.msg import Float64
import math

#
# Small demo of moving a number of joints together.
# Flexes the 2 fingers and then waves the wrist.
#


# We set up all the publishers needed for all joints in a dict and create a simple
# sendupdate function to send joint targets from a dictioary of joint name and angle
# (in degrees)
# Note naist hand is only 2 fingers and a thumb
joint_names = ["thj1", "thj2", "thj3", "thj4", "thj5", "ffj0",
               "ffj3", "ffj4", "rfj0", "rfj3", "rfj4", "wrj1", "wrj2"]
joint_pubs = {}
for jname in joint_names:
    topic = "/sh_" + jname + "_muscle_position_controller/command"
    print topic
    joint_pubs[jname] = rospy.Publisher(topic, Float64, latch=True)


def sendupdate(joints):
    print "Sending:"
    for jname in joints:
        if jname not in joint_pubs:
            print "\tJoint %s not found" % jname
            return
        msg = Float64()
        msg.data = math.radians(float(joints[jname]))
        print "\t" + jname + ": " + str(joints[jname])
        joint_pubs[jname].publish(msg)

rospy.init_node('example_sendupdate')

start_pose = {'ffj0': 27.0, 'ffj3': 0, 'ffj4': 0,
              'rfj0': 40.0, 'rfj3': 0, 'rfj4': 0,
              'thj1': 20, 'thj2': 31, 'thj3': 0, 'thj4': 25, 'thj5': -29,
              'wrj1': 0, 'wrj2': 0
              }

print "Start"
sendupdate(start_pose)
rospy.sleep(2)

# FF curl
sendupdate({'ffj3': 90})
rospy.sleep(3)
sendupdate({'ffj0': 120})
rospy.sleep(2)
sendupdate(start_pose)
rospy.sleep(4)

# RF curl
sendupdate({'rfj0': 120, 'rfj3': 90})
rospy.sleep(4)
sendupdate(start_pose)
rospy.sleep(4)

# Back to the start
sendupdate(start_pose)
rospy.sleep(2)

# Wrist wave
sendupdate({'wrj2': 10})
rospy.sleep(6)
sendupdate({'wrj2': -20})
rospy.sleep(6)
sendupdate({'wrj2': 0})
rospy.sleep(4)

# Back to the start
sendupdate(start_pose)

rospy.sleep(2)
