#!/usr/bin/env python3
#
# Copyright 2019, 2022 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

import math
import rospy
from sr_robot_msgs.msg import sendupdate
from std_msgs.msg import Float64

# Small demo of moving a number of joints together.
# Flexes the 2 fingers and then waves the wrist.


# We set up all the publishers needed for all joints in a dict and create a simple
# sendupdate function to send joint targets from a dictioary of joint name and angle
# (in degrees)
# Note naist hand is only 2 fingers and a thumb
joint_names = ["thj1", "thj2", "thj3", "thj4", "thj5", "ffj0",
               "ffj3", "ffj4", "rfj0", "rfj3", "rfj4", "wrj1", "wrj2"]
joint_pubs = {}
for jname in joint_names:
    topic = "/sh_" + jname + "_muscle_position_controller/command"
    print(topic)
    joint_pubs[jname] = rospy.Publisher(topic, Float64, latch=True)


def sendupdate(joints):  # pylint: disable=E0102
    print("Sending:")
    for jointname in joints:
        if jointname not in joint_pubs:
            print(f"\tJoint {jointname} not found")
            return
        msg = Float64()
        msg.data = math.radians(float(joints[jointname]))
        print(f"\t{jointname}: {str(joints[jointname])}")
        joint_pubs[jointname].publish(msg)


rospy.init_node('example_sendupdate')

start_pose = {'ffj0': 27.0, 'ffj3': 0, 'ffj4': 0,
              'rfj0': 40.0, 'rfj3': 0, 'rfj4': 0,
              'thj1': 20, 'thj2': 31, 'thj3': 0, 'thj4': 25, 'thj5': -29,
              'wrj1': 0, 'wrj2': 0
              }

print("Start")
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
