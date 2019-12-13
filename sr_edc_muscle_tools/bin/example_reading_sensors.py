#!/usr/bin/env python
#
# Copyright 2019 Shadow Robot Company Ltd.
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

import roslib
import rospy
from std_msgs.msg import Float64
from sr_robot_msgs.msg import JointMusclePositionControllerState
import math

# A callback function for the state messages we are reading.
status = None


def state_cb(data):
    global status
    status = data

rospy.init_node('example_reading_sensors')

# Set up the subscriber so we can read the data, each joint publishes its
# own state.
rospy.Subscriber('/sh_ffj3_muscle_position_controller/state',
                 JointMusclePositionControllerState, state_cb)

print "Try moving ffj3"
r = rospy.Rate(1)  # hz
while not rospy.is_shutdown():
    if status:
        print "ffj3 position:%s radians  pressure0:%s pressure1:%s" % (status.process_value,
                                                                       status.muscle_pressure_0,
                                                                       status.muscle_pressure_1)
    r.sleep()
