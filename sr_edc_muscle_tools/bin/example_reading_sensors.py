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

import rospy
from sr_robot_msgs.msg import JointMusclePositionControllerState

# A callback function for the state messages we are reading.
status = None


def state_cb(data):
    global status  # pylint: disable=W0603
    status = data


rospy.init_node('example_reading_sensors')

# Set up the subscriber so we can read the data, each joint publishes its
# own state.
rospy.Subscriber('/sh_ffj3_muscle_position_controller/state',
                 JointMusclePositionControllerState, state_cb)

print("Try moving ffj3")
rate_speed = rospy.Rate(1)  # hz
while not rospy.is_shutdown():
    if status:
        status_string = (f"ffj3 position:{status.process_value} radians"
                         f" pressure0:{status.muscle_pressure_0}"
                         f" pressure1:{status.muscle_pressure_1}")
        print(status_string)
    rate_speed.sleep()
