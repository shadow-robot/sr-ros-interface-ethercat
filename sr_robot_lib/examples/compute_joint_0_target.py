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

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class Merger:

    def __init__(self):
        self.subscriber = rospy.Subscriber(
            "/joint_states", JointState, self.callback)
        self.publisher = rospy.Publisher("/lfj0_target", Float64)

    def callback(self, data):
        msg_to_send = Float64()

        msg_to_send.data = data.position[17] + data.position[18]

        self.publisher.publish(msg_to_send)


if __name__ == '__main__':
    rospy.init_node("merger", anonymous=True)
    merger = Merger()
    rospy.spin()
