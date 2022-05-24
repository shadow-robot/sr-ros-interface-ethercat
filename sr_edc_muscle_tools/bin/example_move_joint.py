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
from std_msgs.msg import Float64

rospy.init_node('example1')

# Move a joint, send position.
# NB Load position controllers first

# Create  publisher for the joints controller
ffj3_pub = rospy.Publisher(
    "/sh_ffj3_muscle_position_controller/command", Float64, latch=True)

# Send targets
ffj3_pub.publish(0)
rospy.sleep(2)
ffj3_pub.publish(math.radians(90))
rospy.sleep(6)

# Using explicit message
msg = Float64()
msg.data = 0
ffj3_pub.publish(msg)
rospy.sleep(2)
