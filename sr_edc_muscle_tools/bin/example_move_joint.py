#!/usr/bin/env python
import roslib
import rospy
from std_msgs.msg import Float64
import math

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
