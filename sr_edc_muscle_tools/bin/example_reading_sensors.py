#!/usr/bin/env python
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
