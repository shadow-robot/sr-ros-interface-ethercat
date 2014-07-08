#! /usr/bin/python
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import with_statement

import rospy

from pr2_mechanism_msgs.msg import MechanismStatistics
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import math

import threading
mutex = threading.Lock()

has_warned_invalid = False

# Math.isnan is new in python 2.6, need to check version
import sys
check_nan = False
my_version = sys.version_info
if my_version[0] == 2 and my_version[1] >= 6:
    check_nan = True

def joint_to_diag(js):
    global has_warned_invalid
    ds = DiagnosticStatus()
    ds.level = DiagnosticStatus.OK
    ds.message = 'OK'

    # Hack to stop gripper joints from being "uncalibrated"
    if not js.is_calibrated and js.name.find("float") < 0 and js.name.find("finger") < 0:
        ds.level = DiagnosticStatus.WARN
        ds.message = 'Uncalibrated'

    if check_nan and (math.isnan(js.position) or math.isnan(js.velocity) or \
            math.isnan(js.measured_effort) or math.isnan(js.commanded_effort)):
        ds.level = DiagnosticStatus.ERROR
        ds.message = 'NaN in joint data'
        if not has_warned_invalid:
            rospy.logerr("NaN value for joint data. controller_manager restart required.")
            has_warned_invalid = True

    if check_nan and (math.isinf(js.position) or math.isinf(js.velocity) or \
            math.isinf(js.measured_effort) or math.isinf(js.commanded_effort)):
        ds.level = DiagnosticStatus.ERROR
        ds.message = 'Inf in joint data'
        if not has_warned_invalid:
            rospy.logerr("Infinite value for joint data. controller_manager restart required.")
            has_warned_invalid = True

    ds.name = "Joint (%s)" % js.name
    ds.values = [
        KeyValue('Position', str(js.position)),
        KeyValue('Velocity', str(js.velocity)),
        KeyValue('Measured Effort', str(js.measured_effort)),
        KeyValue('Commanded Effort', str(js.commanded_effort)),
        KeyValue('Calibrated', str(js.is_calibrated)),
        KeyValue('Odometer', str(js.odometer)),
        KeyValue('Max Position', str(js.max_position)),
        KeyValue('Min Position', str(js.min_position)),
        KeyValue('Max Abs. Velocity', str(js.max_abs_velocity)),
        KeyValue('Max Abs. Effort', str(js.max_abs_effort)),
        KeyValue('Limits Hit', str(js.violated_limits)) ]
    
    return ds


rospy.init_node('joints_to_diagnostics')

last_msg = None
last_update_time = 0
def state_cb(msg):
    with mutex:
        global last_msg, last_update_time
        last_msg = msg
        last_update_time = rospy.get_time()

def publish_diags():
    with mutex:
        global last_msg, last_update_time
        if last_msg is None:
            return
        
        # Don't publish anything without updates
        if rospy.get_time() - last_update_time > 3:
            return
        
        d = DiagnosticArray()
        d.header.stamp = last_msg.header.stamp
        if last_msg.joint_statistics == []:
            ds = DiagnosticStatus()
            ds.level = 0
            ds.message = 'No Joint states from controller manager'
            ds.name = "Joints: none"
            d.status = [ds]
        else:
            d.status = [joint_to_diag(js) for js in last_msg.joint_statistics]
        pub_diag.publish(d)

pub_diag = rospy.Publisher('/diagnostics', DiagnosticArray)
rospy.Subscriber('mechanism_statistics', MechanismStatistics, state_cb)

my_rate = rospy.Rate(1.0)
while not rospy.is_shutdown():
    my_rate.sleep()
    publish_diags()


