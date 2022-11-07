#!/bin/bash
# Copyright 2022 Shadow Robot Company Ltd.
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

DURATION=20000

for muscle in ffj0 ffj3 ffj4 rfj0 rfj3 rfj4 thj2 thj3 thj4 thj5 wrj1 wrj2; do
rostopic pub -1 /sh_"$muscle"_muscle_valve_controller/command sr_robot_msgs/JointMuscleValveControllerCommand "cmd_valve_muscle: [4, -4]
cmd_duration_ms: [$DURATION,$DURATION]"
echo $muscle >> foo.out
sleep 20
rostopic echo -n1 /sh_"$muscle"_muscle_valve_controller/state/muscle_pressure_0 >> foo.out

rostopic pub -1 /sh_"$muscle"_muscle_valve_controller/command sr_robot_msgs/JointMuscleValveControllerCommand "cmd_valve_muscle: [-4, 4]
cmd_duration_ms: [$DURATION,$DURATION]"
sleep 20
rostopic echo -n1 /sh_"$muscle"_muscle_valve_controller/state/muscle_pressure_1 >> foo.out

rostopic pub -1 /sh_"$muscle"_muscle_valve_controller/command sr_robot_msgs/JointMuscleValveControllerCommand "cmd_valve_muscle: [-4, -4]
cmd_duration_ms: [$DURATION,$DURATION]"

done
exit
