#!/bin/bash

DURATION=$2

rostopic pub -1 /sh_"$1"_muscle_valve_controller/command sr_robot_msgs/JointMuscleValveControllerCommand "cmd_valve_muscle: [-4, -4]
cmd_duration_ms: [$DURATION,$DURATION]"

exit
