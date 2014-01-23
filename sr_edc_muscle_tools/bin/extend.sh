#!/bin/bash

if [ $# -ne 2 ]; then
	echo "Usage: $0 MUSCLE DURATION_MS";
	echo
	echo "    " MUSCLE is one of: ffj0 ffj3 ffj4 rfj0 rfj3 rfj4 thj2 thj3 thj4 thj5 wrj1 wrj2
	echo 
	echo "    " Note that valve controllers need to be running.
        echo "    " Run with: rosrun sr_edc_muscle_tools start_valve_controllers.sh 
	echo
	exit 1;
fi
DURATION=$2

rostopic pub -1 /sh_"$1"_muscle_valve_controller/command sr_robot_msgs/JointMuscleValveControllerCommand "cmd_valve_muscle: [-4, 4]
cmd_duration_ms: [$DURATION,$DURATION]" 

exit
